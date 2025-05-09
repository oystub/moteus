#pragma once

#include <algorithm>
#include <cmath>
#include <array>

#include "mjlib/base/visitor.h"
#include "mjlib/micro/persistent_config.h"

#include "dronecan.h"
#include "dronecan_param.h"
#include "dronecan_param_store.h"
#include "math.h"
#include "moteus_controller.h"

#include <canard/subscriber.h>
#include <uavcan.equipment.esc.RawCommand.h>
#include <uavcan.equipment.esc.RPMCommand.h>
#include <uavcan.equipment.actuator.ArrayCommand.h>


class Rotor {
public:
    struct Config;
    struct State;
    {
        cmd_.mode = moteus::kStopped; // Start in stopped mode
        cmd_.position = std::numeric_limits<float>::quiet_NaN(); // We never use position setpoints
        cmd_.timeout_s = std::numeric_limits<float>::quiet_NaN(); // We never use timeouts

        // Register the parameter stores
        param_store->Register(&config_);
        persistent_config->Register("rotor", &config_, [](){});
    }

    Config* config(){
        return &config_;
    }

    void Process(){
        if (isTimeout()){
            cmd_.mode = moteus::kStopped;
        }
    }

    void handle_esc_RawCommand(const CanardRxTransfer& transfer, const uavcan_equipment_esc_RawCommand& msg){
        // RawCommand contains a value in the range [-8192, 8191]
        // We treat negative values as 0.

        if (msg.cmd.len <= config_.esc_index){
            // The command is not for this ESC
            return;
        }
        auto raw_command = msg.cmd.data[config_.esc_index];
        float thrust_command = static_cast<float>(std::clamp(raw_command, int16_t{0}, int16_t{8191})) / 8191.f; // Normalize to [0, 1]

        // For now, we assume a quadratic relationship between thrust and speed
        auto rpm_command = thrustToRpm(thrust_command);

        handleVelocityCommand(rpm_command);
        sendMotorCommand();
    }

    void handle_actuator_ArrayCommand(const CanardRxTransfer& transfer, const uavcan_equipment_actuator_ArrayCommand& msg){
        // There is no guarantee for the orderingof ActuatorCommands in the ArrayCommand
        // We check all commands and only use the ones that are meant for this rotor

        // Don't trust the specified length field
        size_t len = std::min(static_cast<size_t>(msg.commands.len), sizeof(msg.commands.data) / sizeof(msg.commands.data[0]));

        for (size_t i = 0; i < len; ++i){
            const auto& command = msg.commands.data[i];
            if (command.command_type != UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_UNITLESS){
                // We only accept unitless commands
                continue;
            }
            // Command should be in range [-1, 1], according to spec. We clamp it to be sure.
            const float clamped_value = std::clamp(command.command_value, -1.f, 1.f);
            if (command.actuator_id == config_.elevation_index){
                state_.commanded_elevation_rad = clamped_value * config_.max_elevation_deg * moteus::kPi / 180.f;
            } else if (command.actuator_id == config_.azimuth_index){
                state_.commanded_azimuth_rad = clamped_value * moteus::kPi;
            }
        }
    }

    struct Config {
        uint8_t esc_index{0};
        uint8_t elevation_index{0};
        uint8_t azimuth_index{1};
        float rpm_min{100};
        float rpm_max{10000};
        float azimuth_offset_deg{0};
        float elevation_gain_per_deg{0.01};
        float max_elevation_deg{20};
        enum class OperatingMode : uint8_t {
            kPolar,
            kEuclidean,
        } operating_mode { OperatingMode::kPolar };


        // Dronecan parameter handling
        template <typename Store>
        void RegisterParameters(Store& store) {
            DRONECAN_PARAMETER(ROT_ESC_IDX, esc_index, 0, 0, 19);
            DRONECAN_PARAMETER(ROT_ELV_IDX, elevation_index, 0, 0, 255);
            DRONECAN_PARAMETER(ROT_AZM_IDX, azimuth_index, 1, 0, 255);
            DRONECAN_PARAMETER(ROT_RPM_MAX, rpm_max, 10000, 0, 10000);
            DRONECAN_PARAMETER(ROT_AZM_OFF, azimuth_offset_deg, 0, -180, 180);
            DRONECAN_PARAMETER(ROT_ELV_GAIN, elevation_gain_per_deg, 0.01, 0, 0.1);
            DRONECAN_PARAMETER(ROT_MAX_ELV, max_elevation_deg, 20, 0, 90);
            DRONECAN_PARAMETER(ROT_CMD_MODE, (uint8_t&)operating_mode, 0, 0, 1);
        }

        // Moteus serialization
        template <typename Archive>
        void Serialize(Archive* a) {
            a->Visit(MJ_NVP(esc_index));
            a->Visit(MJ_NVP(elevation_index));
            a->Visit(MJ_NVP(azimuth_index));
            a->Visit(MJ_NVP(rpm_max));
            a->Visit(mjlib::base::MakeNameValuePair(&azimuth_offset_deg, "azimuth_offset"));
            a->Visit(MJ_NVP(elevation_gain_per_deg));
            a->Visit(MJ_NVP(max_elevation_deg));
            a->Visit(MJ_NVP(operating_mode));
        } 
    } config_;

    struct State {
        bool disarmed{true};
        float commanded_rps{0};
        float commanded_azimuth_rad{0};
        float commanded_elevation_rad{0};
    } state_;

    moteus::BldcServoCommandData cmd_{};
    moteus::MoteusController* controller_;

    void handleVelocityCommand(float rpm_command){
        // If command is negative or zero, keep it at zero
        if (rpm_command <= 0){
            state_.commanded_rps = 0.f;
        } else {
            // Else, clamp it to the min and max values
            const auto clamped_rpm = std::clamp(rpm_command, 0.f, config_.rpm_max);
            state_.commanded_rps = clamped_rpm / 60.0f;
        }
    }

    // Check that we get commands at the required rate
    bool isTimeout(){
        return false;
    }

    void sendMotorCommand(){
        if (state_.commanded_rps == 0){
            // A command of 0 means stop the motor.
            // kStop lets the motor coast to a stop
            // kBrake would short the motor terminals to stop it faster
            cmd_.mode = moteus::kStopped;
        } else {
            cmd_.mode = moteus::BldcServoMode::kSinusoidalVelocity;
            cmd_.velocity = state_.commanded_rps;
            cmd_.sinusoidal_velocity_scale = modulationGain(state_.commanded_elevation_rad, state_.commanded_rps);
            cmd_.sinusoidal_velocity_phase = state_.commanded_azimuth_rad;
        }
        controller_->bldc_servo()->Command(cmd_);
    }

    // Map a [0, 1] thrust value to an RPM value
    float thrustToRpm(float thrust){
        return config_.rpm_max * std::sqrt(thrust);
    }
    
    // From a given elevation and speed, calculate the gain for the modulation
    float modulationGain(float elevation, float speed){
        return config_.elevation_gain_per_deg * (180.f / moteus::kPi) * elevation;
    }

    Canard::ObjCallback<Rotor, uavcan_equipment_esc_RawCommand> raw_command_cb {this, &Rotor::handle_esc_RawCommand};
    Canard::Subscriber<uavcan_equipment_esc_RawCommand> raw_command_sub{raw_command_cb, 0};
    Canard::ObjCallback<Rotor, uavcan_equipment_actuator_ArrayCommand> array_command_cb {this, &Rotor::handle_actuator_ArrayCommand};
    Canard::Subscriber<uavcan_equipment_actuator_ArrayCommand> array_command_sub{array_command_cb, 0};
};

namespace mjlib {
namespace base {
    template <>
    struct IsEnum<Rotor::Config::OperatingMode> {
    static constexpr bool value = true;

    using P = Rotor::Config::OperatingMode;
    static std::array<std::pair<P, const char*>, 2> map() {
        return {{
            { P::kPolar, "polar" },
            { P::kEuclidean, "euclidean" },
        }};
    }
    };
}}