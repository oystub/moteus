#pragma once


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
    class Config;
    class State;
public:
    Rotor(moteus::MoteusController** controller) : 
        controller_(controller)
    {
        cmd_.mode = moteus::kStopped; // Start in stopped mode
        cmd_.position = std::numeric_limits<float>::quiet_NaN(); // We never use position setpoints
        cmd_.timeout_s = std::numeric_limits<float>::quiet_NaN(); // We never use timeouts
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
        // We treat negative or zero values as 0.
        // Otherwise, we linearly scale the value to be between the min and max RPM values
        // such that 1 => config_.rpm_min and 8191 => config_.rpm_max

        if (msg.cmd.len <= config_.esc_index){
            // The command is not for this ESC
            return;
        }
        auto raw_command = msg.cmd.data[config_.esc_index];

        float rpm_command{0};
        if (raw_command > 0){
            // Scale the raw command to be bewteen the min and max RPM values
            // TODO: Reserve some speed for sinusoidal control (or do we put this on top of the max RPM?)
            rpm_command = (static_cast<float>(raw_command - 1) / 8190.0f) * (config_.rpm_max - config_.rpm_min) + config_.rpm_min;
        }
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
                state_.commanded_elevation = clamped_value * config_.velocity_elevation_gain;
            } else if (command.actuator_id == config_.azimuth_index){
                state_.commanded_azimuth = clamped_value * moteus::kPi;
            }
        }
    }

private:
    struct Config {
        uint8_t esc_index{0};
        uint8_t elevation_index{0};
        uint8_t azimuth_index{1};
        float rpm_min{100};
        float rpm_max{10000};
        float azimuth_offset_deg{0};
        float velocity_elevation_gain{0.1};

        // Dronecan parameter handling
        template <typename Store>
        void RegisterParameters(Store& store) {
            DRONECAN_PARAMETER(ROT_ESC_IDX, esc_index, 0, 0, 19);
            DRONECAN_PARAMETER(ROT_ELV_IDX, elevation_index, 0, 0, 255);
            DRONECAN_PARAMETER(ROT_AZM_IDX, azimuth_index, 1, 0, 255);
            DRONECAN_PARAMETER(ROT_RPM_MIN, rpm_min, 100, 0, 10000);
            DRONECAN_PARAMETER(ROT_RPM_MAX, rpm_max, 10000, 0, 10000);
            DRONECAN_PARAMETER(ROT_AZM_OFF, azimuth_offset_deg, 0, -180, 180);
            DRONECAN_PARAMETER(ROT_VEL_ELV_GAIN, velocity_elevation_gain, 0.1, 0, 0.5);
        }

        // Moteus serialization
        template <typename Archive>
        void Serialize(Archive* a) {
            a->Visit(MJ_NVP(esc_index));
            a->Visit(MJ_NVP(elevation_index));
            a->Visit(MJ_NVP(azimuth_index));
            a->Visit(MJ_NVP(rpm_min));
            a->Visit(MJ_NVP(rpm_max));
            a->Visit(mjlib::base::MakeNameValuePair(&azimuth_offset_deg, "azimuth_offset"));
            a->Visit(MJ_NVP(velocity_elevation_gain));
        } 
    } config_;

    struct State {
        bool disarmed{true};
        float commanded_rps{0};
        float commanded_azimuth{0};
        float commanded_elevation{0};
    } state_;

    moteus::BldcServoCommandData cmd_{};
    moteus::MoteusController** controller_;

    void handleVelocityCommand(float rpm_command){
        // If command is negative or zero, keep it at zero
        if (rpm_command <= 0){
            state_.commanded_rps = 0.f;
        } else {
            // Else, clamp it to the min and max values
            const auto clamped_rpm = std::clamp(rpm_command, config_.rpm_min, config_.rpm_max);
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
            cmd_.sinusoidal_velocity_scale = state_.commanded_elevation;
            cmd_.sinusoidal_velocity_phase = state_.commanded_azimuth;
        }
        if (controller_ == nullptr || *controller_ == nullptr){
            return;
        }
        (*controller_)->bldc_servo()->Command(cmd_);
    }

    Canard::ObjCallback<Rotor, uavcan_equipment_esc_RawCommand> raw_command_cb {this, &Rotor::handle_esc_RawCommand};
    Canard::Subscriber<uavcan_equipment_esc_RawCommand> raw_command_sub{raw_command_cb, 0};
    Canard::ObjCallback<Rotor, uavcan_equipment_actuator_ArrayCommand> array_command_cb {this, &Rotor::handle_actuator_ArrayCommand};
    Canard::Subscriber<uavcan_equipment_actuator_ArrayCommand> array_command_sub{array_command_cb, 0};
};