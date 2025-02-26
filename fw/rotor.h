#pragma once

#include "mjlib/base/visitor.h"
#include "mjlib/micro/persistent_config.h"

#include "dronecan.h"
#include "dronecan_param.h"
#include "dronecan_param_store.h"
#include "moteus_controller.h"

#include <canard/subscriber.h>
#include <uavcan.equipment.esc.RawCommand.h>
#include <uavcan.equipment.esc.RPMCommand.h>


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
            rpm_command = (static_cast<float>(raw_command - 1) / 8190.0f) * (config_.rpm_max - config_.rpm_min) + config_.rpm_min;
        }
        handleVelocityCommand(rpm_command);
        sendMotorCommand();
    }

private:
    struct Config {
        uint8_t esc_index = 0;
        float rpm_min = 100;
        float rpm_max = 10000;
        float azimuth_offset_deg = 0;
        float velocity_elevation_gain = 0.1;

        // Dronecan parameter handling
        template <typename Store>
        void RegisterParameters(Store& store) {
            DRONECAN_PARAMETER(ROT_ESC_IDX, esc_index, 0, 0, 19);
            DRONECAN_PARAMETER(ROT_RPM_MIN, rpm_min, 100, 0, 10000);
            DRONECAN_PARAMETER(ROT_RPM_MAX, rpm_max, 10000, 0, 10000);
            DRONECAN_PARAMETER(ROT_AZM_OFF, azimuth_offset_deg, 0, -180, 180);
            DRONECAN_PARAMETER(ROT_VEL_ELV_GAIN, velocity_elevation_gain, 0.1, 0, 0.5);
        }

        // Moteus serialization
        template <typename Archive>
        void Serialize(Archive* a) {
            a->Visit(MJ_NVP(esc_index));
            a->Visit(MJ_NVP(rpm_min));
            a->Visit(MJ_NVP(rpm_max));
            a->Visit(mjlib::base::MakeNameValuePair(&azimuth_offset_deg, "azimuth_offset"));
            a->Visit(MJ_NVP(velocity_elevation_gain));
        } 
    } config_;

    struct State {
        bool disarmed{true};
        float commanded_rps{0};
        float azimuth_input{0};
        float elevation_input{0};
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
            cmd_.mode = moteus::BldcServoMode::kPosition;
            cmd_.velocity = state_.commanded_rps;
        }
        if (controller_ == nullptr || *controller_ == nullptr){
            return;
        }
        (*controller_)->bldc_servo()->Command(cmd_);
    }

    // Subscriber to RawCommand
    Canard::ObjCallback<Rotor, uavcan_equipment_esc_RawCommand> raw_command_cb {this, &Rotor::handle_esc_RawCommand};
    Canard::Subscriber<uavcan_equipment_esc_RawCommand> raw_command_sub{raw_command_cb, 0};
};