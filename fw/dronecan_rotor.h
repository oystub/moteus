#pragma once

#include "dronecan_param.h"
#include "moteus_controller.h"

#include <uavcan.equipment.actuator.ArrayCommand.h>
#include <uavcan.equipment.esc.RPMCommand.h>
#include <uavcan.equipment.esc.RawCommand.h>
#include <array>
#include <numbers>

class DroneCanRotor {
public:
  explicit DroneCanRotor(moteus::MoteusController *controller);

  void handle_esc_RawCommand(const uavcan_equipment_esc_RawCommand &msg);
  void handle_actuator_ArrayCommand(
      const uavcan_equipment_actuator_ArrayCommand &msg);

  void poll(uint32_t time_ms); // for periodic tasks like timeouts etc.

  struct Config;
  Config *config() { return &config_; }

private:
  enum CommandMode : uint8_t {
    CARTESIAN_ESC = 0,
    CARTESIAN_ACTUATOR = 1,
    POLAR_ACTUATOR = 2,
  };

  enum AxisIdx : uint8_t {
    X_POS = 0,
    Y_POS,
    Z_POS,
    X_NEG,
    Y_NEG,
    Z_NEG,
    AXIS_COUNT
  };

  struct CartesianCommand {
    float x{}, y{}; // [-1, 1]
    float z{};      // [-1, 0]
  } cartesian_cmd_;

  enum PolarIdx : uint8_t { THRUST = 0, AZIMUTH, ELEVATION, POLAR_COUNT };

  struct PolarCommand {
    float thrust{},      // [0, 1]
        azimuth_rad{},   // [-pi, pi]
        elevation_rad{}; // [0, max_elevation]
  } polar_cmd_;

  struct SinVelCommand {
    float speed_rps = 0.0F;
    float modulation = 0.0F;
    float phase = 0.0F; // [-pi, pi]
  } sinvel_cmd_;

  struct Config {
    int rot_dir = 1;
    int thrust_dir = -1; // 1=positive z, -1=negative z
    uint8_t command_mode = CommandMode::CARTESIAN_ACTUATOR;
    std::array<int, AxisIdx::AXIS_COUNT> cartesian_cmd_idx = { {0, 1, -1, -1, -1, 2} };
    std::array<int, PolarIdx::POLAR_COUNT> polar_idx = { {0, 1, 2} };
    int polar_idx[PolarIdx::POLAR_COUNT] = {0, 1, 2};

    float max_speed_rpm = 1000.0f;
    float max_elevation_deg = 15.0f;
    float max_modulation = 0.2f; // modulation amount for max_elevation_deg
    float min_actuation_thrust = 0.7f; // Minimum actuation thrust where full x
                                       // and y actuation should be possible.

    template <typename Store> void RegisterParameters(Store &store) {
      DRONECAN_PARAMETER(ROT_CMD_MODE, command_mode, 1, 0, 2);
      DRONECAN_PARAMETER(ROT_DIR, rot_dir, 1, -1, 1);
      DRONECAN_PARAMETER(THR_DIR, thrust_dir, -1, -1, 1);

      DRONECAN_PARAMETER(DC_X_IDX, cartesian_cmd_idx[X_POS], 0, -1, 255);
      DRONECAN_PARAMETER(DC_Y_IDX, cartesian_cmd_idx[Y_POS], 1, -1, 255);
      DRONECAN_PARAMETER(DC_Z_IDX, cartesian_cmd_idx[Z_POS], -1, -1, 20);
      DRONECAN_PARAMETER(DC_X_NEG_IDX, cartesian_cmd_idx[X_NEG], -1, -1, 255);
      DRONECAN_PARAMETER(DC_Y_NEG_IDX, cartesian_cmd_idx[Y_NEG], -1, -1, 255);
      DRONECAN_PARAMETER(DC_Z_NEG_IDX, cartesian_cmd_idx[Z_NEG], 2, -1, 20);

      DRONECAN_PARAMETER(ROT_THR_IDX, polar_idx[PolarIdx::THRUST], 0, -1, 20);
      DRONECAN_PARAMETER(ROT_AZM_IDX, polar_idx[PolarIdx::AZIMUTH], 1, -1, 255);
      DRONECAN_PARAMETER(ROT_ELV_IDX, polar_idx[PolarIdx::ELEVATION], 2, -1,
                         255);

      DRONECAN_PARAMETER(ROT_RPM_MAX, max_speed_rpm, 1000, 0, 10000);
      DRONECAN_PARAMETER(ROT_ELV_MAX, max_elevation_deg, 15, 0, 45);
      DRONECAN_PARAMETER(ROT_MOD_MAX, max_modulation, 0.15f, 0.0f, 1.0f);
      DRONECAN_PARAMETER(ROT_THR_MINACT, min_actuation_thrust, 0.7f, 0.0f,
                         1.0f);
    }

    template <typename Archive> void Serialize(Archive *a) {
      a->Visit(MJ_NVP(max_speed_rpm));
      a->Visit(MJ_NVP(max_elevation_deg));
      a->Visit(MJ_NVP(max_modulation));
      a->Visit(MJ_NVP(min_actuation_thrust));
    }
  } config_;

  moteus::BldcServoCommandData cmd_{};
  moteus::MoteusController *controller_;

  static constexpr bool isNeg(AxisIdx i) { return i >= X_NEG; }
  static constexpr int axis(AxisIdx i) { return static_cast<int>(i) % 3; }
  static inline float normalizeEsc(int16_t raw) {
    // DroneCAN RawCommand: [-8192, 8191]
    return (raw >= 0) ? (static_cast<float>(raw) / 8191.0F) : (static_cast<float>(raw) / 8192.0F);
  }
  static inline bool validIdx(int idx, int len) {
    return idx >= 0 && idx < len;
  }
  static inline float deg2rad(float d) {
    return d * std::numbers::pi_v<float> /  180.0f;
  }

  void processCartesianCommand();
  void processPolarCommand();
  void sendMoteusCommand(moteus::MoteusController *controller);

  void handleCartesianEscRaw(const uavcan_equipment_esc_RawCommand &msg);
  void handleCartesianActuatorRaw(const uavcan_equipment_esc_RawCommand &msg);
  void handlePolarActuatorRaw(const uavcan_equipment_esc_RawCommand &msg);

  void handleCartesianActuatorArray(
      const uavcan_equipment_actuator_ArrayCommand &msg);
  void
  handlePolarActuatorArray(const uavcan_equipment_actuator_ArrayCommand &msg);

  float thrust_to_speed(float thrust) const;
  float elevation_to_modulation(float elevation) const;
};
