#include "dronecan_rotor.h"
#include "euclidean_commands.h"
#include <numbers>

DroneCanRotor::DroneCanRotor(moteus::MoteusController *controller)
    : controller_(controller) {
  cmd_.mode = moteus::BldcServoMode::kStopped;
}

void DroneCanRotor::poll(uint32_t time_ms) {
  // TODO: Trigger timeouts if no throttle
}

void DroneCanRotor::sendMoteusCommand(moteus::MoteusController *controller) {
  if (!controller) {
    return;
  }

  if (polar_cmd_.thrust <= 0.0f) {
    // No thrust command, disconnect motor
    cmd_.mode = moteus::BldcServoMode::kStopped;
    controller->bldc_servo()->Command(cmd_);
    return;
  }

  cmd_.mode = moteus::BldcServoMode::kSinusoidalVelocity;
  cmd_.velocity = sinvel_cmd_.speed_rps;
  cmd_.sinusoidal_velocity_scale = sinvel_cmd_.modulation;
  cmd_.sinusoidal_velocity_phase_rad = sinvel_cmd_.phase;
  controller->bldc_servo()->Command(cmd_);
}

void DroneCanRotor::handle_esc_RawCommand(
    const uavcan_equipment_esc_RawCommand &msg) {
  switch (config_.command_mode) {
  case CommandMode::CARTESIAN_ESC:
    return handleCartesianEscRaw(msg);
  case CommandMode::CARTESIAN_ACTUATOR:
    return handleCartesianActuatorRaw(msg);
  case CommandMode::POLAR_ACTUATOR:
    return handlePolarActuatorRaw(msg);
  }
}

void DroneCanRotor::handleCartesianEscRaw(
    const uavcan_equipment_esc_RawCommand &msg) {
  float xyz[3]{};

  for (int i = 0; i < AxisIdx::AXIS_COUNT; ++i) {
    const int idx = config_.cartesian_cmd_idx[i];
    if (!validIdx(idx, msg.cmd.len))
      continue;

    float val = normalizeEsc(msg.cmd.data[idx]);
    if (isNeg(static_cast<AxisIdx>(i)))
      val = -val;
    xyz[axis(static_cast<AxisIdx>(i))] += val;
  }

  // rotor only produces upward thrust.
  cartesian_cmd_.x = std::clamp(xyz[0], -1.0f, 1.0f);
  cartesian_cmd_.y = std::clamp(xyz[1], -1.0f, 1.0f);
  cartesian_cmd_.z = std::clamp(xyz[2], -1.0f, 0.0f);
}

void DroneCanRotor::handleCartesianActuatorRaw(
    const uavcan_equipment_esc_RawCommand &msg) {
  // Only get the Z axis from RawCommand. The other axes are handled via
  // ArrayCommand.
  float z = 0.f;

  const int z_pos = config_.cartesian_cmd_idx[Z_POS];
  const int z_neg = config_.cartesian_cmd_idx[Z_NEG];

  if (validIdx(z_pos, msg.cmd.len))
    z += normalizeEsc(msg.cmd.data[z_pos]);
  if (validIdx(z_neg, msg.cmd.len))
    z -= normalizeEsc(msg.cmd.data[z_neg]);

  cartesian_cmd_.z = std::clamp(z, -1.0f, 0.0f);
  processCartesianCommand();
}

void DroneCanRotor::handlePolarActuatorRaw(
    const uavcan_equipment_esc_RawCommand &msg) {
  // Only get the thrust from RawCommand. Azimuth and Elevation come from
  // ArrayCommand. In polar mode, positive thrust direction is up (negative Z)
  // instead of down.
  float t = 0.f;

  const int thrust_idx = config_.polar_idx[PolarIdx::THRUST];
  if (validIdx(thrust_idx, msg.cmd.len)) {
    t = normalizeEsc(msg.cmd.data[thrust_idx]); // negate for upward thrust
  }

  polar_cmd_.thrust = std::clamp(t, 0.0f, 1.0f);
  processPolarCommand();
}

void DroneCanRotor::handle_actuator_ArrayCommand(
    const uavcan_equipment_actuator_ArrayCommand &msg) {
  switch (config_.command_mode) {
  case CommandMode::CARTESIAN_ESC:
    return;
  case CommandMode::CARTESIAN_ACTUATOR:
    return handleCartesianActuatorArray(msg);
  case CommandMode::POLAR_ACTUATOR:
    return handlePolarActuatorArray(msg);
  }
}

void DroneCanRotor::handleCartesianActuatorArray(
    const uavcan_equipment_actuator_ArrayCommand &msg) {
  // Get the X and Y axes from ArrayCommand. Z axis is handled via RawCommand.
  // Note: We require that both x and y (and negative counterparts) be sent in
  // the same message.

  float x{}, y{};

  const int x_pos = config_.cartesian_cmd_idx[X_POS];
  const int y_pos = config_.cartesian_cmd_idx[Y_POS];
  const int x_neg = config_.cartesian_cmd_idx[X_NEG];
  const int y_neg = config_.cartesian_cmd_idx[Y_NEG];

  for (const auto &command : msg.commands.data) {
    if (command.command_type !=
        UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_UNITLESS) {
      continue; // Unsupported command type
    }
    const int id = command.actuator_id;

    if (id == x_pos) {
      x += command.command_value;
    } else if (id == y_pos) {
      y += command.command_value;
    } else if (id == x_neg) {
      x -= command.command_value;
    } else if (id == y_neg) {
      y -= command.command_value;
    }
  }

  cartesian_cmd_.x = std::clamp(x, -1.0f, 1.0f);
  cartesian_cmd_.y = std::clamp(y, -1.0f, 1.0f);
  processCartesianCommand();
}

void DroneCanRotor::handlePolarActuatorArray(
    const uavcan_equipment_actuator_ArrayCommand &msg) {
  // Get Azimuth and Elevation from ArrayCommand. Thrust is handled via
  // RawCommand.
  for (const auto &command : msg.commands.data) {
    if (command.command_type !=
        UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_UNITLESS) {
      continue; // Unsupported command type
    }

    const int id = command.actuator_id;

    if (id == config_.polar_idx[PolarIdx::AZIMUTH]) {
      polar_cmd_.azimuth_rad =
          command.command_value *
          std::numbers::pi_v<float>; // Scale [-1,1] to [-pi, pi]
    } else if (id == config_.polar_idx[PolarIdx::ELEVATION]) {
      polar_cmd_.elevation_rad =
          command.command_value *
          (config_.max_elevation_deg * std::numbers::pi_v<float> /
           180.0f); // Scale [-1,1] to [-max_elevation, max_elevation]
    }
  }
  processPolarCommand();
}

void DroneCanRotor::processCartesianCommand() {
  // Convert cartesian_cmd_ to polar_cmd_
  // Using the euclidean_commands.h functions
  // Leave this unimplemented for now.

  auto euclidean_params = compute_prism_dims(
      config_.min_actuation_thrust, deg2rad(config_.max_elevation_deg));

  auto [T, beta, psi] = scale_command(euclidean_params, cartesian_cmd_.x,
                                      cartesian_cmd_.y, cartesian_cmd_.z);
  polar_cmd_.thrust = T;
  polar_cmd_.elevation_rad = beta;
  polar_cmd_.azimuth_rad = psi;

  processPolarCommand();
}

void DroneCanRotor::processPolarCommand() {
  // Convert thrust and elevation to speed and modulation
  // Leave this unimplemented for now.

  sinvel_cmd_.speed_rps = thrust_to_speed(polar_cmd_.thrust);
  sinvel_cmd_.modulation = elevation_to_modulation(polar_cmd_.elevation_rad);
  sinvel_cmd_.phase = polar_cmd_.azimuth_rad;

  // Send to moteus
  if (controller_) {
    sendMoteusCommand(controller_);
  }
}

float DroneCanRotor::thrust_to_speed(float thrust) const {
  // Simple quadratic mapping for now
  thrust = std::clamp(thrust, 0.0f, 1.0f);
  return config_.max_speed_rpm / 60.0f * sqrtf(thrust);
}

float DroneCanRotor::elevation_to_modulation(float elevation_rad) const {
  // Simple linear mapping for now
  const float max_elevation_rad = deg2rad(config_.max_elevation_deg);
  elevation_rad = std::clamp(elevation_rad, 0.0f, max_elevation_rad);
  return (elevation_rad / max_elevation_rad) * config_.max_modulation;
}
