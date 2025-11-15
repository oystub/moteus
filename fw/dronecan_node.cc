#include "fw/dronecan_node.h"

namespace {
constexpr uint32_t kUniqueIdBaseAddr = 0x1FFF7590; // STM32G4 UID base
constexpr uint8_t kIdPadValue = 0xAB;
constexpr char kNodeName[] = "Swashplateless rotor";
} // namespace

DronecanNode::DronecanNode(mjlib::micro::Pool *pool,
                           FdcanCanardInterface *canard_iface,
                           mjlib::micro::PersistentConfig *persistent_config,
                           DronecanParamStore *param_store)
    : pool_(pool), canard_iface_(canard_iface),
      persistent_config_(persistent_config), param_store_(param_store) {}

void DronecanNode::start() {
  // TODO: Dynamic node allocation
  canard_iface_->set_node_id(config_.node_id);
}

void DronecanNode::getUniqueID(uint8_t id[16]) {
  // 96 bits from the STM32G4 UID (unique per board)
  const uint32_t *unique_id_base =
      reinterpret_cast<const uint32_t *>(kUniqueIdBaseAddr);
  memcpy(id, unique_id_base, 12);
  // Pad remaining 32 bits with an arbitrary fixed value
  memset(id + 12, kIdPadValue, 4);
}

void DronecanNode::sendNodeStatus() {
  node_status_msg_.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
  node_status_msg_.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
  node_status_msg_.sub_mode = 0;
  node_status_msg_.vendor_specific_status_code = 0;
  node_status_msg_.uptime_sec = latest_time_ms_ / 1000;

  node_status_pub_.broadcast(node_status_msg_);
}

void DronecanNode::poll(uint32_t time_ms) {
  latest_time_ms_ = time_ms;
  if (time_ms - last_nodestatus_ms_ > 1000) {
    sendNodeStatus();
    last_nodestatus_ms_ = time_ms;
  }
  const uint64_t time_us = static_cast<uint64_t>(time_ms) * 1000;
  canard_iface_->spin_once(time_us);
}

void DronecanNode::handle_GetNodeInfo(
    const CanardRxTransfer &transfer,
    const uavcan_protocol_GetNodeInfoRequest &req) {
  (void)req;

  uavcan_protocol_GetNodeInfoResponse res{};
  res.name.len = snprintf(reinterpret_cast<char *>(res.name.data),
                          sizeof(res.name.data), "%s", kNodeName);
  res.software_version.major = 0;
  res.software_version.minor = 1;
  res.hardware_version.major = 0;
  res.hardware_version.minor = 1;

  getUniqueID(res.hardware_version.unique_id);

  res.status = node_status_msg_;
  res.status.uptime_sec = latest_time_ms_ / 1000;

  node_info_server_.respond(transfer, res);
}

void DronecanNode::sendLogMessage(const char *source, const char *text,
                                  uint8_t level) {
  uavcan_protocol_debug_LogMessage msg{};
  msg.level.value = level;

  msg.source.len = snprintf(reinterpret_cast<char *>(msg.source.data),
                            sizeof(msg.source.data), "%s", source);
  msg.text.len = snprintf(reinterpret_cast<char *>(msg.text.data),
                          sizeof(msg.text.data), "%s", text);

  log_pub_.broadcast(msg);
}

void DronecanNode::handle_param_GetSet(
    const CanardRxTransfer &transfer,
    const uavcan_protocol_param_GetSetRequest &req) {
  auto res = param_store_->GetSet(req);
  param_server_.respond(transfer, res);
}

void DronecanNode::handle_GetTransportStats(
    const CanardRxTransfer &transfer,
    const uavcan_protocol_GetTransportStatsRequest &req) {
  (void)req;
  uavcan_protocol_GetTransportStatsResponse res{};
  // TODO: Fill with actual transport statistics
  transport_stats_server_.respond(transfer, res);
}

void DronecanNode::handle_param_ExecuteOpcode(
    const CanardRxTransfer &transfer,
    const uavcan_protocol_param_ExecuteOpcodeRequest &req) {
  uavcan_protocol_param_ExecuteOpcodeResponse res{};
  res.ok = false;

  if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE) {
    param_store_->ResetAll();
    res.ok = true;
  } else if (req.opcode ==
             UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE) {
    if (persistent_config_) {
      persistent_config_->Write();
      res.ok = true;
    }
  }

  param_opcode_server_.respond(transfer, res);
}

void DronecanNode::handle_tunnel_Broadcast(const CanardRxTransfer &transfer,
                                           const uavcan_tunnel_Broadcast &req) {
  if (dronecan_tunnel_) {
    dronecan_tunnel_->handle_tunnel_Broadcast(transfer, req);
  }
}

void DronecanNode::attachTunnel(MoteusDronecanTunnel *tunnel) {
  dronecan_tunnel_ = tunnel;
  if (tunnel) {
    tunnel->set_uavcan_pub_callback([this](uavcan_tunnel_Broadcast &msg) {
      return tunnel_pub_.broadcast(msg);
    });
  }
}

void DronecanNode::attachRotor(DroneCanRotor *rotor) {
  dronecan_rotor_ = rotor;
}

void DronecanNode::handle_esc_RawCommand(
    const CanardRxTransfer &transfer,
    const uavcan_equipment_esc_RawCommand &msg) {
  if (dronecan_rotor_) {
    dronecan_rotor_->handle_esc_RawCommand(msg);
  }
}

void DronecanNode::handle_actuator_ArrayCommand(
    const CanardRxTransfer &transfer,
    const uavcan_equipment_actuator_ArrayCommand &msg) {
  if (dronecan_rotor_) {
    dronecan_rotor_->handle_actuator_ArrayCommand(msg);
  }
}
