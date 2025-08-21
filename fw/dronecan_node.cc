#include "fw/dronecan_node.h"

 DronecanNode::DronecanNode(mjlib::micro::Pool* pool, FdcanCanardInterface* canard_iface, mjlib::micro::PersistentConfig* persistent_config, DronecanParamStore* param_store)
 : pool_(pool), canard_iface_(canard_iface), persistent_config_(persistent_config), param_store_(param_store)
 {}

void DronecanNode::start()
{
    // TODO: Dynamic node allocation
    canard_iface_->set_node_id(config_.node_id);
}

void DronecanNode::getUniqueID(uint8_t id[16])
{
    // STM32G4 series unique ID base (96 bits)
    const uint32_t* unique_id_base = reinterpret_cast<const uint32_t*>(0x1FFF7590);

    // Copy the 12-byte (96 bit) unique ID
    memcpy(id, unique_id_base, 12);

    // Pad the remaining 4 bytes with a known value, 0xAB
    memset(id + 12, 0xAB, 4);
}

void DronecanNode::sendNodeStatus()
{
    node_status_msg.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status_msg.mode   = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    node_status_msg.sub_mode = 0;
    node_status_msg.vendor_specific_status_code = 0;
    node_status_msg.uptime_sec = latest_time_ms_ / 1000;

    this->node_status_pub.broadcast(node_status_msg);
}

void DronecanNode::poll(uint32_t time_ms){
    latest_time_ms_ = time_ms;
    if (time_ms - last_nodestatus_ms_ > 1000) {
        sendNodeStatus();
        last_nodestatus_ms_ = time_ms;
    }
    const uint64_t time_us = static_cast<uint64_t>(time_ms) * 1000;
    canard_iface_->spin_once(time_us);
}

void DronecanNode::handle_GetNodeInfo(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoRequest& req)
{

    node_status_msg.vendor_specific_status_code = 1;
    (void)req; // Suppress unused parameter warning

    uavcan_protocol_GetNodeInfoResponse node_info_rsp{};
    node_info_rsp.name.len = snprintf(
        reinterpret_cast<char*>(node_info_rsp.name.data),
        sizeof(node_info_rsp.name.data),
        "Moteus ESC"
    );
    node_info_rsp.software_version.major = 0;
    node_info_rsp.software_version.minor = 1;
    node_info_rsp.hardware_version.major = 0;
    node_info_rsp.hardware_version.minor = 1;

    getUniqueID(node_info_rsp.hardware_version.unique_id);

    node_info_rsp.status = this->node_status_msg;
    node_info_rsp.status.uptime_sec = latest_time_ms_ / 1000;

    this->node_info_server.respond(transfer, node_info_rsp);
}

void DronecanNode::sendLogMessage(const char* source, const char* text, uint8_t level)
{
    uavcan_protocol_debug_LogMessage msg{};
    msg.level.value = level;

    msg.source.len = snprintf(
        reinterpret_cast<char*>(msg.source.data),
        sizeof(msg.source.data),
        "%s",
        source
    );
    msg.text.len = snprintf(
        reinterpret_cast<char*>(msg.text.data),
        sizeof(msg.text.data),
        "%s",
        text
    );

    this->log_pub.broadcast(msg);
}

void DronecanNode::handle_param_GetSet(const CanardRxTransfer& transfer, const uavcan_protocol_param_GetSetRequest& req)
{
    auto res = param_store_->GetSet(req);
    this->param_server.respond(transfer, res);

    return;
}

void DronecanNode::handle_GetTransportStats(const CanardRxTransfer& transfer, const uavcan_protocol_GetTransportStatsRequest& req){
    uavcan_protocol_GetTransportStatsResponse res{};

    // TODO

    this->transport_stats_server.respond(transfer, res);
}

void DronecanNode::handle_param_ExecuteOpcode(const CanardRxTransfer& transfer, const uavcan_protocol_param_ExecuteOpcodeRequest& req)
{
    uavcan_protocol_param_ExecuteOpcodeResponse res{};
    res.ok = false;

    if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE) {
        param_store_->ResetAll();
        res.ok = true;
    } else if (req.opcode == UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE) {
        if (persistent_config_ != nullptr) {
            persistent_config_->Write();
            res.ok = true;
        }
    }

    this->param_opcode_server.respond(transfer, res);
}

void DronecanNode::handle_tunnel_Broadcast(const CanardRxTransfer& transfer, const uavcan_tunnel_Broadcast& req)
{
    if (dronecan_tunnel_) {
        dronecan_tunnel_->handle_tunnel_Broadcast(transfer, req);
    }
}

void DronecanNode::attachTunnel(MoteusDronecanTunnel* tunnel) {
    dronecan_tunnel_ = tunnel;
    if (tunnel) {
        tunnel->set_uavcan_pub_callback([this](uavcan_tunnel_Broadcast& msg) {
            return this->tunnel_pub_.broadcast(msg);
        });
    }
}