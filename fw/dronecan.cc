#include "fw/dronecan.h"

// Initialization for libcanard
DEFINE_HANDLER_LIST_HEADS();
DEFINE_TRANSFER_OBJECT_HEADS();

/*
 * CanardInterface
 */
CanardInterface::CanardInterface(mjlib::micro::Pool* pool, const size_t reserved_memory_size, moteus::FDCan* can)
    : Canard::Interface(0), can(can), pool_(pool)
{
    this->memory_pool = static_cast<uint8_t*>(pool->Allocate(reserved_memory_size, alignof(uint8_t)));

    this->can = can;
    canardInit(&this->canard,
               memory_pool,
               reserved_memory_size,
               onTransferReceived,
               shouldAcceptTransfer,
               this);
}

bool CanardInterface::broadcast(const Canard::Transfer& bcast_transfer)
{
    populateTxTransfer(bcast_transfer);
    return canardBroadcastObj(&this->canard, &this->tx_transfer) > 0;
}

bool CanardInterface::request(uint8_t destination_node_id, const Canard::Transfer& req_transfer)
{
    populateTxTransfer(req_transfer);
    return canardRequestOrRespondObj(&this->canard, destination_node_id, &this->tx_transfer) > 0;
}

bool CanardInterface::respond(uint8_t destination_node_id, const Canard::Transfer& res_transfer)
{
    populateTxTransfer(res_transfer);
    return canardRequestOrRespondObj(&this->canard, destination_node_id, &this->tx_transfer) > 0;
}

void CanardInterface::process(uint32_t duration_ms)
{
    // Check CAN bus status, and recover from bus off if necessary.
    const auto status = this->can->status();
    if (status.BusOff) {
        this->can->RecoverBusOff();
    }
    
    // Send any pending transfers
    for (const CanardCANFrame* txf = nullptr; (txf = canardPeekTxQueue(&this->canard)) != nullptr;) {
        if (!this->can->ReadyForSend()) {
            // FIFO is full, wait until next cycle
            break;
        }
        std::string_view str(reinterpret_cast<const char*>(txf->data), txf->data_len);

        moteus::FDCan::SendOptions send_options;
        send_options.bitrate_switch = moteus::FDCan::Override::kDisable;
        send_options.fdcan_frame    = moteus::FDCan::Override::kDisable;
        send_options.remote_frame   = moteus::FDCan::Override::kDisable;
        send_options.extended_id    = moteus::FDCan::Override::kRequire;

        this->can->Send(txf->id, str, send_options);
        tx_count_++;
        canardPopTxQueue(&this->canard);
    }

    // Process incoming frames
    while(this->can->PollReal(&rx_header, rx_data)){
        rx_count_++;
        CanardCANFrame frame{};
        frame.id = rx_header.Identifier;
        frame.id &= 0x1FFFFFFF;

        if (rx_header.RxFrameType == FDCAN_REMOTE_FRAME) {
            frame.id |= CANARD_CAN_FRAME_RTR;
        }
        if (rx_header.ErrorStateIndicator == FDCAN_ESI_ACTIVE) {
            frame.id |= CANARD_CAN_FRAME_ERR;
        }
        if (rx_header.IdType == FDCAN_EXTENDED_ID) {
            frame.id |= CANARD_CAN_FRAME_EFF;
        }

        // Force EFF usage, remove ERR/RTR
        frame.id |= CANARD_CAN_FRAME_EFF;
        frame.id &= ~(CANARD_CAN_FRAME_ERR | CANARD_CAN_FRAME_RTR);

        frame.iface_id = 0;

        size_t data_len = std::min(moteus::FDCan::ParseDlc(rx_header.DataLength), int{8});
        frame.data_len = static_cast<uint8_t>(data_len);
        memcpy(frame.data, rx_data, data_len);

        canardHandleRxFrame(&this->canard, &frame, duration_ms);
    }
}

void CanardInterface::onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer)
{
    CanardInterface* iface = static_cast<CanardInterface*>(ins->user_reference);
    iface->handle_message(*transfer);
}

bool CanardInterface::shouldAcceptTransfer(const CanardInstance* ins,
                                           uint64_t* out_data_type_signature,
                                           uint16_t data_type_id,
                                           CanardTransferType transfer_type,
                                           uint8_t source_node_id)
{
    // Suppress unused parameter warnings
    (void)source_node_id;
    (void)transfer_type;

    CanardInterface* iface = static_cast<CanardInterface*>(ins->user_reference);
    bool ok = iface->accept_message(data_type_id, *out_data_type_signature);
    return ok;
}

void CanardInterface::populateTxTransfer(const Canard::Transfer& transfer)
{
    this->tx_transfer = {
        .transfer_type        = transfer.transfer_type,
        .data_type_signature  = transfer.data_type_signature,
        .data_type_id         = transfer.data_type_id,
        .inout_transfer_id    = transfer.inout_transfer_id,
        .priority             = transfer.priority,
        .payload              = static_cast<const uint8_t*>(transfer.payload),
        .payload_len          = static_cast<uint16_t>(transfer.payload_len),
    };
}

void CanardInterface::set_node_id(uint8_t node_id)
{
    canardSetLocalNodeID(&this->canard, node_id);
}

/*
 * DronecanNode
 */
DronecanNode::DronecanNode(mjlib::micro::Pool* pool, moteus::FDCan* can, DronecanParamStore* param_store, uint8_t node_id)
    : canard_iface(pool, 8192, can), pool_(pool), param_store_(param_store), rotor_(&moteus_controller_)
{
    canard_iface.set_node_id(node_id);
    param_store_->Register(&config_);
    param_store_->Register(rotor_.config());
}

uint32_t DronecanNode::millis32() const
{
    return this->millisecond_counter;
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

void DronecanNode::sendDummyNodeStatus()
{
    //TODO canardCleanupStaleTransfers ?

    node_status_msg.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    node_status_msg.mode   = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
    node_status_msg.sub_mode = 0;
    node_status_msg.vendor_specific_status_code = 0;
    node_status_msg.uptime_sec = this->millis32() / 1000UL;

    this->node_status_pub.broadcast(node_status_msg);
}

void DronecanNode::poll()
{
    canard_iface.process(1);
}

void DronecanNode::pollMillisecond()
{
    this->millisecond_counter++;
}

void DronecanNode::handle_GetNodeInfo(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoRequest& req)
{
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
    node_info_rsp.status.uptime_sec = millis32() / 1000UL;

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

    auto error_counters = canard_iface.can->ErrorCounters();
    res.transfer_errors = error_counters.TxErrorCnt + error_counters.RxErrorCnt;
    res.transfers_tx = canard_iface.tx_count_;
    res.transfers_rx = canard_iface.rx_count_;

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
    if (!current_read_header_) { 
        return; 
    }

    if (req.buffer.len < 2) {
        // Not enough data for source and destination
        return;
    }

    // TODO: Check protocol and channel ID

    // Source and destination are the first two bytes
    current_read_header_->source = req.buffer.data[0];
    current_read_header_->destination = req.buffer.data[1];
    // Remaining bytes are the data
    current_read_header_->size = req.buffer.len - 2;
    memcpy(current_read_data_.data(), req.buffer.data + 2, current_read_header_->size);

    auto copy = current_read_callback_;
    auto bytes = current_read_header_->size;

    current_read_callback_ = {};
    current_read_header_ = {};
    current_read_data_ = {};

    copy(mjlib::micro::error_code(), bytes);
    return;
}

constexpr mjlib::multiplex::MicroDatagramServer::Properties DronecanNode::properties() const {
    mjlib::multiplex::MicroDatagramServer::Properties properties{
        // We reserve 2 bytes for source and destination
        .max_size = sizeof(decltype(uavcan_tunnel_Broadcast::buffer.data)) - 2
    };
    
    return properties;
}

void DronecanNode::AsyncRead(Header* header,
                           const mjlib::base::string_span& data,
                           const mjlib::micro::SizeCallback& callback){
    MJ_ASSERT(!current_read_callback_);
    current_read_callback_ = callback;
    current_read_data_ = data;
    current_read_header_ = header;
}
void DronecanNode::AsyncWrite(const Header& header,
                            const std::string_view& data,
                            const Header& query_header,
                            const mjlib::micro::SizeCallback& callback){
    // We don't preserve BRS or FD flags, as they don't matter
    // when we tunnel through DroneCAN.
    uavcan_tunnel_Broadcast broadcast_msg{};
    broadcast_msg.protocol.protocol = UAVCAN_TUNNEL_PROTOCOL_UNDEFINED;
    broadcast_msg.channel_id = 0; // TODO: Set channel ID

    if (data.size() > static_cast<size_t>(properties().max_size)) {
      callback(mjlib::micro::error_code(), 0);
      return;
    }
    // First two bytes are source and destination
    broadcast_msg.buffer.data[0] = 1; // TODO: Set correct instance number
    broadcast_msg.buffer.data[1] = 0;
    // Remaining bytes are the data
    std::memcpy(broadcast_msg.buffer.data + 2, data.data(), data.size());
    broadcast_msg.buffer.len = data.size() + 2;

    const bool success = tunnel_pub.broadcast(broadcast_msg);

    callback(mjlib::micro::error_code(), success ? data.size() : 0);
}
