#include "canard_iface.h"

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
