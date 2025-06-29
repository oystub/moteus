#include "fw/fdcan_canard_interface.h"

// Initialization for libcanard
DEFINE_HANDLER_LIST_HEADS();
DEFINE_TRANSFER_OBJECT_HEADS();

FdcanCanardInterface::FdcanCanardInterface(uint8_t index, mjlib::micro::Pool& pool, size_t canard_memory_size, moteus::FDCan& fdcan) : 
Canard::Interface(index, false), 
fdcan_(fdcan),
canard_(static_cast<CanardInstance*>(pool.Allocate(sizeof(CanardInstance), alignof(CanardInstance)))),
tx_transfer_(static_cast<CanardTxTransfer*>(pool.Allocate(sizeof(CanardTxTransfer), alignof(CanardTxTransfer)))),
rx_frame_(static_cast<CanardCANFrame*>(pool.Allocate(sizeof(CanardCANFrame), alignof(CanardCANFrame)))),
rx_header_(static_cast<FDCAN_RxHeaderTypeDef*>(pool.Allocate(sizeof(FDCAN_RxHeaderTypeDef), alignof(FDCAN_RxHeaderTypeDef)))){
  
  uint8_t* const memory_pool = static_cast<uint8_t*>(pool.Allocate(canard_memory_size, alignof(uint8_t))); 
  canardInit(canard_, memory_pool, canard_memory_size, onTransferReceived, shouldAcceptTransfer, this);
  canardInitTxTransfer(tx_transfer_);
}

void FdcanCanardInterface::set_node_id(uint8_t node_id) {
  if (canardGetLocalNodeID(canard_) != CANARD_BROADCAST_NODE_ID) {
    canardForgetLocalNodeID(canard_);
  }
  if (node_id == CANARD_BROADCAST_NODE_ID) {
    return;
  }
  canardSetLocalNodeID(canard_, node_id);
}

bool FdcanCanardInterface::broadcast(const Canard::Transfer& bcast_transfer)
{
  populateTxTransfer(bcast_transfer);
  return canardBroadcastObj(canard_, tx_transfer_) > 0;
}

bool FdcanCanardInterface::request(uint8_t destination_node_id, const Canard::Transfer& req_transfer)
{
  populateTxTransfer(req_transfer);
  return canardRequestOrRespondObj(canard_, destination_node_id, tx_transfer_) > 0;
}

bool FdcanCanardInterface::respond(uint8_t destination_node_id, const Canard::Transfer& res_transfer)
{
  populateTxTransfer(res_transfer);
  return canardRequestOrRespondObj(canard_, destination_node_id, tx_transfer_) > 0;
}

void FdcanCanardInterface::spin_once(uint64_t timestamp_usec) {
  // Check CAN bus status, and recover from bus off if necessary.
  const auto status = fdcan_.status();
  if (status.BusOff) {
    fdcan_.RecoverBusOff();
  }

  if (timestamp_usec - last_cleanup_usec_ >= cleanup_period_usec) {
    canardCleanupStaleTransfers(canard_, timestamp_usec);
    last_cleanup_usec_ = timestamp_usec;
  }

  // Send any pending transfers
  for (const CanardCANFrame* txf = nullptr; (txf = canardPeekTxQueue(canard_)) != nullptr;) {
    if (!fdcan_.ReadyForSend()) {
      // FIFO is full, wait until next cycle
      break;
    }
    std::string_view str(reinterpret_cast<const char*>(txf->data), txf->data_len);

    moteus::FDCan::SendOptions send_options;
    send_options.bitrate_switch = txf->canfd && brs_enabled_ ? moteus::FDCan::Override::kRequire : moteus::FDCan::Override::kDisable;
    send_options.fdcan_frame    = txf->canfd ? moteus::FDCan::Override::kRequire : moteus::FDCan::Override::kDisable;
    send_options.remote_frame   = moteus::FDCan::Override::kDisable;
    send_options.extended_id    = moteus::FDCan::Override::kRequire;

    fdcan_.Send(txf->id, str, send_options);
    canardPopTxQueue(canard_);
  }

  // Process incoming frames
  while(fdcan_.PollReal(*rx_header_, rx_frame_->data)) {
    // Translate to CanardCANFrame ID format
    rx_frame_->id = rx_header_->Identifier & 0x1FFFFFFF;
    if (rx_header_->RxFrameType == FDCAN_REMOTE_FRAME) {
      rx_frame_->id |= CANARD_CAN_FRAME_RTR;
    }
    if (rx_header_->ErrorStateIndicator == FDCAN_ESI_ACTIVE) {
      rx_frame_->id |= CANARD_CAN_FRAME_ERR;
    }
    if (rx_header_->IdType == FDCAN_EXTENDED_ID) {
      rx_frame_->id |= CANARD_CAN_FRAME_EFF;
    }
    if (rx_header_->FDFormat == FDCAN_FD_CAN) {
      rx_frame_->canfd = true;
    } else {
      rx_frame_->canfd = false;
    }

    rx_frame_->iface_id = this->get_index();

    const size_t data_len = moteus::FDCan::ParseDlc(rx_header_->DataLength);
    rx_frame_->data_len = static_cast<uint8_t>(data_len);

    canardHandleRxFrame(canard_, rx_frame_, timestamp_usec);
  }
}

void FdcanCanardInterface::populateTxTransfer(const Canard::Transfer& transfer)
{
  *tx_transfer_ = {
    .transfer_type        = transfer.transfer_type,
    .data_type_signature  = transfer.data_type_signature,
    .data_type_id         = transfer.data_type_id,
    .inout_transfer_id    = transfer.inout_transfer_id,
    .priority             = transfer.priority,
    .payload              = static_cast<const uint8_t*>(transfer.payload),
    .payload_len          = static_cast<uint16_t>(transfer.payload_len),
    .canfd                = transfer.canfd,
    .tao                  = transfer.canfd == false
  };
}

void FdcanCanardInterface::onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer) {
  FdcanCanardInterface* iface = static_cast<FdcanCanardInterface*>(ins->user_reference);
  iface->handle_message(*transfer);
}

bool FdcanCanardInterface::shouldAcceptTransfer(const CanardInstance* ins, uint64_t* out_data_type_signature, uint16_t data_type_id, CanardTransferType transfer_type, uint8_t source_node_id) {
  // Suppress unused parameter warnings
  (void)source_node_id;
  (void)transfer_type;

  FdcanCanardInterface* iface = static_cast<FdcanCanardInterface*>(ins->user_reference);
  bool ok = iface->accept_message(data_type_id, transfer_type, *out_data_type_signature);
  return ok;
}
