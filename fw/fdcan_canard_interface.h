#pragma once

#include <canard.h>
#include <canard/interface.h>
#include <canard/transfer_object.h>

#include "mjlib/micro/pool_ptr.h"
#include "fw/fdcan.h"

class FdcanCanardInterface : public Canard::Interface {
public:
  FdcanCanardInterface(uint8_t index, mjlib::micro::Pool& pool, size_t canard_memory_size, moteus::FDCan& fdcan);

  bool broadcast(const Canard::Transfer& bcast_transfer) override;
  bool request(uint8_t destination_node_id, const Canard::Transfer& req_transfer) override;
  bool respond(uint8_t destination_node_id, const Canard::Transfer& res_transfer) override;
  uint8_t get_node_id() const override { return canardGetLocalNodeID(canard_); }

  void set_node_id(uint8_t node_id);
  void set_brs_enabled(bool enabled) { brs_enabled_ = enabled; }
  bool is_brs_enabled() const { return brs_enabled_; }
  void spin_once(uint64_t timestamp_usec);

private:
  moteus::FDCan& fdcan_;
  static constexpr uint64_t cleanup_period_usec = 1'000'000; // Per libcanard recommendation
  uint64_t last_cleanup_usec_{0};
  bool brs_enabled_{false};
  // We allocate large objects in the pool, as moteus has better tools for monitoring this than stack usage.
  CanardInstance* const canard_;
  CanardTxTransfer* const tx_transfer_;
  CanardCANFrame* const rx_frame_;
  FDCAN_RxHeaderTypeDef* const rx_header_;
  

  void populateTxTransfer(const Canard::Transfer& transfer);
  static void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer);
  static bool shouldAcceptTransfer(const CanardInstance* ins, uint64_t* out_data_type_signature, uint16_t data_type_id, CanardTransferType transfer_type, uint8_t source_node_id);
};
