#pragma once

// Canard base API
#include <canard.h>
// Canard C++ API
#include <canard/publisher.h>
#include <canard/subscriber.h>
#include <canard/service_client.h>
#include <canard/service_server.h>
#include <canard/handler_list.h>
#include <canard/transfer_object.h>


#include "mjlib/micro/pool_ptr.h"
#include "fdcan.h"

class CanardInterface : public Canard::Interface
{

public:
    CanardInterface(mjlib::micro::Pool* pool, const size_t reserved_memory_size, moteus::FDCan* can);

    bool broadcast(const Canard::Transfer& bcast_transfer) override;
    bool request(uint8_t destination_node_id, const Canard::Transfer& req_transfer) override;
    bool respond(uint8_t destination_node_id, const Canard::Transfer& res_transfer) override;

    void process(uint32_t duration_ms);

    uint8_t get_node_id() const override { return canard.node_id; }
    void set_node_id(uint8_t node_id);

    static void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer);
    static bool shouldAcceptTransfer(const CanardInstance* ins,
                                     uint64_t* out_data_type_signature,
                                     uint16_t data_type_id,
                                     CanardTransferType transfer_type,
                                     uint8_t source_node_id);

    int64_t tx_count_ = 0;
    int64_t rx_count_ = 0;
    moteus::FDCan* can;
private:
    uint8_t* memory_pool;
    CanardInstance canard;
    CanardTxTransfer tx_transfer;
    mjlib::micro::Pool* pool_;

    void populateTxTransfer(const Canard::Transfer& transfer);

    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[64];
};
