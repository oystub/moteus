#pragma once

#include <stdio.h>
#include <stdlib.h>

// Canard base API
#include <canard.h>
// Canard C++ API
#include <canard/publisher.h>
#include <canard/subscriber.h>
#include <canard/service_client.h>
#include <canard/service_server.h>
#include <canard/handler_list.h>
#include <canard/transfer_object.h>

// DroneCAN message and service types
#include <uavcan.protocol.param.ExecuteOpcode.h>
#include <uavcan.protocol.param.GetSet.h>
#include <uavcan.protocol.NodeStatus.h>
#include <uavcan.protocol.GetNodeInfo.h>
#include <uavcan.protocol.debug.LogMessage.h>
#include <uavcan.tunnel.Broadcast.h>

// Motrus specific
#include "fdcan.h"
#include "mjlib/multiplex/micro_datagram_server.h"

class DronecanNode;  // Forward declaration

class CanardInterface : public Canard::Interface
{
    friend class DronecanNode;

public:
    CanardInterface(uint8_t* memory_pool, size_t memory_pool_size, moteus::FDCan* can);

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

private:
    uint8_t* memory_pool;
    CanardInstance canard;
    CanardTxTransfer tx_transfer;
    moteus::FDCan* can;

private:
    void populateTxTransfer(const Canard::Transfer& transfer);

    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[64];
};

class DronecanNode : public mjlib::multiplex::MicroDatagramServer
{
public:
    DronecanNode(uint8_t* memory_pool, size_t memory_pool_size, moteus::FDCan* can, uint8_t node_id = 0);

    void sendDummyNodeStatus();
    void poll();
    void pollMillisecond();

    constexpr mjlib::multiplex::MicroDatagramServer::Properties properties() const override;
    void AsyncRead(Header* header,
                           const mjlib::base::string_span& data,
                           const mjlib::micro::SizeCallback& callback) override;
    void AsyncWrite(const Header& header,
                            const std::string_view& data,
                            const Header& query_header,
                            const mjlib::micro::SizeCallback& callback) override;

private:
    void sendLogMessage(const char* source, const char* text, uint8_t level);

    // Handlers
    void handle_GetNodeInfo(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoRequest& req);
    void handle_param_GetSet(const CanardRxTransfer& transfer, const uavcan_protocol_param_GetSetRequest& req);
    void handle_param_ExecuteOpcode(const CanardRxTransfer& transfer, const uavcan_protocol_param_ExecuteOpcodeRequest& req);
    void handle_tunnel_Broadcast(const CanardRxTransfer& transfer, const uavcan_tunnel_Broadcast& req);

    static void getUniqueID(uint8_t id[16]);
    uint32_t millis32() const;

    uint32_t millisecond_counter{0};

    uavcan_protocol_NodeStatus node_status_msg;
    CanardInterface canard_iface;

    // Status and logging
    Canard::Publisher<uavcan_protocol_NodeStatus> node_status_pub{canard_iface};
    Canard::Publisher<uavcan_protocol_debug_LogMessage> log_pub{canard_iface};
    Canard::ObjCallback<DronecanNode, uavcan_protocol_GetNodeInfoRequest> node_info_req_cb{
        this, &DronecanNode::handle_GetNodeInfo};
    Canard::Server<uavcan_protocol_GetNodeInfoRequest> node_info_server{
        canard_iface, node_info_req_cb};

    // Parameter handling
    Canard::ObjCallback<DronecanNode, uavcan_protocol_param_GetSetRequest> param_get_set_req_cb{
        this, &DronecanNode::handle_param_GetSet};
    Canard::Server<uavcan_protocol_param_GetSetRequest> param_server{
        canard_iface, param_get_set_req_cb};
    Canard::ObjCallback<DronecanNode, uavcan_protocol_param_ExecuteOpcodeRequest> param_executeopcode_req_cb{
        this, &DronecanNode::handle_param_ExecuteOpcode};
    Canard::Server<uavcan_protocol_param_ExecuteOpcodeRequest> param_opcode_server{
        canard_iface, param_executeopcode_req_cb};

    // Moteus tunneling over DroneCAN
    Canard::Publisher<uavcan_tunnel_Broadcast> tunnel_pub{canard_iface};
    Canard::ObjCallback<DronecanNode, uavcan_tunnel_Broadcast> tunnel_broadcast_cb{
        this, &DronecanNode::handle_tunnel_Broadcast};
    Canard::Subscriber<uavcan_tunnel_Broadcast> tunnel_sub{
        tunnel_broadcast_cb, 0};
    mjlib::micro::SizeCallback current_read_callback_;
    Header* current_read_header_ = nullptr;
    mjlib::base::string_span current_read_data_;
};

