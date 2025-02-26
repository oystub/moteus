#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <array>

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
#include <uavcan.protocol.GetTransportStats.h>
#include <uavcan.tunnel.Broadcast.h>

// Dronecan functions
#include "dronecan_param.h"
#include "dronecan_param.h"
#include "dronecan_param_store.h"

// Motrus specific
#include "fdcan.h"
#include "mjlib/micro/persistent_config.h"
#include "mjlib/multiplex/micro_datagram_server.h"
#include "mjlib/micro/pool_ptr.h"

class DronecanNode;  // Forward declaration

class CanardInterface : public Canard::Interface
{
    friend class DronecanNode;

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

class DronecanNode : public mjlib::multiplex::MicroDatagramServer
{
private:
    class Config;
public:
    DronecanNode(mjlib::micro::Pool* pool, moteus::FDCan* can, DronecanParamStore* param_store, uint8_t node_id = 0);

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

    void RegisterPersistentConfig(mjlib::micro::PersistentConfig* config) { persistent_config_ = config; }

    Config* config() { return &config_; }

private:
    struct Config {
        uint8_t node_id = 0;
        bool dna = false;

        template <typename Store>
        void RegisterParameters(Store& store) {
            DRONECAN_PARAMETER(DC_NODE_ID, node_id, 0, 0, 127);
            DRONECAN_PARAMETER(DC_DNA, dna, 1);
        }

        template <typename Archive>
        void Serialize(Archive* a) {
            a->Visit(MJ_NVP(node_id));
            a->Visit(MJ_NVP(dna));
        }
    };

    void sendLogMessage(const char* source, const char* text, uint8_t level);

    // Handlers
    void handle_GetNodeInfo(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoRequest& req);
    void handle_param_GetSet(const CanardRxTransfer& transfer, const uavcan_protocol_param_GetSetRequest& req);
    void handle_param_ExecuteOpcode(const CanardRxTransfer& transfer, const uavcan_protocol_param_ExecuteOpcodeRequest& req);
    void handle_tunnel_Broadcast(const CanardRxTransfer& transfer, const uavcan_tunnel_Broadcast& req);
    void handle_GetTransportStats(const CanardRxTransfer& transfer, const uavcan_protocol_GetTransportStatsRequest& req);

    static void getUniqueID(uint8_t id[16]);
    uint32_t millis32() const;

    uint32_t millisecond_counter{0};

    uavcan_protocol_NodeStatus node_status_msg;
    CanardInterface canard_iface;
    mjlib::micro::PersistentConfig* persistent_config_{nullptr};
    mjlib::micro::Pool* pool_;
    DronecanParamStore* param_store_;
    Config config_;

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
    Canard::ObjCallback<DronecanNode, uavcan_protocol_GetTransportStatsRequest> transport_stats_cb{
        this, &DronecanNode::handle_GetTransportStats};
    Canard::Server<uavcan_protocol_GetTransportStatsRequest> transport_stats_server{
        canard_iface, transport_stats_cb};

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