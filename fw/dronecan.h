#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <array>


// DroneCAN message and service types
#include <uavcan.protocol.param.ExecuteOpcode.h>
#include <uavcan.protocol.param.GetSet.h>
#include <uavcan.protocol.NodeStatus.h>
#include <uavcan.protocol.GetNodeInfo.h>
#include <uavcan.protocol.debug.LogMessage.h>
#include <uavcan.protocol.GetTransportStats.h>
#include <uavcan.tunnel.Broadcast.h>

#include "canard_iface.h"
#include "dronecan_param.h"
#include "dronecan_param_store.h"
#include "moteus_controller.h"

// Moteus specific
#include "mjlib/micro/persistent_config.h"

class DronecanNode
{
private:
    class Config;
public:
DronecanNode(mjlib::micro::Pool* pool, CanardInterface* canard_iface, mjlib::micro::PersistentConfig* persistent_config, DronecanParamStore* param_store);
    void sendDummyNodeStatus();
    void poll();
    void pollMillisecond();

    Config* config() { return &config_; }

    void sendLogMessage(const char* source, const char* text, uint8_t level);
    void broadcastTunnel(uint8_t protocol, uint8_t channel, const uint8_t* data, size_t size);

private:
    struct Config {
        uint8_t node_id{42};
        bool dna{false};

        template <typename Store>
        void RegisterParameters(Store& store) {
            DRONECAN_PARAMETER(DC_NODE_ID, node_id, 42, 1, 127);
            DRONECAN_PARAMETER(DC_DNA, dna, 1);
        }

        template <typename Archive>
        void Serialize(Archive* a) {
            a->Visit(MJ_NVP(node_id));
            a->Visit(MJ_NVP(dna));
        }
    };

    

    // Handlers
    void handle_GetNodeInfo(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoRequest& req);
    void handle_param_GetSet(const CanardRxTransfer& transfer, const uavcan_protocol_param_GetSetRequest& req);
    void handle_param_ExecuteOpcode(const CanardRxTransfer& transfer, const uavcan_protocol_param_ExecuteOpcodeRequest& req);
    void handle_GetTransportStats(const CanardRxTransfer& transfer, const uavcan_protocol_GetTransportStatsRequest& req);

    static void getUniqueID(uint8_t id[16]);
    uint32_t millis32() const;

    uint32_t millisecond_counter{0};

    uavcan_protocol_NodeStatus node_status_msg;
    mjlib::micro::Pool* pool_;
    CanardInterface* canard_iface_;
    mjlib::micro::PersistentConfig* persistent_config_;
    DronecanParamStore* param_store_;
    Config config_;

    // Status and logging
    Canard::Publisher<uavcan_protocol_NodeStatus> node_status_pub{*canard_iface_};
    Canard::Publisher<uavcan_protocol_debug_LogMessage> log_pub{*canard_iface_};
    Canard::Publisher<uavcan_tunnel_Broadcast> tunnel_pub{*canard_iface_};
    Canard::ObjCallback<DronecanNode, uavcan_protocol_GetNodeInfoRequest> node_info_req_cb{
        this, &DronecanNode::handle_GetNodeInfo};
    Canard::Server<uavcan_protocol_GetNodeInfoRequest> node_info_server{
        *canard_iface_, node_info_req_cb};

    // Parameter handling
    Canard::ObjCallback<DronecanNode, uavcan_protocol_param_GetSetRequest> param_get_set_req_cb{
        this, &DronecanNode::handle_param_GetSet};
    Canard::Server<uavcan_protocol_param_GetSetRequest> param_server{
        *canard_iface_, param_get_set_req_cb};
    Canard::ObjCallback<DronecanNode, uavcan_protocol_param_ExecuteOpcodeRequest> param_executeopcode_req_cb{
        this, &DronecanNode::handle_param_ExecuteOpcode};
    Canard::Server<uavcan_protocol_param_ExecuteOpcodeRequest> param_opcode_server{
        *canard_iface_, param_executeopcode_req_cb};
    Canard::ObjCallback<DronecanNode, uavcan_protocol_GetTransportStatsRequest> transport_stats_cb{
        this, &DronecanNode::handle_GetTransportStats};
    Canard::Server<uavcan_protocol_GetTransportStatsRequest> transport_stats_server{
        *canard_iface_, transport_stats_cb};
};