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

#include "fdcan_canard_interface.h"
#include <canard/publisher.h>
#include <canard/subscriber.h>
#include <canard/service_client.h>
#include <canard/service_server.h>
#include "dronecan_param.h"
#include "dronecan_param_store.h"
#include "moteus_controller.h"
#include "fw/dronecan_tunnel.h"

// Moteus specific
#include "mjlib/micro/persistent_config.h"

class DronecanNode {
public:
    DronecanNode(mjlib::micro::Pool* pool,
                 FdcanCanardInterface* canard_iface,
                 mjlib::micro::PersistentConfig* persistent_config,
                 DronecanParamStore* param_store);

    void start();
    void poll(uint32_t time_ms);

    struct Config {
        uint8_t node_id{42};

        template <typename Store>
        void RegisterParameters(Store& store) {
            DRONECAN_PARAMETER(DC_NODE_ID, node_id, 42, 1, 127);
        }

        template <typename Archive>
        void Serialize(Archive* a) {
            a->Visit(MJ_NVP(node_id));
        }
    };

    Config* config() { return &config_; }

    void sendLogMessage(const char* source, const char* text, uint8_t level);
    void attachTunnel(MoteusDronecanTunnel* tunnel);

private:
    void sendNodeStatus();

    static void getUniqueID(uint8_t id[16]);

    uint32_t last_nodestatus_ms_{0};
    uint32_t latest_time_ms_{0};
    uavcan_protocol_NodeStatus node_status_msg_{};

    mjlib::micro::Pool* const pool_;
    FdcanCanardInterface* const canard_iface_;
    mjlib::micro::PersistentConfig* const persistent_config_;
    DronecanParamStore* const param_store_;
    Config config_{};

    MoteusDronecanTunnel* dronecan_tunnel_{nullptr};

    Canard::Publisher<uavcan_protocol_NodeStatus> node_status_pub_{*canard_iface_};
    Canard::Publisher<uavcan_protocol_debug_LogMessage> log_pub_{*canard_iface_};

    void handle_GetNodeInfo(const CanardRxTransfer& transfer, const uavcan_protocol_GetNodeInfoRequest& req);
    Canard::ObjCallback<DronecanNode, uavcan_protocol_GetNodeInfoRequest> node_info_req_cb_{this, &DronecanNode::handle_GetNodeInfo};
    Canard::Server<uavcan_protocol_GetNodeInfoRequest> node_info_server_{*canard_iface_, node_info_req_cb_};

    void handle_param_GetSet(const CanardRxTransfer& transfer, const uavcan_protocol_param_GetSetRequest& req);
    Canard::ObjCallback<DronecanNode, uavcan_protocol_param_GetSetRequest> param_get_set_req_cb_{this, &DronecanNode::handle_param_GetSet};
    Canard::Server<uavcan_protocol_param_GetSetRequest> param_server_{*canard_iface_, param_get_set_req_cb_};

    void handle_param_ExecuteOpcode(const CanardRxTransfer& transfer, const uavcan_protocol_param_ExecuteOpcodeRequest& req);
    Canard::ObjCallback<DronecanNode, uavcan_protocol_param_ExecuteOpcodeRequest> param_executeopcode_req_cb_{this, &DronecanNode::handle_param_ExecuteOpcode};
    Canard::Server<uavcan_protocol_param_ExecuteOpcodeRequest> param_opcode_server_{*canard_iface_, param_executeopcode_req_cb_};

    void handle_GetTransportStats(const CanardRxTransfer& transfer, const uavcan_protocol_GetTransportStatsRequest& req);
    Canard::ObjCallback<DronecanNode, uavcan_protocol_GetTransportStatsRequest> transport_stats_cb_{this, &DronecanNode::handle_GetTransportStats};
    Canard::Server<uavcan_protocol_GetTransportStatsRequest> transport_stats_server_{*canard_iface_, transport_stats_cb_};

    void handle_tunnel_Broadcast(const CanardRxTransfer& transfer, const uavcan_tunnel_Broadcast& req);
    Canard::ObjCallback<DronecanNode, uavcan_tunnel_Broadcast> tunnel_broadcast_cb_{this, &DronecanNode::handle_tunnel_Broadcast};
    Canard::Subscriber<uavcan_tunnel_Broadcast> tunnel_sub_{tunnel_broadcast_cb_, 0};
    Canard::Publisher<uavcan_tunnel_Broadcast> tunnel_pub_{*canard_iface_};
};
