#include <iostream>
#include <cstring>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

// DroneCAN / UAVCAN includes
#include <socketcan.h>
#include <canard/publisher.h>
#include <canard/subscriber.h>
#include <canard/service_client.h>
#include <canard/service_server.h>
#include <canard/handler_list.h>
#include <canard/transfer_object.h>

// Include the DSDL definitions you need:
#include <dronecan_msgs.h> // for uavcan_tunnel_Broadcast, GetNodeInfo, NodeStatus, etc.

#ifndef MY_NODE_ID
#define MY_NODE_ID 126
#endif

/*
  declare heads of handler and transfer lists
 */
 DEFINE_HANDLER_LIST_HEADS();
 DEFINE_TRANSFER_OBJECT_HEADS();

//----------------------------------------------------------
// Some utility functions
//----------------------------------------------------------
static uint64_t micros64(void)
{
    static uint64_t first_us = 0;
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    uint64_t tus = (uint64_t)(ts.tv_sec * 1000000ULL + ts.tv_nsec / 1000ULL);
    if (first_us == 0) {
        first_us = tus;
    }
    return tus - first_us;
}

static uint32_t millis32(void)
{
    return (uint32_t)(micros64() / 1000ULL);
}

static void getUniqueID(uint8_t id[16])
{
    memset(id, 0, 16);
    FILE* f = fopen("/etc/machine-id", "r");
    if (f) {
        fread(id, 1, 16, f);
        fclose(f);
    }
}

//----------------------------------------------------------
// Simple Moteus interface using SocketCAN
//----------------------------------------------------------
class MoteusInterface {
public:
    explicit MoteusInterface(const std::string& interface_name)
        : interface_name_(interface_name), socket_fd_(-1)
    {}

    ~MoteusInterface() {
        if (socket_fd_ != -1) {
            close(socket_fd_);
        }
    }

    bool init() {
        struct ifreq ifr{};
        struct sockaddr_can addr{};

        socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_fd_ < 0) {
            std::cerr << "Error opening Moteus socket\n";
            return false;
        }

        // Enable CAN FD
        int enable_canfd = 1;
        if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
                       &enable_canfd, sizeof(enable_canfd)) < 0) {
            std::cerr << "Error enabling CAN FD mode for Moteus\n";
            return false;
        }

        // Optionally set non-blocking so we can poll in a single loop
        int flags = fcntl(socket_fd_, F_GETFL, 0);
        if (flags >= 0) {
            fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK);
        }

        std::strncpy(ifr.ifr_name, interface_name_.c_str(), IFNAMSIZ - 1);
        if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
            std::cerr << "Error getting interface index for Moteus\n";
            return false;
        }

        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(socket_fd_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            std::cerr << "Error binding Moteus socket\n";
            return false;
        }
        return true;
    }

    // Read one frame, non-blocking. Returns true if a frame was read.
    bool readFrameOnce(canfd_frame& frameOut) {
        ssize_t num_bytes = ::read(socket_fd_, &frameOut, sizeof(frameOut));
        if (num_bytes == (ssize_t)sizeof(canfd_frame)) {
            return true;
        }
        return false;
    }

    // Send a CAN FD frame
    bool sendFrame(const canfd_frame& frame) {
        ssize_t num_bytes = ::write(socket_fd_, &frame, sizeof(frame));
        if (num_bytes < 0) {
            std::cerr << "Error sending CAN frame to Moteus\n";
            return false;
        }
        return true;
    }

private:
    std::string interface_name_;
    int socket_fd_;
};


//----------------------------------------------------------
// A specialized CanardInterface that uses SocketCAN
//----------------------------------------------------------
class CanardInterface : public Canard::Interface {
    friend class TunnelNode;

public:
    CanardInterface(uint8_t iface_index) : Interface(iface_index) {}

    void init(const char* interface_name);

    bool broadcast(const Canard::Transfer &bcast_transfer) override;
    bool request(uint8_t destination_node_id, const Canard::Transfer &req_transfer) override;
    bool respond(uint8_t destination_node_id, const Canard::Transfer &res_transfer) override;

    void process(uint32_t duration_ms);

    static void onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer);
    static bool shouldAcceptTransfer(const CanardInstance* ins,
                                     uint64_t* out_data_type_signature,
                                     uint16_t data_type_id,
                                     CanardTransferType transfer_type,
                                     uint8_t source_node_id);

    uint8_t get_node_id() const override { return canard.node_id; }
    void set_node_id(uint8_t node_id) {
        canardSetLocalNodeID(&canard, node_id);
    }

private:
    uint8_t memory_pool[2048];
    CanardInstance canard;
    CanardTxTransfer tx_transfer;

    SocketCANInstance socketcan;
};

//----------------------------------------------------------
// The Node class that publishes and subscribes to DroneCAN
//----------------------------------------------------------
class TunnelNode
{
public:
    // Our constructor sets up the interface index for canard_iface
    TunnelNode() : canard_iface(0),
                   // Subscribers/publishers must reference the same Canard::Interface
                   node_status(canard_iface),
                   tunnel_broadcast(canard_iface),
                   // The callback object for the tunnel broadcast
                   tunnel_broadcast_cb(this, &TunnelNode::handle_tunnel_broadcast),
                   // The subscriber that will be registered in init()
                   tunnel_broadcast_sub(tunnel_broadcast_cb, 0),
                   node_info_req_cb(this, &TunnelNode::handle_GetNodeInfo),
                   node_info_server(canard_iface, node_info_req_cb)
    {
        // Initialize the node_status_msg
        memset(&node_status_msg, 0, sizeof(node_status_msg));
    }

    // Attach Moteus so we can forward frames to it
    void attachMoteus(MoteusInterface* moteus) {
        moteus_ptr = moteus;
    }

    void init(const char* dronecan_iface) {
        // Initialize the underlying interface
        canard_iface.init(dronecan_iface);

        // Optionally set local node ID
        canard_iface.set_node_id(MY_NODE_ID);

        // The subscriber is constructed above, and by default
        // registers itself with the interface. If your library
        // requires an explicit "start" or "subscribe" call, do so:
        //   tunnel_broadcast_sub.start(canard_iface);

        // Now we can receive broadcast messages of type uavcan.tunnel.Broadcast
    }

    // Called from main loop to poll incoming frames
    void process1HzTasks(uint64_t timestamp_usec) {
        // Send the NodeStatus at 1Hz
        //send_NodeStatus();
    }

    // Forward Moteus -> DroneCAN
    // We'll embed [source, dest, data...] in the `uavcan_tunnel_Broadcast`
    void tunnel_data(uint8_t source, uint8_t destination,
                     const uint8_t* data, uint8_t len)
    {
        uavcan_tunnel_Broadcast msg{};
        msg.protocol.protocol = UAVCAN_TUNNEL_PROTOCOL_UNDEFINED;
        msg.channel_id = 0;

        msg.buffer.len = len + 2;
        if (msg.buffer.len > sizeof(msg.buffer.data)) {
            msg.buffer.len = sizeof(msg.buffer.data);
        }
        msg.buffer.data[0] = source;
        msg.buffer.data[1] = destination;
        memcpy(&msg.buffer.data[2], data, msg.buffer.len - 2);

        tunnel_broadcast.broadcast(msg);
    }

    void processCanard(uint32_t timeout_msec) {
        canard_iface.process(timeout_msec);
    }

private:
    // 1) Callback for the tunnel broadcast
    void handle_tunnel_broadcast(const CanardRxTransfer& transfer,
                                 const uavcan_tunnel_Broadcast& msg)
    {
        if (!moteus_ptr) {
            // No Moteus interface
            return;
        }
        if (msg.buffer.len < 2) {
            // Not enough data to contain [source + destination + payload...]
            return;
        }

        uint8_t source      = msg.buffer.data[0];
        uint8_t destination = msg.buffer.data[1];
        const uint8_t* raw_data = &msg.buffer.data[2];
        size_t raw_len = msg.buffer.len - 2;

        // Build a CAN FD frame
        canfd_frame f{};
        // Example ID encoding:
        // 7 bits for source, 7 bits for destination
        uint16_t can_id = ((uint16_t)(source & 0x7F) << 8) | (destination & 0x7F);
        // optional: can_id |= 0x8000 if "reply requested"

        f.can_id = can_id;
        f.len = (raw_len > 64) ? 64 : (uint8_t)raw_len;
        memcpy(f.data, raw_data, f.len);

        // Send frame to Moteus
        moteus_ptr->sendFrame(f);
    }

    // 2) Node Info server request callback
    void handle_GetNodeInfo(const CanardRxTransfer& transfer,
                            const uavcan_protocol_GetNodeInfoRequest& req)
    {
        uavcan_protocol_GetNodeInfoResponse node_info_rsp{};
        // fill in some info
        node_info_rsp.name.len = snprintf((char*)node_info_rsp.name.data,
                                          sizeof(node_info_rsp.name.data),
                                          "Moteus Tunnel Node");

        node_info_rsp.software_version.major = 1;
        node_info_rsp.software_version.minor = 0;
        node_info_rsp.hardware_version.major = 1;
        node_info_rsp.hardware_version.minor = 0;
        getUniqueID(node_info_rsp.hardware_version.unique_id);

        node_info_rsp.status = node_status_msg;
        node_info_rsp.status.uptime_sec = millis32() / 1000UL;

        node_info_server.respond(transfer, node_info_rsp);
    }

    // Send a 1Hz NodeStatus
    void send_NodeStatus() {
        node_status_msg.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
        node_status_msg.mode   = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;
        node_status_msg.sub_mode = 0;
        node_status_msg.uptime_sec = millis32() / 1000UL;

        node_status.broadcast(node_status_msg);
    }

private:
    // The underlying interface
    CanardInterface canard_iface;

    // Moteus pointer for bridging
    MoteusInterface* moteus_ptr = nullptr;

    // 1Hz NodeStatus publisher
    Canard::Publisher<uavcan_protocol_NodeStatus> node_status;
    uavcan_protocol_NodeStatus node_status_msg;

    // The Tunnel broadcast publisher
    Canard::Publisher<uavcan_tunnel_Broadcast> tunnel_broadcast;

    // Subscriber for the tunnel broadcast
    Canard::ObjCallback<TunnelNode, uavcan_tunnel_Broadcast> tunnel_broadcast_cb;
    Canard::Subscriber<uavcan_tunnel_Broadcast> tunnel_broadcast_sub;

    // Node Info server
    Canard::ObjCallback<TunnelNode, uavcan_protocol_GetNodeInfoRequest> node_info_req_cb;
    Canard::Server<uavcan_protocol_GetNodeInfoRequest> node_info_server;
};


//----------------------------------------------------------
// Implementation of the CanardInterface
//----------------------------------------------------------
void CanardInterface::init(const char* interface_name)
{
    int16_t res = socketcanInit(&socketcan, interface_name);
    if (res < 0) {
        std::cerr << "Failed to open CAN iface '" << interface_name << "'\n";
        exit(1);
    }

    // init canard
    canardInit(&canard,
               memory_pool,
               sizeof(memory_pool),
               onTransferReceived,
               shouldAcceptTransfer,
               this);

    // Optionally set local node ID if desired, or do it externally
    // canardSetLocalNodeID(&canard, MY_NODE_ID);
}

bool CanardInterface::broadcast(const Canard::Transfer &bcast_transfer)
{
    tx_transfer = {
        .transfer_type       = bcast_transfer.transfer_type,
        .data_type_signature = bcast_transfer.data_type_signature,
        .data_type_id        = bcast_transfer.data_type_id,
        .inout_transfer_id   = bcast_transfer.inout_transfer_id,
        .priority            = bcast_transfer.priority,
        .payload             = (const uint8_t*)bcast_transfer.payload,
        .payload_len         = uint16_t(bcast_transfer.payload_len),
#if CANARD_ENABLE_DEADLINE
        .deadline_usec       = micros64() + (bcast_transfer.timeout_ms * 1000),
#endif
    };
    return canardBroadcastObj(&canard, &tx_transfer) > 0;
}

bool CanardInterface::request(uint8_t destination_node_id, const Canard::Transfer &req_transfer)
{
    tx_transfer = {
        .transfer_type       = req_transfer.transfer_type,
        .data_type_signature = req_transfer.data_type_signature,
        .data_type_id        = req_transfer.data_type_id,
        .inout_transfer_id   = req_transfer.inout_transfer_id,
        .priority            = req_transfer.priority,
        .payload             = (const uint8_t*)req_transfer.payload,
        .payload_len         = uint16_t(req_transfer.payload_len),
#if CANARD_ENABLE_DEADLINE
        .deadline_usec       = micros64() + (req_transfer.timeout_ms * 1000),
#endif
    };
    return canardRequestOrRespondObj(&canard, destination_node_id, &tx_transfer) > 0;
}

bool CanardInterface::respond(uint8_t destination_node_id, const Canard::Transfer &res_transfer)
{
    tx_transfer = {
        .transfer_type       = res_transfer.transfer_type,
        .data_type_signature = res_transfer.data_type_signature,
        .data_type_id        = res_transfer.data_type_id,
        .inout_transfer_id   = res_transfer.inout_transfer_id,
        .priority            = res_transfer.priority,
        .payload             = (const uint8_t*)res_transfer.payload,
        .payload_len         = uint16_t(res_transfer.payload_len),
#if CANARD_ENABLE_DEADLINE
        .deadline_usec       = micros64() + (res_transfer.timeout_ms * 1000),
#endif
    };
    return canardRequestOrRespondObj(&canard, destination_node_id, &tx_transfer) > 0;
}

void CanardInterface::process(uint32_t timeout_msec)
{
    // Transmit
    for (const CanardCANFrame* txf = nullptr; (txf = canardPeekTxQueue(&canard)) != nullptr; ) {
        int16_t tx_res = socketcanTransmit(&socketcan, txf, 0);
        if (tx_res != 0) {
            canardPopTxQueue(&canard);
        }
    }
    
    // Receive
    uint32_t start_ms = millis32();
    while (millis32() - start_ms < timeout_msec) {
        CanardCANFrame rx_frame;
        int16_t rx_res = socketcanReceive(&socketcan, &rx_frame, timeout_msec);
        if (rx_res > 0) {
            auto res = canardHandleRxFrame(&canard, &rx_frame, micros64());
            switch (res) {
                case -CANARD_ERROR_RX_MISSED_START:
                case -CANARD_ERROR_RX_WRONG_TOGGLE:
                case -CANARD_ERROR_RX_SHORT_FRAME:
                case -CANARD_ERROR_RX_BAD_CRC:
                    std::cout << "CAN RX error: " << res << "\n";
                    break;
                default:
                    break;
            }
        }
    }
}

void CanardInterface::onTransferReceived(CanardInstance* ins, CanardRxTransfer* transfer)
{
    // The user_reference is our 'this' pointer
    CanardInterface* iface = static_cast<CanardInterface*>(ins->user_reference);
    // Dispatch to the standard handler logic (C++ classes)
    iface->handle_message(*transfer);
}

bool CanardInterface::shouldAcceptTransfer(const CanardInstance* ins,
                                           uint64_t* out_data_type_signature,
                                           uint16_t data_type_id,
                                           CanardTransferType transfer_type,
                                           uint8_t source_node_id)
{
    // The user_reference is our 'this' pointer
    CanardInterface* iface = (CanardInterface*)ins->user_reference;
    return iface->accept_message(data_type_id, *out_data_type_signature);
}


//----------------------------------------------------------
// A simple main that starts Moteus + TunnelNode
//----------------------------------------------------------
int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <moteus_iface> <dronecan_iface>\n";
        return 1;
    }

    // 1) Initialize Moteus
    MoteusInterface moteus(argv[1]);
    if (!moteus.init()) {
        return 1;
    }

    // 2) Create the DroneCAN node
    TunnelNode node;
    node.attachMoteus(&moteus); // now the node can forward DroneCAN → Moteus

    // Initialize the DroneCAN interface
    node.init(argv[2]);

    // 3) Main loop: poll both sides
    uint64_t next_1hz = micros64();
    while (true) {
        // a) Poll DroneCAN for e.g. 1ms
        node.processCanard(1);

        // b) Read Moteus frames (non-blocking) and forward them to DroneCAN
        canfd_frame frame{};
        while (moteus.readFrameOnce(frame)) {
            // Extract the source/dest from can_id in your real scheme
            // For demonstration, assume top 7 bits is source, lower 7 bits is destination
            uint8_t source      = (frame.can_id >> 8) & 0xFF;
            uint8_t destination = frame.can_id & 0x7F;
            node.tunnel_data(source, destination, frame.data, (uint8_t)frame.len);
        }

        // c) 1Hz tasks (NodeStatus, etc.)
        uint64_t now = micros64();
        if (now >= next_1hz) {
            next_1hz += 1000000ULL;
            node.process1HzTasks(now);
        }

        // optional short sleep to prevent max CPU usage
        // usleep(1000);
    }

    return 0;
}
