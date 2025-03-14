#include "moteus_tunnel.h"

MoteusDronecanTunnel::MoteusDronecanTunnel(CanardInterface* canard_iface) : canard_iface_(canard_iface){};

void MoteusDronecanTunnel::handle_tunnel_Broadcast(const CanardRxTransfer& transfer, const uavcan_tunnel_Broadcast& req)
{
    if (!current_read_header_) { 
        return; 
    }

    if (req.buffer.len < 2) {
        // Not enough data for source and destination
        return;
    }

    // Ignore messages that are not meant for us
    if (req.protocol.protocol != UAVCAN_TUNNEL_PROTOCOL_UNDEFINED || req.channel_id != config_.channel) {
        return;
    }

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

constexpr mjlib::multiplex::MicroDatagramServer::Properties MoteusDronecanTunnel::properties() const {
    mjlib::multiplex::MicroDatagramServer::Properties properties{
        // We reserve 2 bytes for source and destination
        .max_size = sizeof(decltype(uavcan_tunnel_Broadcast::buffer.data)) - 2
    };
    
    return properties;
}

void MoteusDronecanTunnel::AsyncRead(Header* header,
    const mjlib::base::string_span& data,
    const mjlib::micro::SizeCallback& callback){
    MJ_ASSERT(!current_read_callback_);
    current_read_callback_ = callback;
    current_read_data_ = data;
    current_read_header_ = header;
}
void MoteusDronecanTunnel::AsyncWrite(const Header& header,
     const std::string_view& data,
     const Header& query_header,
     const mjlib::micro::SizeCallback& callback){
    // We don't preserve BRS or FD flags, as they don't matter
    // when we tunnel through DroneCAN.
    uavcan_tunnel_Broadcast broadcast_msg{};
    broadcast_msg.protocol.protocol = UAVCAN_TUNNEL_PROTOCOL_UNDEFINED;
    broadcast_msg.channel_id = config_.channel;

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
