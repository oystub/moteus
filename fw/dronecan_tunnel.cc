#include "fw/dronecan_tunnel.h"

MoteusDronecanTunnel::MoteusDronecanTunnel(){};

MoteusDronecanTunnel::MoteusDronecanTunnel(const std::function<bool(const uavcan_tunnel_Broadcast&)>& callback)
    : uavcan_pub_callback_(callback) {}

constexpr mjlib::multiplex::MicroDatagramServer::Properties MoteusDronecanTunnel::properties() const {
    mjlib::multiplex::MicroDatagramServer::Properties properties{
        .max_size = 64
    };
    
    return properties;
}

void MoteusDronecanTunnel::handle_tunnel_Broadcast(
    const CanardRxTransfer& transfer,
    const uavcan_tunnel_Broadcast& req)
{
    if (!current_read_header_) {
        return;
    }

    if (req.buffer.len < 2) {
        return; // need at least source + destination
    }

    if (req.protocol.protocol != UAVCAN_TUNNEL_PROTOCOL_UNDEFINED ||
        req.channel_id != config_.channel_id) {
        return;
    }

    const uint8_t source      = req.buffer.data[0];
    const uint8_t destination = req.buffer.data[1];

    // If first chunk, initialize header
    if (current_read_header_->size == 0) {
        current_read_header_->source = source;
        current_read_header_->destination = destination;
    }

    uint8_t payload_len = req.buffer.len - 2;

    // The 60th byte = continuation marker
    if (req.buffer.len == sizeof(req.buffer.data)) {
        if (payload_len > 0) {
            payload_len -= 1;
        }
    }

    if (current_read_header_->size + payload_len <= current_read_data_.size()) {
        memcpy(current_read_data_.data() + current_read_header_->size,
               req.buffer.data + 2,
               payload_len);
        current_read_header_->size += payload_len;
    } else {
        // overflow, reset state
        current_read_callback_ = {};
        current_read_header_ = {};
        current_read_data_ = {};
        return;
    }

    // If not full size, this was the last chunk
    if (req.buffer.len < sizeof(req.buffer.data)) {
        auto copy = current_read_callback_;
        auto bytes = current_read_header_->size;

        current_read_callback_ = {};
        current_read_header_ = {};
        current_read_data_ = {};

        copy(mjlib::micro::error_code(), bytes);
    }
}

void MoteusDronecanTunnel::AsyncRead(Header* header,
    const mjlib::base::string_span& data,
    const mjlib::micro::SizeCallback& callback){
    MJ_ASSERT(!current_read_callback_);
    current_read_callback_ = callback;
    current_read_data_ = data;
    current_read_header_ = header;
    current_read_header_->size = 0;
}

void MoteusDronecanTunnel::AsyncWrite(
    const Header& header,
    const std::string_view& data,
    const Header& query_header,
    const mjlib::micro::SizeCallback& callback)
{
    // Reject oversized writes
    if (data.size() > static_cast<size_t>(properties().max_size)) {
        callback(mjlib::micro::error_code(), 0);
        return;
    }

    // We don't preserve BRS or FD flags, as they don't matter
    // when we tunnel through DroneCAN.

    uavcan_tunnel_Broadcast broadcast_msg{};
    broadcast_msg.protocol.protocol = UAVCAN_TUNNEL_PROTOCOL_UNDEFINED;
    broadcast_msg.channel_id = config_.channel_id;

    const size_t max_buf = sizeof(broadcast_msg.buffer.data);  // 60
    const size_t overhead = 2; // source + destination
    const size_t max_payload = max_buf - overhead - 1;

    size_t offset = 0;
    bool success = true;

    while (offset < data.size()) {
        broadcast_msg.buffer.data[0] = 1; // TODO: set correct instance number
        broadcast_msg.buffer.data[1] = 0;

        size_t remaining = data.size() - offset;
        size_t chunk_len = std::min(remaining, max_payload);

        // Copy payload
        std::memcpy(broadcast_msg.buffer.data + overhead,
                    data.data() + offset,
                    chunk_len);

        broadcast_msg.buffer.len = chunk_len + overhead;

        if (remaining > max_payload) {
            // Set len to 60 to mark continuation. Receiver will ignore last byte.
            broadcast_msg.buffer.len = max_buf;
            broadcast_msg.buffer.data[max_buf - 1] = 0; // continuation marker
        }

        if (uavcan_pub_callback_) {
            if (!uavcan_pub_callback_(broadcast_msg)) {
                success = false;
                break;
            }
        }

        offset += chunk_len;
    }

    callback(mjlib::micro::error_code(),
             success ? static_cast<int>(data.size()) : 0);
}
