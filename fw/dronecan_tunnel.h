#pragma once

#include <functional>

#include "mjlib/base/visitor.h"
#include "mjlib/micro/persistent_config.h"
#include "mjlib/multiplex/micro_datagram_server.h"

#include <canard/publisher.h>
#include <canard/subscriber.h>
#include <uavcan.tunnel.Broadcast.h>

#include "dronecan_param.h"
#include "dronecan_param_store.h"

class MoteusDronecanTunnel : public mjlib::multiplex::MicroDatagramServer {
    struct Config; // Forward declaration
public:
    MoteusDronecanTunnel(const std::function<bool(const uavcan_tunnel_Broadcast&)>& callback);
    MoteusDronecanTunnel();

    void handle_tunnel_Broadcast(const CanardRxTransfer& transfer, const uavcan_tunnel_Broadcast& req);
    void set_uavcan_pub_callback(
        const std::function<bool(uavcan_tunnel_Broadcast&)> callback) {
        uavcan_pub_callback_ = callback;
    }

    constexpr mjlib::multiplex::MicroDatagramServer::Properties properties() const override;
    void AsyncRead(Header* header,
                           const mjlib::base::string_span& data,
                           const mjlib::micro::SizeCallback& callback) override;
    void AsyncWrite(const Header& header,
                            const std::string_view& data,
                            const Header& query_header,
                            const mjlib::micro::SizeCallback& callback) override;
    Config* config() { return &config_; }
   
private:
    struct Config {
        uint8_t channel_id{0};

        template <typename Store>
        void RegisterParameters(Store& store) {
            DRONECAN_PARAMETER(DC_MTUN_CHANID, channel_id, 0, 0, 255);
        }

        template <typename Archive>
        void Serialize(Archive* a) {
            a->Visit(MJ_NVP(channel_id));
        }
    } config_;

    mjlib::micro::SizeCallback current_read_callback_;
    Header* current_read_header_ = nullptr;
    mjlib::base::string_span current_read_data_;

    std::function<bool(uavcan_tunnel_Broadcast&)> uavcan_pub_callback_;
};