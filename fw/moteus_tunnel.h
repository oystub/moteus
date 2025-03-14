#pragma once

#include "mjlib/base/visitor.h"
#include "mjlib/micro/persistent_config.h"
#include "mjlib/multiplex/micro_datagram_server.h"

#include <canard/publisher.h>
#include <canard/subscriber.h>
#include <uavcan.tunnel.Broadcast.h>

#include "dronecan_param.h"
#include "dronecan_param_store.h"
#include "canard_iface.h"

class MoteusDronecanTunnel : public mjlib::multiplex::MicroDatagramServer {
    struct Config; // Forward declaration
public:
    MoteusDronecanTunnel(CanardInterface* canard_iface);

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
        uint8_t channel{0};

        template <typename Store>
        void RegisterParameters(Store& store) {
            DRONECAN_PARAMETER(DC_MTUN_CHAN, channel, 0, 0, 255);
        }

        template <typename Archive>
        void Serialize(Archive* a) {
            a->Visit(MJ_NVP(channel));
        }
    } config_;

    void handle_tunnel_Broadcast(const CanardRxTransfer& transfer, const uavcan_tunnel_Broadcast& req);

    CanardInterface* canard_iface_;

    Canard::Publisher<uavcan_tunnel_Broadcast> tunnel_pub{*canard_iface_};
    Canard::ObjCallback<MoteusDronecanTunnel, uavcan_tunnel_Broadcast> tunnel_broadcast_cb{
        this, &MoteusDronecanTunnel::handle_tunnel_Broadcast};
    Canard::Subscriber<uavcan_tunnel_Broadcast> tunnel_sub{
        tunnel_broadcast_cb, 0};
    mjlib::micro::SizeCallback current_read_callback_;
    Header* current_read_header_ = nullptr;
    mjlib::base::string_span current_read_data_;
};
