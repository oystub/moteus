#pragma once

#include "fdcan.h"
#include "mjlib/base/visitor.h"
#include "mjlib/micro/persistent_config.h"
#include "mjlib/micro/async_stream.h"
#include "moteus_controller.h"

#include "canard.h"

#include <uavcan.protocol.GetNodeInfo.h>
#include <uavcan.protocol.NodeStatus.h>
#include <uavcan.protocol.param.GetSet_res.h>
#include <uavcan.protocol.param.Value.h>
#include <uavcan.protocol.debug.LogLevel.h>

namespace Swashplateless {
    constexpr float pi = 3.14159265358979323846f;

enum class ParamType {
    INT8,
    INT16,
    INT32,
    INT64,
    UINT8,
    UINT16,
    UINT32,
    UINT64,
    FLOAT32,
    BOOLEAN,
    CHAR32,
};

union ParamValue {
    int64_t i;
    float f;
    const char *c;
};

struct DroneCANParam {
    const char *name;
    ParamType type;
    void * value_ptr;
    ParamValue default_value;
    ParamValue min_value;
    ParamValue max_value;

    template <typename Archive> void Serialize(Archive *a) const {
        switch (type) {
        case ParamType::INT8:
            a->Visit(mjlib::base::MakeNameValuePair<int8_t>(
                reinterpret_cast<int8_t *>(value_ptr), name));
            break;
        case ParamType::INT16:
            a->Visit(mjlib::base::MakeNameValuePair<int16_t>(
                reinterpret_cast<int16_t *>(value_ptr), name));
            break;
        case ParamType::INT32:
            a->Visit(mjlib::base::MakeNameValuePair<int32_t>(
                reinterpret_cast<int32_t *>(value_ptr), name));
            break;
        case ParamType::INT64:
            a->Visit(mjlib::base::MakeNameValuePair<int64_t>(
                reinterpret_cast<int64_t *>(value_ptr), name));
            break;
        case ParamType::UINT8:
            a->Visit(mjlib::base::MakeNameValuePair<uint8_t>(
                reinterpret_cast<uint8_t *>(value_ptr), name));
            break;
        case ParamType::UINT16:
            a->Visit(mjlib::base::MakeNameValuePair<uint16_t>(
                reinterpret_cast<uint16_t *>(value_ptr), name));
            break;
        case ParamType::UINT32:
            a->Visit(mjlib::base::MakeNameValuePair<uint32_t>(
                reinterpret_cast<uint32_t *>(value_ptr), name));
            break;
        case ParamType::UINT64:
            a->Visit(mjlib::base::MakeNameValuePair<uint64_t>(
                reinterpret_cast<uint64_t *>(value_ptr), name));
            break;
        case ParamType::FLOAT32:
            a->Visit(mjlib::base::MakeNameValuePair<float>(
                reinterpret_cast<float *>(value_ptr), name));
            break;
        case ParamType::BOOLEAN:
            a->Visit(mjlib::base::MakeNameValuePair<bool>(
                reinterpret_cast<bool *>(value_ptr), name));
            break;
        case ParamType::CHAR32:
            // We split the string into 4 uint64_t's, since persistent config
            // doesn't support strings
            for (int i = 0; i < 4; i++) {
                char name_with_index[32];
                snprintf(name_with_index, 32, "%s_%d", this->name, i);
                uint64_t *ptr = reinterpret_cast<uint64_t *>(value_ptr) + i;
                a->Visit(mjlib::base::MakeNameValuePair<uint64_t>(
                    ptr, name_with_index));
            }
            break;
        default:
            // Unknow type
            break;
        }
    }

    void setValue(const ParamValue &value) const;
    void setValue(const uavcan_protocol_param_Value &value) const;
    uavcan_protocol_param_GetSetResponse toParamGetSetResponse() const;
};

struct SwashplatelessPersistentConfig {
    uint8_t dc_node_id{0};
    bool dc_dna{true};
    uint8_t dc_esc_idx{0};
    uint8_t dc_azm_idx{0};
    uint8_t dc_elv_idx{0};
    uint64_t esc_cmd_timeout_us{0};
    uint64_t act_cmd_timeout_us{0};

    int8_t mot_dir{0};
    float mot_max_rpm{0};
    float mot_max_acc{0};
    float mot_max_elev_deg{0};
    float mot_start_rpm{0};
    float mot_start_acc{0};

    uint8_t cal_mode{0};
    float cal_elv_gain{0};
    float cal_azm_offset_deg{0};

    uint64_t dbg_disarm_after_us{0};

    char name[32]{0};
};

struct SwashplatelessTempConfig {
    bool dbg_silent{true};
};

class Config {
  public:
    SwashplatelessPersistentConfig config;
    SwashplatelessTempConfig tmpcfg;
    DroneCANParam params[19] = {
        {"DC_NODE_ID",
         ParamType::UINT8,
         &config.dc_node_id,
         {.i = 42},
         {.i = 0},
         {.i = 127}},
        {"DC_DNA",
         ParamType::BOOLEAN,
         &config.dc_dna,
         {.i = 0},
         {.i = 0},
         {.i = 1}},
        {"DC_ESC_IDX",
         ParamType::UINT8,
         &config.dc_esc_idx,
         {.i = 0},
         {.i = 0},
         {.i = 19}},
        {"DC_AZM_IDX",
         ParamType::UINT8,
         &config.dc_azm_idx,
         {.i = 0},
         {.i = 0},
         {.i = 127}},
        {"DC_ELV_IDX",
         ParamType::UINT8,
         &config.dc_elv_idx,
         {.i = 1},
         {.i = 0},
         {.i = 127}},
        {"ESC_CMD_TIMEOUT",
         ParamType::UINT64,
         &config.esc_cmd_timeout_us,
         {.i = 10000},
         {.i = 0},
         {.i = 10000000}},
        {"ACT_CMD_TIMEOUT",
         ParamType::UINT64,
         &config.act_cmd_timeout_us,
         {.i = 10000},
         {.i = 0},
         {.i = 10000000}},
        {"MOT_DIR",
         ParamType::INT8,
         &config.mot_dir,
         {.i = 1},
         {.i = -1},
         {.i = 1}},
        {"MOT_MAX_RPM",
         ParamType::FLOAT32,
         &config.mot_max_rpm,
         {.f = 5100},
         {.f = 0},
         {.f = 10000}},
        {"MOT_MAX_ACC",
         ParamType::FLOAT32,
         &config.mot_max_acc,
         {.f = 60000},
         {.f = 0},
         {.f = 100000}},
        {"MOT_MAX_ELEV_DEG",
         ParamType::FLOAT32,
         &config.mot_max_elev_deg,
         {.f = 20},
         {.f = 0},
         {.f = 90}},
        {"MOT_START_RPM",
         ParamType::FLOAT32,
         &config.mot_start_rpm,
         {.f = 1200},
         {.f = 0},
         {.f = 10000}},
        {"MOT_START_ACC",
         ParamType::FLOAT32,
         &config.mot_start_acc,
         {.f = 6000},
         {.f = 0},
         {.f = 100000}},
        {"CAL_MODE",
         ParamType::UINT8,
         &config.cal_mode,
         {.i = 1},
         {.i = 0},
         {.i = 1}},
        {"CAL_ELV_GAIN",
         ParamType::FLOAT32,
         &config.cal_elv_gain,
         {.f = 1.16},
         {.f = 0},
         {.f = 10}},
        {"CAL_AZM_OFFSET_DEG",
         ParamType::FLOAT32,
         &config.cal_azm_offset_deg,
         {.f = 90},
         {.f = 0},
         {.f = 360}},
        {"DBG_SILENT",
         ParamType::BOOLEAN,
         &tmpcfg.dbg_silent,
         {.i = 0},
         {.i = 0},
         {.i = 1}},
        {"DBG_DISARM_AFTER",
         ParamType::UINT64,
         &config.dbg_disarm_after_us,
         {.i = 0},
         {.i = 0},
         {.i = 1000000000}},
        {"NAME",
         ParamType::CHAR32,
         &config.name,
         {.c = "swashplateless"},
         {0},
         {0}},
    };
    template <typename Archive> void Serialize(Archive *a) const {
        for (const auto &param : params) {
            param.Serialize(a);
        }
    }
};

class SwashplatelessController {
  public:
    SwashplatelessController(moteus::FDCan *can,
                             moteus::MoteusController *controller,  mjlib::micro::AsyncStream* stream);
    void initStorage(mjlib::micro::PersistentConfig &storage);
    void initCanard();
    void pollMillisecond();
    void poll();

    enum class RotorMode {
        DISARMED=0,
        IDLE=1,
        SPINUP=2,
        RUNNING=3,
    };

    struct RotorState {
        RotorMode mode{RotorMode::DISARMED};
        uint64_t start_time_usec{0};
        float throttle_input{0};
        float azimuth_input{0};
        float elevation_input{0};
        uint64_t last_throttle_input_time_usec{0};
        uint64_t last_azimuth_input_time_usec{0};
        uint64_t last_elevation_input_time_usec{0};
    } rotor_state_;

    struct DNAState {
        uint32_t send_next_node_id_allocation_request_at_ms{0};
        uint32_t node_id_allocation_unique_id_offset{0};
    } dna_state_;

  private:
    moteus::FDCan *can_;
    CanardInstance canard_;
    moteus::MoteusController *controller_;
    mjlib::micro::AsyncStream* stream_;
    mjlib::micro::PersistentConfig *persistent_config_{nullptr};
    uint64_t microsecond_counter_{0};
    uint8_t memory_pool_[512];

    void processTxRx(uint64_t timestamp_usec);

    static void onTransferReceived(CanardInstance *ins,
                                   CanardRxTransfer *transfer);
    static bool shouldAcceptTransfer(const CanardInstance *ins,
                                     uint64_t *out_data_type_signature,
                                     uint16_t data_type_id,
                                     CanardTransferType transfer_type,
                                     uint8_t source_node_id);

                                     
    void sendNodeStatus();
    void sendESCStatus();
    void sendLogMessage(const char* source, const char* text, uint8_t level = UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG);

    static void getUniqueID(uint8_t id[16]);
    void requestDNA();
    uint64_t micros64();
    uint32_t millis32();
    bool isInputTimeout();

    void handleGetNodeInfo(CanardInstance *ins, CanardRxTransfer *transfer);
    void handleParamGetSet(CanardInstance *ins, CanardRxTransfer *transfer);
    void handleExecuteOpcode(CanardInstance *ins, CanardRxTransfer *transfer);
    void handleRawCommand(CanardInstance *ins, CanardRxTransfer *transfer);
    void handleArrayCommand(CanardInstance *ins, CanardRxTransfer *transfer);
    void handleDNAAllocation(CanardInstance *ins, CanardRxTransfer *transfer); //TODO

    void updateMotor(); //TODO
};

size_t copy_utf8(char *dst, const char *src, size_t max_bytes);

}   // namespace DroneCAN
