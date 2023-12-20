#include "swashplateless.h"
#include "canard.h"
#include "fw/canard.h"

#include "mjlib/micro/error_code.h"

#include <uavcan.equipment.actuator.ArrayCommand.h>
#include <uavcan.equipment.actuator.Command.h>
#include <uavcan.equipment.esc.RawCommand.h>
#include <uavcan.protocol.GetNodeInfo.h>
#include <uavcan.protocol.NodeStatus.h>
#include <uavcan.protocol.dynamic_node_id.Allocation.h>
#include <uavcan.protocol.param.ExecuteOpcode.h>
#include <uavcan.protocol.param.GetSet.h>
#include <uavcan.protocol.param.GetSet_req.h>
#include <uavcan.protocol.param.GetSet_res.h>
#include <uavcan.protocol.param.Value.h>
#include <uavcan.protocol.debug.LogMessage.h>

namespace Swashplateless {
Config cfg;

void
DroneCANParam::setValue(const ParamValue &value) const {
    // Check that param is within bounds
    switch (type) {
    case ParamType::INT8:
    case ParamType::INT16:
    case ParamType::INT32:
    case ParamType::INT64:
    case ParamType::UINT8:
    case ParamType::UINT16:
    case ParamType::UINT32:
    case ParamType::UINT64:
        if (value.i < min_value.i || value.i > max_value.i) {
            return;
        }
        break;
    case ParamType::FLOAT32:
        if (value.f < min_value.f || value.f > max_value.f) {
            return;
        }
        break;
    case ParamType::BOOLEAN:
        if (value.i != 0 && value.i != 1) {
            return;
        }
        break;
    case ParamType::CHAR32:
        // No min/max for strings
        break;
    default:
        // Unknown type
        return;
    }

    switch (type) {
    case ParamType::INT8:
        *reinterpret_cast<int8_t *>(value_ptr) = value.i;
        break;
    case ParamType::INT16:
        *reinterpret_cast<int16_t *>(value_ptr) = value.i;
        break;
    case ParamType::INT32:
        *reinterpret_cast<int32_t *>(value_ptr) = value.i;
        break;
    case ParamType::INT64:
        *reinterpret_cast<int64_t *>(value_ptr) = value.i;
        break;
    case ParamType::UINT8:
        *reinterpret_cast<uint8_t *>(value_ptr) = value.i;
        break;
    case ParamType::UINT16:
        *reinterpret_cast<uint16_t *>(value_ptr) = value.i;
        break;
    case ParamType::UINT32:
        *reinterpret_cast<uint32_t *>(value_ptr) = value.i;
        break;
    case ParamType::UINT64:
        *reinterpret_cast<uint64_t *>(value_ptr) = value.i;
        break;
    case ParamType::FLOAT32:
        *reinterpret_cast<float *>(value_ptr) = value.f;
        break;
    case ParamType::BOOLEAN:
        *reinterpret_cast<bool *>(value_ptr) = value.i;
        break;
    case ParamType::CHAR32:
        copy_utf8(static_cast<char *>(value_ptr), value.c, 32);
        break;
    default:
        // Unknown type
        break;
    }
}

void
DroneCANParam::setValue(const uavcan_protocol_param_Value &value) const {
    switch (type) {
    case ParamType::INT8:
    case ParamType::INT16:
    case ParamType::INT32:
    case ParamType::INT64:
    case ParamType::UINT8:
    case ParamType::UINT16:
    case ParamType::UINT32:
    case ParamType::UINT64:
        if (value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE) {
            // Wrong type
            return;
        }
        return setValue(ParamValue{.i = value.integer_value});
    case ParamType::FLOAT32:
        if (value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE) {
            // Wrong type
            return;
        }
        return setValue(ParamValue{.f = value.real_value});
    case ParamType::BOOLEAN:
        // We support both int and bool for boolean values, for QGC
        // compatibility
        if (value.union_tag == UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE) {
            return setValue(ParamValue{.i = value.boolean_value});
        } else if (value.union_tag ==
                   UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE) {
            return setValue(ParamValue{.i = value.integer_value});
        } else {
            // Wrong type
            return;
        }
    case ParamType::CHAR32:
        if (value.union_tag != UAVCAN_PROTOCOL_PARAM_VALUE_STRING_VALUE) {
            // Wrong type
            return;
        }
        // UAVCAN strings are not null terminated, so we need to add a null if
        // the string is shoter than 32 bytes
        if (value.string_value.len < 32) {
            ((char *) value.string_value.data)[value.string_value.len] = '\0';
        }
        return setValue(
            ParamValue{.c = (const char *) value.string_value.data});
    default:
        // Unknown type
        return;
    }
}

uavcan_protocol_param_GetSetResponse
DroneCANParam::toParamGetSetResponse() const {
    struct uavcan_protocol_param_GetSetResponse res;
    memset(&res, 0, sizeof(res));

    // Get name
    strncpy((char *) res.name.data, name, sizeof(res.name.data) - 1);
    res.name.len = strlen((char *) res.name.data);

    // Get value
    switch (type) {
    case ParamType::INT8:
        res.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
        res.value.integer_value = *reinterpret_cast<int8_t *>(value_ptr);
        break;
    case ParamType::INT16:
        res.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
        res.value.integer_value = *reinterpret_cast<int16_t *>(value_ptr);
        break;
    case ParamType::INT32:
        res.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
        res.value.integer_value = *reinterpret_cast<int32_t *>(value_ptr);
        break;
    case ParamType::INT64:
        res.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
        res.value.integer_value = *reinterpret_cast<int64_t *>(value_ptr);
        break;
    case ParamType::UINT8:
        res.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
        res.value.integer_value = *reinterpret_cast<uint8_t *>(value_ptr);
        break;
    case ParamType::UINT16:
        res.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
        res.value.integer_value = *reinterpret_cast<uint16_t *>(value_ptr);
        break;
    case ParamType::UINT32:
        res.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
        res.value.integer_value = *reinterpret_cast<uint32_t *>(value_ptr);
        break;
    case ParamType::UINT64:
        res.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
        res.value.integer_value = *reinterpret_cast<uint64_t *>(value_ptr);
        break;
    case ParamType::FLOAT32:
        res.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
        res.value.real_value = *reinterpret_cast<float *>(value_ptr);
        break;
    case ParamType::BOOLEAN:
        res.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE;
        res.value.boolean_value = *reinterpret_cast<bool *>(value_ptr);
        break;
    case ParamType::CHAR32:
        res.value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_STRING_VALUE;
        strncpy((char *) res.value.string_value.data, (char *) value_ptr, 32);
        res.value.string_value.len =
            strlen((char *) res.value.string_value.data);
        break;
    default:
        // Unknown type
        break;
    }

    // Get defaut, min and max values
    switch (type) {
    case ParamType::INT8:
    case ParamType::INT16:
    case ParamType::INT32:
    case ParamType::INT64:
    case ParamType::UINT8:
    case ParamType::UINT16:
    case ParamType::UINT32:
    case ParamType::UINT64:
        res.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_INTEGER_VALUE;
        res.default_value.integer_value = default_value.i;
        res.max_value.union_tag =
            UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
        res.max_value.integer_value = max_value.i;
        res.min_value.union_tag =
            UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_INTEGER_VALUE;
        res.min_value.integer_value = min_value.i;
        break;
    case ParamType::FLOAT32:
        res.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_REAL_VALUE;
        res.default_value.real_value = default_value.f;
        res.max_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_REAL_VALUE;
        res.max_value.real_value = max_value.f;
        res.min_value.union_tag = UAVCAN_PROTOCOL_PARAM_NUMERICVALUE_REAL_VALUE;
        res.min_value.real_value = min_value.f;
        break;
    case ParamType::BOOLEAN:
        res.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_BOOLEAN_VALUE;
        res.default_value.boolean_value = default_value.i;
        // Min and max are not used for boolean values
        break;
    case ParamType::CHAR32:
        res.default_value.union_tag = UAVCAN_PROTOCOL_PARAM_VALUE_STRING_VALUE;
        strncpy((char *) res.default_value.string_value.data, default_value.c,
                32);
        res.default_value.string_value.len =
            strlen((const char *) res.default_value.string_value.data);
        // Min and max values are not used for string parameters
        break;
    default:
        // Unknown type
        break;
    }

    return res;
}

size_t
copy_utf8(char *dst, const char *src, size_t max_bytes) {
    if (max_bytes == 0) {
        return 0;
    }
    size_t i = 0;
    while (i < max_bytes - 1) {
        dst[i] = src[i];
        if (src[i] == '\0') {
            // We have reached the end of the string
            return i;
        }
        ++i;
    }
    // If the string is too long, we need to truncate it
    // UTF-8 characters can be 1-4 bytes long, so we need to find the last
    // character that fits.
    while (i > 0 && (src[i] & 0b11000000) == 0b10000000) {
        // This is a UTF-8 continuation byte (starts with 0b10), so we are
        // in the middle of a character. We need to go back until we find
        // the start of the character.
        --i;
    }
    dst[i] = '\0';
    return i;
}

SwashplatelessController::SwashplatelessController(
    moteus::FDCan *can, moteus::MoteusController *controller,
    mjlib::micro::AsyncStream *stream) {
    can_ = can;
    controller_ = controller;
    stream_ = stream;
}

void
SwashplatelessController::initStorage(
    mjlib::micro::PersistentConfig &persistentConfig) {

    persistent_config_ = &persistentConfig;

    const auto numParams = sizeof(cfg.params) / sizeof(cfg.params[0]);
    for (size_t i = 0; i < numParams; i++) {
        auto &param = cfg.params[i];
        param.setValue(param.default_value);
    }

    persistentConfig.Register("dc", &cfg,
                              []() { cfg.tmpcfg.dbg_silent = false; });
}

void SwashplatelessController::initCanard() {
        canardInit(&canard_, memory_pool_, sizeof(memory_pool_), onTransferReceived,
               shouldAcceptTransfer, this);

    // If DNA is enabled, we start in anonymous mode
    if (cfg.config.dc_dna > 0) {
        canardSetLocalNodeID(&canard_, CANARD_BROADCAST_NODE_ID);
    } else {
        canardSetLocalNodeID(&canard_, cfg.config.dc_node_id);
    }
}

void
SwashplatelessController::processTxRx(uint64_t timestamp_usec) {
    // Check CAN bus status, and recover from bus off if necessary.
    const auto status = can_->status();
    if (status.BusOff) {
        can_->RecoverBusOff();
    }

    // Send out pending frames
    for (const CanardCANFrame *txf = NULL;
         (txf = canardPeekTxQueue(&canard_)) != NULL;) {
        // If we are in silent mode, we don't actually send anything.
        if (cfg.tmpcfg.dbg_silent) {
            canardPopTxQueue(&canard_);
            continue;
        }

        if (!can_->ReadyForSend()) {
            // The CAN peripheral is not ready to send, so we'll
            // try again later.
            break;
        }

        std::string_view str(reinterpret_cast<const char *>(txf->data),
                             txf->data_len);

        // Need to send the frame as a standard CAN frame.
        moteus::FDCan::SendOptions send_options;
        send_options.bitrate_switch = moteus::FDCan::Override::kDisable;
        send_options.fdcan_frame = moteus::FDCan::Override::kDisable;
        send_options.remote_frame = moteus::FDCan::Override::kDisable;
        send_options.extended_id = moteus::FDCan::Override::kRequire;
        can_->Send(txf->id, str, send_options);
        // Remove the frame from the queue.
        canardPopTxQueue(&canard_);
    }

    FDCAN_RxHeaderTypeDef fdcan_header = {};
    uint8_t buf[8];

    const bool got_data = can_->PollDroneCAN(&fdcan_header, buf);
    if (got_data) {
        // Uavcan frames are always CAN and never CAN FD.
        if (fdcan_header.FDFormat != FDCAN_CLASSIC_CAN) {
            return;
        }
        // Uavcan frames use the extended identifier space.
        if (fdcan_header.IdType != FDCAN_EXTENDED_ID) {
            return;
        }
        // Ignore errors.
        if (fdcan_header.ErrorStateIndicator != FDCAN_ESI_ACTIVE) {
            return;
        }

        CanardCANFrame frame;
        frame.iface_id = 1;
        frame.id = fdcan_header.Identifier;
        frame.id &= 0x1FFFFFFF;
        if (fdcan_header.RxFrameType == FDCAN_REMOTE_FRAME) {
            frame.id |= CANARD_CAN_FRAME_RTR;
        }
        if (fdcan_header.ErrorStateIndicator == FDCAN_ESI_ACTIVE) {
            frame.id |= CANARD_CAN_FRAME_ERR;
        }
        if (fdcan_header.IdType == FDCAN_EXTENDED_ID) {
            frame.id |= CANARD_CAN_FRAME_EFF;
        }
        frame.id |= CANARD_CAN_FRAME_EFF;
        frame.id &= ~(CANARD_CAN_FRAME_ERR | CANARD_CAN_FRAME_RTR);
        frame.data_len = static_cast<uint8_t>(
            moteus::FDCan::ParseDlc(fdcan_header.DataLength));
        memset(frame.data, 0, 8);
        memcpy(frame.data, buf, min(frame.data_len, uint8_t(8)));
        auto err = canardHandleRxFrame(&canard_, &frame, timestamp_usec);
        if (err) {
        }
    }
}

void
SwashplatelessController::onTransferReceived(CanardInstance *ins,
                                             CanardRxTransfer *transfer) {
    auto controller =
        static_cast<SwashplatelessController *>(ins->user_reference);
    if (transfer->transfer_type == CanardTransferTypeRequest) {
        switch (transfer->data_type_id) {
        case UAVCAN_PROTOCOL_GETNODEINFO_ID: {
            controller->handleGetNodeInfo(ins, transfer);
            break;
        }
        case UAVCAN_PROTOCOL_PARAM_GETSET_ID: {
            controller->handleParamGetSet(ins, transfer);
            break;
        }
        case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID: {
            controller->handleExecuteOpcode(ins, transfer);
            break;
        }
        default:
            break;
        }
    }
    if (transfer->transfer_type == CanardTransferTypeBroadcast) {
        switch (transfer->data_type_id) {
        case UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID: {
            controller->handleRawCommand(ins, transfer);
            break;
        }
        case UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID: {
            controller->handleArrayCommand(ins, transfer);
            break;
        }
        case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID: {
            controller->handleDNAAllocation(ins, transfer);
            break;
        }
        default:
            break;
        }
    }
}

bool
SwashplatelessController::shouldAcceptTransfer(
    const CanardInstance *ins, uint64_t *out_data_type_signature,
    uint16_t data_type_id, CanardTransferType transfer_type,
    uint8_t source_node_id) {
    if (transfer_type == CanardTransferTypeRequest) {
        switch (data_type_id) {
        case UAVCAN_PROTOCOL_GETNODEINFO_ID: {
            *out_data_type_signature = UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_PARAM_GETSET_ID: {
            *out_data_type_signature = UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID: {
            *out_data_type_signature =
                UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE;
            return true;
        }
        default:
            break;
        }
    }
    if (transfer_type == CanardTransferTypeBroadcast) {
        switch (data_type_id) {
        case UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_ID: {
            *out_data_type_signature =
                UAVCAN_EQUIPMENT_ESC_RAWCOMMAND_SIGNATURE;
            return true;
        }
        case UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_ID: {
            *out_data_type_signature =
                UAVCAN_EQUIPMENT_ACTUATOR_ARRAYCOMMAND_SIGNATURE;
            return true;
        }
        case UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID: {
            *out_data_type_signature =
                UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE;
            return true;
        }
        default:
            break;
        }
    }
    return false;
}

void
SwashplatelessController::poll() {
    processTxRx(microsecond_counter_);

    if (canardGetLocalNodeID(&canard_) == CANARD_BROADCAST_NODE_ID){
        if (millis32() >= dna_state_.send_next_node_id_allocation_request_at_ms) {
            requestDNA();
        }
    }
}

void
SwashplatelessController::pollMillisecond() {
    microsecond_counter_ += 1000;
    updateMotor();
    if (microsecond_counter_ % 1000000 == 0) {
        sendNodeStatus();
        char buf[128];
        snprintf(buf, sizeof(buf), "Motor mode: %d, throttle: %f, azm: %f, elv: %f", (int)rotor_state_.mode, (double)rotor_state_.throttle_input, (double)rotor_state_.azimuth_input, (double)rotor_state_.elevation_input);
        sendLogMessage(cfg.config.name, buf, UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG);
    }
    if (microsecond_counter_ % 100000 == 0) {
        // Sens ESC status every 100 ms
        // sendESCStatus();
    }
}

void
SwashplatelessController::sendNodeStatus() {
    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];

    struct uavcan_protocol_NodeStatus msg;
    msg.uptime_sec = microsecond_counter_ / 1000000ULL;
    // TODO: Send actual health and mode
    msg.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    msg.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;

    msg.sub_mode = 0;   // Spec says this should be set to zero

    // We send motor mode in the first byte of the vendor specific status
    // code
    ((int8_t*)&msg.vendor_specific_status_code)[0] = (int8_t)rotor_state_.mode;
    ((int8_t*)&msg.vendor_specific_status_code)[1] = 0; // Reserved

    uint32_t len = uavcan_protocol_NodeStatus_encode(&msg, buffer);
    static uint8_t transfer_id;
    canardBroadcast(&canard_, UAVCAN_PROTOCOL_NODESTATUS_SIGNATURE,
                    UAVCAN_PROTOCOL_NODESTATUS_ID, &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW, buffer, len);
}

void SwashplatelessController::sendLogMessage(const char* source, const char* text, uint8_t level){
    struct uavcan_protocol_debug_LogMessage msg;
    memset(&msg, 0, sizeof(msg));
    uint8_t buffer[UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_MAX_SIZE];

    msg.level.value = level;
    
    strncpy((char*)msg.source.data, source, sizeof(msg.source.data));
    msg.source.data[30] = '\0';
    msg.source.len = strlen((char*)msg.source.data);

    strncpy((char*)msg.text.data, text, sizeof(msg.text.data));
    msg.text.data[89] = '\0';
    msg.text.len = strlen((char*)msg.text.data);

    uint32_t len = uavcan_protocol_debug_LogMessage_encode(&msg, buffer);
    static uint8_t transfer_id;

    canardBroadcast(&canard_,
                    UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_SIGNATURE,
                    UAVCAN_PROTOCOL_DEBUG_LOGMESSAGE_ID,
                    &transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    len);
}

void
SwashplatelessController::handleGetNodeInfo(CanardInstance *ins,
                                            CanardRxTransfer *transfer) {
    if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID) {
        // Ignore if source is broadcast
        return;
    }

    uint8_t buffer[UAVCAN_PROTOCOL_GETNODEINFO_RESPONSE_MAX_SIZE];
    struct uavcan_protocol_GetNodeInfoResponse msg;
    memset(&msg, 0, sizeof(msg));

    msg.status.uptime_sec = microsecond_counter_ / 1000000ULL;

    // TODO: Send actual health and mode
    msg.status.health = UAVCAN_PROTOCOL_NODESTATUS_HEALTH_OK;
    msg.status.mode = UAVCAN_PROTOCOL_NODESTATUS_MODE_OPERATIONAL;

    msg.status.sub_mode = 0;
    // We send motor mode in the first byte of the vendor specific status
    // code
    ((int8_t*)&msg.status.vendor_specific_status_code)[0] = (int8_t)rotor_state_.mode;
    ((int8_t*)&msg.status.vendor_specific_status_code)[1] = 0; // Reserved

    // TODO: Generate from git
    msg.software_version.major = 1;
    msg.software_version.minor = 0;
    msg.software_version.optional_field_flags = 0;
    msg.software_version.vcs_commit = 0;

    // TODO: Use actual hardware version
    msg.hardware_version.major = 1;
    msg.hardware_version.minor = 0;

    getUniqueID(msg.hardware_version.unique_id);

    memset(msg.name.data, 0, sizeof(msg.name.data));
    strncpy((char *) msg.name.data, cfg.config.name, sizeof(msg.name.data));
    msg.name.len = strlen((char *) msg.name.data);

    uint16_t total_size =
        uavcan_protocol_GetNodeInfoResponse_encode(&msg, buffer);

    canardRequestOrRespond(&canard_, transfer->source_node_id,
                           UAVCAN_PROTOCOL_GETNODEINFO_SIGNATURE,
                           UAVCAN_PROTOCOL_GETNODEINFO_ID,
                           &transfer->transfer_id, CANARD_TRANSFER_PRIORITY_LOW,
                           CanardResponse, &buffer[0], total_size);
}

void
SwashplatelessController::handleParamGetSet(CanardInstance *ins,
                                            CanardRxTransfer *transfer) {
    struct uavcan_protocol_param_GetSetRequest req;
    if (uavcan_protocol_param_GetSetRequest_decode(transfer, &req) > 0) {
        // Something went wrong decoding the message
        return;
    }
    req.name.data[req.name.len] = '\0';   // Make sure the string is null
                                          // terminated

    bool is_by_name = req.name.len > 0;
    bool is_set_request = is_by_name && (req.value.union_tag !=
                                         UAVCAN_PROTOCOL_PARAM_VALUE_EMPTY);

    const DroneCANParam *param = nullptr;
    auto num_params = sizeof(cfg.params) / sizeof(cfg.params[0]);

    for (size_t i = 0; i < num_params; i++) {
        if (is_by_name) {
            if (strcmp(cfg.params[i].name,
                       reinterpret_cast<char *>(req.name.data)) == 0) {
                param = &cfg.params[i];
                break;
            }
        } else {
            if (i == req.index) {
                param = &cfg.params[i];
                break;
            }
        }
    }

    // Prepare a response struct
    struct uavcan_protocol_param_GetSetResponse res;
    memset(&res, 0, sizeof(res));

    if (param != nullptr) {
        // A parameter was found
        if (is_set_request) {
            // Try to set the parameter
            param->setValue(req.value);
        }
        res = param->toParamGetSetResponse();
    }

    // Send the response
    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_GETSET_RESPONSE_MAX_SIZE];
    uint16_t total_size =
        uavcan_protocol_param_GetSetResponse_encode(&res, buffer);
    canardRequestOrRespond(
        ins, transfer->source_node_id, UAVCAN_PROTOCOL_PARAM_GETSET_SIGNATURE,
        UAVCAN_PROTOCOL_PARAM_GETSET_ID, &transfer->transfer_id,
        transfer->priority, CanardResponse, &buffer[0], total_size);
}

void
SwashplatelessController::handleExecuteOpcode(CanardInstance *ins,
                                              CanardRxTransfer *transfer) {
    struct uavcan_protocol_param_ExecuteOpcodeRequest req;
    if (uavcan_protocol_param_ExecuteOpcodeRequest_decode(transfer, &req) > 0) {
        // Something went wrong decoding the message
        return;
    }

    // Prepare a response struct
    struct uavcan_protocol_param_ExecuteOpcodeResponse res;
    memset(&res, 0, sizeof(res));

    switch (req.opcode) {
    case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_SAVE:
        // Save parameters to flash
        if (persistent_config_ != nullptr) {
            persistent_config_->Write();
            res.ok = true;
        }
        break;
    case UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_REQUEST_OPCODE_ERASE:
        // We reset everything to default values
        for (auto &param : cfg.params) {
            param.setValue(param.default_value);
        }
        if (persistent_config_ != nullptr) {
            persistent_config_->Write();
            res.ok = true;
        }
        break;
    };

    // Send the response
    uint8_t buffer[UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_RESPONSE_MAX_SIZE];
    uint16_t total_size =
        uavcan_protocol_param_ExecuteOpcodeResponse_encode(&res, buffer);
    canardRequestOrRespond(ins, transfer->source_node_id,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_SIGNATURE,
                           UAVCAN_PROTOCOL_PARAM_EXECUTEOPCODE_ID,
                           &transfer->transfer_id, transfer->priority,
                           CanardResponse, &buffer[0], total_size);
}

void
SwashplatelessController::handleRawCommand(CanardInstance *ins,
                                           CanardRxTransfer *transfer) {
    struct uavcan_equipment_esc_RawCommand msg;
    if (uavcan_equipment_esc_RawCommand_decode(transfer, &msg) > 0) {
        // Something went wrong decoding the message
        return;
    }
    if (msg.cmd.len <= cfg.config.dc_esc_idx) {
        // Does not contain a command for this ESC. We interpret this as a zero
        // throttle command.
        rotor_state_.throttle_input = 0;
    } else {
        // DroneCAN throttle command is [-8192, 8191]
        // We disregard negative values, and remap to [0, 1]
        auto clamped_throttle = clamp(msg.cmd.data[cfg.config.dc_esc_idx],
                                      int16_t{0}, int16_t{8191});
        rotor_state_.throttle_input = clamped_throttle / 8191.0f;
    }

    rotor_state_.last_throttle_input_time_usec = micros64();

    // We update the motor immediately
    updateMotor();
}

void
SwashplatelessController::handleArrayCommand(CanardInstance *ins,
                                             CanardRxTransfer *transfer) {
    struct uavcan_equipment_actuator_ArrayCommand msg;
    if (uavcan_equipment_actuator_ArrayCommand_decode(transfer, &msg) > 0) {
        // Something went wrong decoding the message
        return;
    }

    for (uint8_t i = 0; i < msg.commands.len; ++i) {
        // Check for azimuth command
        if (msg.commands.data[i].actuator_id == cfg.config.dc_azm_idx) {
            // We currently only support unitless commands
            if (msg.commands.data[i].command_type ==
                UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_UNITLESS) {
                rotor_state_.azimuth_input = msg.commands.data[i].command_value;
                rotor_state_.last_azimuth_input_time_usec = micros64();
            }
        }
        // Check for elevation command
        if (msg.commands.data[i].actuator_id == cfg.config.dc_elv_idx) {
            // We currently only support unitless commands
            if (msg.commands.data[i].command_type ==
                UAVCAN_EQUIPMENT_ACTUATOR_COMMAND_COMMAND_TYPE_UNITLESS) {
                rotor_state_.elevation_input =
                    msg.commands.data[i].command_value;
                rotor_state_.last_elevation_input_time_usec = micros64();
            }
        }
    }

    // We update the motor immediately
    updateMotor();
}

void
SwashplatelessController::handleDNAAllocation(CanardInstance *ins,
                                              CanardRxTransfer *transfer) {
    if (canardGetLocalNodeID(&canard_) != CANARD_BROADCAST_NODE_ID) {
        // We already have a node ID, so we ignore DNA allocation messages
        return;
    }

    // Delay our next allocation request according to the spec
    dna_state_.send_next_node_id_allocation_request_at_ms =
        millis32() +
        UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        (rand() %
         UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    if (transfer->source_node_id == CANARD_BROADCAST_NODE_ID) {
        // Allocation request from another allocatee. Reset offset
        dna_state_.node_id_allocation_unique_id_offset = 0;
        return;
    }

    struct uavcan_protocol_dynamic_node_id_Allocation msg;
    if (uavcan_protocol_dynamic_node_id_Allocation_decode(transfer, &msg) > 0) {
        // Something went wrong decoding the message
        return;
    }

    uint8_t my_unique_id[16];
    getUniqueID(my_unique_id);
    if (memcmp(msg.unique_id.data, my_unique_id, msg.unique_id.len) != 0) {
        // Not for us
        dna_state_.node_id_allocation_unique_id_offset = 0;
        return;
    }

    if (msg.unique_id.len < sizeof(msg.unique_id.data)) {
        // We have a partial match, wait for more
        dna_state_.node_id_allocation_unique_id_offset = msg.unique_id.len;
        dna_state_.send_next_node_id_allocation_request_at_ms -=
            UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS;
        return;
    } else {
        // Allocation complete
        canardSetLocalNodeID(ins, msg.node_id);
    }
}

void
SwashplatelessController::requestDNA() {
    const uint32_t now = millis32();
    static uint8_t node_id_allocation_transfer_id = 0;

    dna_state_.send_next_node_id_allocation_request_at_ms =
        now + UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MIN_REQUEST_PERIOD_MS +
        (rand() %
         UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_MAX_FOLLOWUP_DELAY_MS);

    uint8_t allocation_request[CANARD_CAN_FRAME_MAX_DATA_LEN - 1];
    allocation_request[0] = cfg.config.dc_node_id << 1U;

    if (dna_state_.node_id_allocation_unique_id_offset == 0) {
        allocation_request[0] |= 1;   // Frist part of unique ID
    }

    uint8_t my_unique_id[16];
    getUniqueID(my_unique_id);

    static const uint8_t MaxLenOfUniqueIDInRequest = 6;
    uint8_t uid_size =
        (uint8_t) (sizeof(my_unique_id) -
                   dna_state_.node_id_allocation_unique_id_offset);

    if (uid_size > MaxLenOfUniqueIDInRequest) {
        uid_size = MaxLenOfUniqueIDInRequest;
    }

    memmove(&allocation_request[1],
            &my_unique_id[dna_state_.node_id_allocation_unique_id_offset],
            uid_size);

    canardBroadcast(
        &canard_, UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_SIGNATURE,
        UAVCAN_PROTOCOL_DYNAMIC_NODE_ID_ALLOCATION_ID,
        &node_id_allocation_transfer_id, CANARD_TRANSFER_PRIORITY_LOW,
        allocation_request, uid_size + 1);

    dna_state_.node_id_allocation_unique_id_offset = 0;
}

void
SwashplatelessController::getUniqueID(uint8_t id[16]) {
    // STM32G4 series unique ID base (96 bits)
    const uint32_t *unique_id_base = (const uint32_t *) 0x1FFF7590;
    // Copy the 12-byte (96 bit) unique ID
    memcpy(id, unique_id_base, 12);
    // Pad the remaining 4 bytes with a known value, 0xAB
    memset(id + 12, 0xAB, 4);
}

bool
SwashplatelessController::isInputTimeout() {
    return false;
    auto now = micros64();
    if (cfg.config.esc_cmd_timeout_us) {
        if (now - rotor_state_.last_throttle_input_time_usec >
            cfg.config.esc_cmd_timeout_us) {
            return true;
        }
    }
    if (cfg.config.act_cmd_timeout_us) {
        if (now - rotor_state_.last_azimuth_input_time_usec >
            cfg.config.act_cmd_timeout_us) {
            return true;
        }
        if (now - rotor_state_.last_elevation_input_time_usec >
            cfg.config.act_cmd_timeout_us) {
            return true;
        }
    }
    return false;
}

void
SwashplatelessController::updateMotor() {
    // MODE TRANSITIONS
    // Disarm on input timeout
    if (isInputTimeout()) {
        rotor_state_.mode = RotorMode::DISARMED;
    }
    switch (rotor_state_.mode) {
    case RotorMode::DISARMED: {
        // If we have valid input and zero throttle, we go to the idle mode
        if (!isInputTimeout() && rotor_state_.throttle_input == 0.0f) {
            rotor_state_.mode = RotorMode::IDLE;
        }
        break;
    }
    case RotorMode::IDLE: {
        // If we get a nonzero throttle input, we start the motor
        if (rotor_state_.throttle_input > 0.0f) {
            rotor_state_.mode = RotorMode::SPINUP;
            rotor_state_.start_time_usec = micros64();
        }
        break;
    }
    case RotorMode::SPINUP: {
        // If we have reached spinup speed, we go to the running mode
        const float rpm =
            controller_->bldc_servo()->status().velocity_filt * 60;
        if (rpm > cfg.config.mot_start_rpm * static_cast<float>(cfg.config.mot_dir)) {
            rotor_state_.mode = RotorMode::RUNNING;
        }
        __attribute__ ((fallthrough));
    }
    case RotorMode::RUNNING: {
        // If the debug timeout is enabled and expired, we disarm
        if (cfg.config.dbg_disarm_after_us > 0) {
            if (micros64() - rotor_state_.start_time_usec >
                cfg.config.dbg_disarm_after_us) {
                rotor_state_.mode = RotorMode::IDLE;
            }
        }
        // If we have zero throttle, we go to the idle mode
        if (rotor_state_.throttle_input == 0.0f) {
            rotor_state_.mode = RotorMode::IDLE;
        }
        break;
    }
    }

    // SEND MOTOR COMMAND
    if (rotor_state_.mode == RotorMode::DISARMED || rotor_state_.mode == RotorMode::IDLE){
        moteus::BldcServoCommandData bldc_cmd;
        bldc_cmd.mode = moteus::kStopped;
        controller_->bldc_servo()->Command(bldc_cmd);
        return;
    }

    if (rotor_state_.mode == RotorMode::SPINUP || rotor_state_.mode == RotorMode::RUNNING) {
        moteus::BldcServoCommandData bldc_cmd;
        bldc_cmd.mode = moteus::kSinusoidal;
        bldc_cmd.position = std::numeric_limits<float>::quiet_NaN();
        bldc_cmd.accel_limit = cfg.config.mot_max_acc / 60.0f; // rpm/s to rps/s

        if (cfg.config.cal_mode == 0){
            // Raw input mode
            const float desired_azimuth_rad = rotor_state_.azimuth_input * pi; // Maps from [-1, 1] to [-pi, pi]
            bldc_cmd.sinusoidal_amplitude = rotor_state_.elevation_input; // We use elevation input as raw amplitude
            bldc_cmd.sinusoidal_phase = desired_azimuth_rad;
            bldc_cmd.velocity = rotor_state_.throttle_input * 100.f * (float)cfg.config.mot_dir; // Maps from [0, 1] to [0, 100] rps
        }else if(cfg.config.cal_mode == 1){
            // Angle input mode with simple calibration
            const float max_elev_rad = cfg.config.mot_max_elev_deg * pi / 180.0f;
            const float desired_elevation_rad = rotor_state_.elevation_input * max_elev_rad; // Maps from [0, 1] to [0, max_elev_rad]
            const float desired_azimuth_rad = rotor_state_.azimuth_input * pi; // Maps from [-1, 1] to [-pi, pi]

            // We use a simple calibration model where we assume that the elevation is proportional
            // to the modulation amplitude, and that the azimuth equals modulation phase plus a constant offset.
            bldc_cmd.sinusoidal_amplitude = desired_elevation_rad * cfg.config.cal_elv_gain;
            const float cal_azm_offset_rad = cfg.config.cal_azm_offset_deg * pi / 180.0f;
            bldc_cmd.sinusoidal_phase = desired_azimuth_rad + cal_azm_offset_rad;
            // Maps from [0, 1] to [0, max_rpm] proportional to the square root of the throttle command
            // This is to provide a linear relationship between throttle and thrust
            bldc_cmd.velocity = sqrtf(rotor_state_.throttle_input) * (cfg.config.mot_max_rpm / 60.f) * (float)cfg.config.mot_dir; 
        } else {
            // Invalid calibration mode
            rotor_state_.mode = RotorMode::DISARMED;
        }

        // If in spinup mode, disable modulation and limit acceleration
        if (rotor_state_.mode == RotorMode::SPINUP){
            bldc_cmd.sinusoidal_amplitude = 0;
            bldc_cmd.accel_limit = cfg.config.mot_start_acc / 60.0f; // rpm/s to rps/s
        }
        controller_->bldc_servo()->Command(bldc_cmd);
        return;
    }


}

uint64_t
SwashplatelessController::micros64() {
    // TODO: Better way to get microsecond time
    return microsecond_counter_;
}

uint32_t
SwashplatelessController::millis32() {
    return (microsecond_counter_ / 1000ULL);
}
};   // namespace DroneCAN