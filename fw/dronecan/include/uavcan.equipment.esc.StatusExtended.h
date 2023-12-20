#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <canard.h>


#define UAVCAN_EQUIPMENT_ESC_STATUSEXTENDED_MAX_SIZE 7
#define UAVCAN_EQUIPMENT_ESC_STATUSEXTENDED_SIGNATURE (0x4E6F30399D3E2692ULL)
#define UAVCAN_EQUIPMENT_ESC_STATUSEXTENDED_ID 1036

#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
class uavcan_equipment_esc_StatusExtended_cxx_iface;
#endif

struct uavcan_equipment_esc_StatusExtended {
#if defined(__cplusplus) && defined(DRONECAN_CXX_WRAPPERS)
    using cxx_iface = uavcan_equipment_esc_StatusExtended_cxx_iface;
#endif
    uint8_t input_pct;
    uint8_t output_pct;
    int16_t motor_temperature_degC;
    uint16_t motor_angle;
    uint32_t status_flags;
};

#ifdef __cplusplus
extern "C"
{
#endif

uint32_t uavcan_equipment_esc_StatusExtended_encode(struct uavcan_equipment_esc_StatusExtended* msg, uint8_t* buffer
#if CANARD_ENABLE_TAO_OPTION
    , bool tao
#endif
);
bool uavcan_equipment_esc_StatusExtended_decode(const CanardRxTransfer* transfer, struct uavcan_equipment_esc_StatusExtended* msg);

#if defined(CANARD_DSDLC_INTERNAL)
static inline void _uavcan_equipment_esc_StatusExtended_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_esc_StatusExtended* msg, bool tao);
static inline void _uavcan_equipment_esc_StatusExtended_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_esc_StatusExtended* msg, bool tao);
void _uavcan_equipment_esc_StatusExtended_encode(uint8_t* buffer, uint32_t* bit_ofs, struct uavcan_equipment_esc_StatusExtended* msg, bool tao) {
    (void)buffer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardEncodeScalar(buffer, *bit_ofs, 7, &msg->input_pct);
    *bit_ofs += 7;
    canardEncodeScalar(buffer, *bit_ofs, 7, &msg->output_pct);
    *bit_ofs += 7;
    canardEncodeScalar(buffer, *bit_ofs, 9, &msg->motor_temperature_degC);
    *bit_ofs += 9;
    canardEncodeScalar(buffer, *bit_ofs, 9, &msg->motor_angle);
    *bit_ofs += 9;
    canardEncodeScalar(buffer, *bit_ofs, 24, &msg->status_flags);
    *bit_ofs += 24;
}

void _uavcan_equipment_esc_StatusExtended_decode(const CanardRxTransfer* transfer, uint32_t* bit_ofs, struct uavcan_equipment_esc_StatusExtended* msg, bool tao) {
    (void)transfer;
    (void)bit_ofs;
    (void)msg;
    (void)tao;

    canardDecodeScalar(transfer, *bit_ofs, 7, false, &msg->input_pct);
    *bit_ofs += 7;

    canardDecodeScalar(transfer, *bit_ofs, 7, false, &msg->output_pct);
    *bit_ofs += 7;

    canardDecodeScalar(transfer, *bit_ofs, 9, true, &msg->motor_temperature_degC);
    *bit_ofs += 9;

    canardDecodeScalar(transfer, *bit_ofs, 9, false, &msg->motor_angle);
    *bit_ofs += 9;

    canardDecodeScalar(transfer, *bit_ofs, 24, false, &msg->status_flags);
    *bit_ofs += 24;

}
#endif
#ifdef CANARD_DSDLC_TEST_BUILD
struct uavcan_equipment_esc_StatusExtended sample_uavcan_equipment_esc_StatusExtended_msg(void);
#endif
#ifdef __cplusplus
} // extern "C"

#ifdef DRONECAN_CXX_WRAPPERS
#include <canard/cxx_wrappers.h>
BROADCAST_MESSAGE_CXX_IFACE(uavcan_equipment_esc_StatusExtended, UAVCAN_EQUIPMENT_ESC_STATUSEXTENDED_ID, UAVCAN_EQUIPMENT_ESC_STATUSEXTENDED_SIGNATURE, UAVCAN_EQUIPMENT_ESC_STATUSEXTENDED_MAX_SIZE);
#endif
#endif
