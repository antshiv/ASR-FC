#include "asr_fc/platform/gd3000_protocol.h"

#include <stddef.h>

#define GD3000_STATUS0_OVERTEMPERATURE (1u << 0)
#define GD3000_STATUS0_DESATURATION (1u << 1)
#define GD3000_STATUS0_LOW_VLS (1u << 2)
#define GD3000_STATUS0_OVERCURRENT (1u << 3)
#define GD3000_STATUS0_PHASE_ERROR (1u << 4)
#define GD3000_STATUS0_FRAMING_ERROR (1u << 5)
#define GD3000_STATUS0_WRITE_ERROR (1u << 6)
#define GD3000_STATUS0_RESET_EVENT (1u << 7)

asr_fc_gd3000_status_t asr_fc_gd3000_null_command(
    uint8_t status_register,
    uint8_t *command) {
    if (command == NULL) {
        return ASR_FC_GD3000_NULL_POINTER;
    }
    if (status_register > 3u) {
        return ASR_FC_GD3000_INVALID_STATUS_REGISTER;
    }

    *command = status_register;
    return ASR_FC_GD3000_OK;
}

asr_fc_gd3000_status_t asr_fc_gd3000_status_read_sequence(
    uint8_t status_register,
    asr_fc_gd3000_status_read_t *sequence) {
    if (sequence == NULL) {
        return ASR_FC_GD3000_NULL_POINTER;
    }

    asr_fc_gd3000_status_t result = asr_fc_gd3000_null_command(
        status_register, &sequence->request);
    if (result != ASR_FC_GD3000_OK) {
        return result;
    }

    /* NXP AN5169 clocks every requested status out with a NULL0 transfer. */
    sequence->flush = 0u;
    return ASR_FC_GD3000_OK;
}

asr_fc_gd3000_status_t asr_fc_gd3000_decode_status0(
    uint8_t value,
    asr_fc_gd3000_status0_t *status) {
    if (status == NULL) {
        return ASR_FC_GD3000_NULL_POINTER;
    }

    status->overtemperature =
        (value & GD3000_STATUS0_OVERTEMPERATURE) != 0u;
    status->desaturation = (value & GD3000_STATUS0_DESATURATION) != 0u;
    status->low_vls = (value & GD3000_STATUS0_LOW_VLS) != 0u;
    status->overcurrent = (value & GD3000_STATUS0_OVERCURRENT) != 0u;
    status->phase_error = (value & GD3000_STATUS0_PHASE_ERROR) != 0u;
    status->framing_error = (value & GD3000_STATUS0_FRAMING_ERROR) != 0u;
    status->write_error = (value & GD3000_STATUS0_WRITE_ERROR) != 0u;
    status->reset_event = (value & GD3000_STATUS0_RESET_EVENT) != 0u;
    return ASR_FC_GD3000_OK;
}

bool asr_fc_gd3000_status0_has_fault(
    const asr_fc_gd3000_status0_t *status) {
    if (status == NULL) {
        return true;
    }

    return status->overtemperature || status->desaturation ||
           status->low_vls || status->overcurrent || status->phase_error ||
           status->framing_error || status->write_error;
}
