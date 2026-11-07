#include "CubeMarsFeedback.h"

#include <math.h>
#include <stddef.h>

#define CUBEMARS_SERVO_STATUS_FUNCTION 0x29u

/* Convert two network-order bytes to signed 16 bits without depending on
 * implementation-defined unsigned-to-signed conversion. */
static int16_t load_int16_be(const uint8_t *data) {
    uint32_t raw = ((uint32_t)data[0] << 8) | (uint32_t)data[1];
    int32_t signed_value = (raw >= 0x8000u) ? (int32_t)raw - 65536 : (int32_t)raw;
    return (int16_t)signed_value;
}

static int8_t load_int8(uint8_t raw) {
    int16_t signed_value = (raw >= 0x80u) ? (int16_t)raw - 256 : (int16_t)raw;
    return (int8_t)signed_value;
}

bool CubeMarsDecodeServoFeedback(const CubeMarsFrame *frame,
                                 uint8_t expected_controller_id,
                                 CubeMarsServoFeedback *out_feedback) {
    if (frame == NULL || out_feedback == NULL || !frame->extended ||
        frame->length != 8u ||
        frame->id != ((CUBEMARS_SERVO_STATUS_FUNCTION << 8) |
                      (uint32_t)expected_controller_id)) {
        return false;
    }

    CubeMarsServoFeedback decoded = {
        .position_degrees = (float)load_int16_be(frame->data + 0) * 0.1f,
        .speed_erpm = (float)load_int16_be(frame->data + 2) * 10.0f,
        .iq_amps = (float)load_int16_be(frame->data + 4) * 0.01f,
        .temperature_c = load_int8(frame->data[6]),
        .status_code = frame->data[7]
    };
    *out_feedback = decoded;
    return true;
}

bool CubeMarsFeedbackToMechanical(const CubeMarsServoFeedback *feedback,
                                   const CubeMarsConfig *config,
                                   bool position_is_motor_shaft,
                                   float *out_position_rev,
                                   float *out_velocity_rev_s) {
    if (feedback == NULL || config == NULL || out_position_rev == NULL ||
        out_velocity_rev_s == NULL || config->pole_pairs == 0u ||
        !isfinite(config->gear_ratio) || config->gear_ratio <= 0.0f ||
        !isfinite(feedback->position_degrees) ||
        !isfinite(feedback->speed_erpm) || feedback->status_code != 0u) {
        return false;
    }

    double position_rev = (double)feedback->position_degrees / 360.0;
    double velocity_rev_s = (double)feedback->speed_erpm /
                            (60.0 * (double)config->pole_pairs);

    /* The 0x29 velocity is electrical motor speed; adapt it to the requested
     * CAN26 reference. Position's reference is not confirmed by the manual. */
    if (config->input_is_joint_speed) {
        velocity_rev_s /= (double)config->gear_ratio;
        if (position_is_motor_shaft) {
            position_rev /= (double)config->gear_ratio;
        }
    } else if (!position_is_motor_shaft) {
        position_rev *= (double)config->gear_ratio;
    }

    if (!isfinite(position_rev) || !isfinite(velocity_rev_s) ||
        fabs(position_rev) > 3.4e38 || fabs(velocity_rev_s) > 3.4e38) {
        return false;
    }
    *out_position_rev = (float)position_rev;
    *out_velocity_rev_s = (float)velocity_rev_s;
    return true;
}
