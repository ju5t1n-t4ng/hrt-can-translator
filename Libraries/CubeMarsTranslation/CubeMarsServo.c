#include "CubeMarsServo.h"

#include <math.h>
#include <stddef.h>

#define CUBEMARS_SERVO_SPEED_MODE 3u

/* File-private helper: most-significant byte first (big-endian). */
static void store_int32_be(uint8_t data[4], int32_t value) {
    uint32_t raw = (uint32_t)value;
    data[0] = (uint8_t)((raw >> 24) & 0xFFu);
    data[1] = (uint8_t)((raw >> 16) & 0xFFu);
    data[2] = (uint8_t)((raw >>  8) & 0xFFu);
    data[3] = (uint8_t)( raw        & 0xFFu);
}

bool CubeMarsMakeServoVelocityFrame(uint8_t controller_id,
                                    int32_t velocity_erpm,
                                    CubeMarsFrame *out_frame) {
    if (out_frame == NULL ||
        velocity_erpm < -CUBEMARS_SERVO_MAX_PROTOCOL_ERPM ||
        velocity_erpm > CUBEMARS_SERVO_MAX_PROTOCOL_ERPM) {
        return false;
    }

    CubeMarsFrame frame = {0};
    /* 29-bit extended CAN ID: mode in bits 8+, controller ID in bits 0-7. */
    frame.id = (CUBEMARS_SERVO_SPEED_MODE << 8) | controller_id;
    frame.length = 4;
    frame.extended = true;
    store_int32_be(frame.data, velocity_erpm);

    *out_frame = frame;
    return true;
}

bool CubeMarsTranslateServoVelocity(float velocity_rev_s,
                                    float feedforward_torque_nm,
                                    const CubeMarsConfig *config,
                                    CubeMarsFrame *out_frame) {
    if (config == NULL || out_frame == NULL ||
        config->pole_pairs == 0u ||
        !isfinite(config->gear_ratio) || config->gear_ratio <= 0.0f ||
        config->max_abs_erpm <= 0 ||
        config->max_abs_erpm > CUBEMARS_SERVO_MAX_PROTOCOL_ERPM ||
        !isfinite(velocity_rev_s) ||
        !isfinite(feedforward_torque_nm) ||
        feedforward_torque_nm != 0.0f) {
        return false;
    }

    /* Rev/s is a CAN26 property; gearbox-output vs motor-shaft is selectable. */
    double motor_rev_s = (double)velocity_rev_s;
    if (config->input_is_joint_speed) {
        motor_rev_s *= (double)config->gear_ratio;
    }
    double erpm = motor_rev_s * 60.0 * (double)config->pole_pairs;

    if (!isfinite(erpm) ||
        erpm < -(double)config->max_abs_erpm ||
        erpm > (double)config->max_abs_erpm) {
        return false;
    }

    /* Round to nearest ERPM count, instead of silently truncating. */
    int32_t whole_erpm = (int32_t)lround(erpm);
    return CubeMarsMakeServoVelocityFrame(config->controller_id,
                                          whole_erpm, out_frame);
}
