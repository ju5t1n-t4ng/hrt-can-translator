#ifndef CUBEMARS_FEEDBACK_H
#define CUBEMARS_FEEDBACK_H

#include "CubeMarsServo.h"

#include <stdbool.h>
#include <stdint.h>

/* AK V3 servo telemetry, function 0x29, exactly 8 data bytes.
 * The position field is in degrees, but the manual does not unambiguously
 * establish whether it represents the motor shaft or gearbox output.
 * Preserve it in degrees and choose a physical reference when converting.
 */
typedef struct {
    float position_degrees;    /* signed int16 * 0.1 degree */
    float speed_erpm;          /* signed int16 * 10 electrical RPM */
    float iq_amps;             /* signed int16 * 0.01 A */
    int8_t temperature_c;      /* signed int8 C */
    uint8_t status_code;       /* 0=normal, 1..7=fault, 0x77=disable response */
} CubeMarsServoFeedback;

/* Decode a real CubeMars 0x29 status message. No HAL or CAN26 dependencies.
 * Returns false for an unexpected identifier, format or payload length.
 * Fault and disable frames are decoded too, so the caller can inspect them.
 * The output remains unchanged on failure.
 */
bool CubeMarsDecodeServoFeedback(const CubeMarsFrame *frame,
                                 uint8_t expected_controller_id,
                                 CubeMarsServoFeedback *out_feedback);

/* Convert decoded motor status into CAN26-style rev and rev/s.
 * config->input_is_joint_speed selects CAN26's mechanical reference:
 *   true = gearbox output; false = motor shaft.
 * position_is_motor_shaft is a separate, UNCONFIRMED hardware assumption.
 * Only healthy feedback (status_code==0) is accepted for encoder estimates.
 */
bool CubeMarsFeedbackToMechanical(const CubeMarsServoFeedback *feedback,
                                   const CubeMarsConfig *config,
                                   bool position_is_motor_shaft,
                                   float *out_position_rev,
                                   float *out_velocity_rev_s);

#endif
