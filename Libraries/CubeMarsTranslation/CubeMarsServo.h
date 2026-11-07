#ifndef CUBEMARS_SERVO_H
#define CUBEMARS_SERVO_H

#include <stdbool.h>
#include <stdint.h>

/* AK60-39 V3.0 nominal motor properties (manufacturer specifications). */
#define AK60_39_POLE_PAIRS 14u
#define AK60_39_GEAR_RATIO 39.0f

/* AK V3 manual Section 2.1.4: protocol encoding limit, NOT a safe joint limit. */
#define CUBEMARS_SERVO_MAX_PROTOCOL_ERPM 100000

/* Hardware-independent representation of a Classical CAN frame. */
typedef struct {
    uint32_t id;
    uint8_t data[8];
    uint8_t length;
    bool extended;
} CubeMarsFrame;

/* All unresolved rover settings remain explicit and configurable. */
typedef struct {
    uint8_t controller_id;            /* TEST value 1; confirm physical controller ID */
    uint8_t pole_pairs;               /* 14 for AK60-39 V3.0 */
    float gear_ratio;                 /* 39 motor turns per output-shaft turn */
    bool input_is_joint_speed;        /* false: CAN26 rev/s at motor shaft;
                                        true: CAN26 rev/s at gearbox output */
    int32_t max_abs_erpm;             /* TEST limit, not approved motor safety limit */
} CubeMarsConfig;

/* Encode an already converted signed ERPM into the CubeMars Servo speed frame.
   Returns false for out-of-range ERPM; leaves output unchanged on failure. */
bool CubeMarsMakeServoVelocityFrame(uint8_t controller_id,
                                    int32_t velocity_erpm,
                                    CubeMarsFrame *out_frame);

/* Take values ALREADY DECODED from CAN26 and return a CubeMars Servo frame.
   No CAN messages are transmitted here.
   A nonzero CAN26 feedforward torque is not representable in Servo speed mode:
   reject the request instead of silently discarding it. */
bool CubeMarsTranslateServoVelocity(float velocity_rev_s,
                                    float feedforward_torque_nm,
                                    const CubeMarsConfig *config,
                                    CubeMarsFrame *out_frame);

#endif /* CUBEMARS_SERVO_H */
