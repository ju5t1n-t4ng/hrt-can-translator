#ifndef CAN26_CUBEMARS_FEEDBACK_H
#define CAN26_CUBEMARS_FEEDBACK_H

#include <stdbool.h>
#include "CANPacket.h"
#include "CubeMarsFeedback.h"

/* Convert one healthy CubeMars servo 0x29 frame to a CAN26 encoder estimate.
 * `destination` is the CAN26 receiver (usually the Jetson).
 * No real CAN messages are transmitted. */
bool CubeMarsToCAN26EncoderEstimates(const CubeMarsFrame *motor_frame,
                                      const CubeMarsConfig *config,
                                      bool position_is_motor_shaft,
                                      CANDevice_t destination,
                                      CANPacket_t *out_packet);

/* Respond to a CAN26 GetEncoderEstimates request using the most recently
 * received 0x29 frame. The application MUST independently establish that the
 * feedback is fresh; pass false when it is missing or stale. Supports only
 * encoder index 0. This module does NOT poll the motor or maintain timestamps.
 */
bool CAN26ReplyToEncoderEstimateRequest(const CANPacket_t *request,
                                          const CubeMarsFrame *latest_motor_frame,
                                          bool feedback_is_fresh,
                                          const CubeMarsConfig *config,
                                          bool position_is_motor_shaft,
                                          CANPacket_t *out_packet);

#endif
