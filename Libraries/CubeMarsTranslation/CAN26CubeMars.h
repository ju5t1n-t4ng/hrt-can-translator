#ifndef CAN26_CUBEMARS_H
#define CAN26_CUBEMARS_H

#include <stdbool.h>
#include "CANPacket.h"           
#include "CubeMarsServo.h"

/* Full software path: already-parsed CAN26 packet -> CubeMars Servo frame.
   Only accepts BLDC_INPUT_VELOCITY addressed to the elbow. */
bool CAN26ToCubeMarsServoVelocity(const CANPacket_t *incoming,
                                  const CubeMarsConfig *config,
                                  CubeMarsFrame *out_frame);

#endif 
