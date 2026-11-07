#include "CAN26CubeMars.h"

#include "CANCommandIDs.h"
#include "CANDevices.h"
#include "Packets/DecodeMotor.h"

#include <stddef.h>

bool CAN26ToCubeMarsServoVelocity(const CANPacket_t *incoming,
                                  const CubeMarsConfig *config,
                                  CubeMarsFrame *out_frame) {
    if (incoming == NULL || config == NULL || out_frame == NULL) {
        return false;
    }

    /* Verifying before calling  CAN26 decoder (which expects six data bytes). */
    if (!incoming->device.motorDomain ||
        incoming->device.deviceUUID != CAN_UUID_BLDC_ELBOW ||
        (incoming->command & 0x7Fu) != CAN_COMMAND_ID__BLDC_INPUT_VELOCITY ||
        incoming->contentsLength != 6u) {
        return false;
    }

    CANMotorPacket_BLDC_SetInputVelocity_Decoded_t decoded =
        CANMotorPacket_BLDC_SetInputVelocity_Decode(incoming);

    return CubeMarsTranslateServoVelocity(decoded.velocity,
                                          decoded.feedForwardTorque,
                                          config,
                                          out_frame);
}
