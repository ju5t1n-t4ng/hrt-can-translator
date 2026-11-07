#include "CAN26CubeMarsFeedback.h"

#include "CANCommandIDs.h"
#include "CANDevices.h"
#include "Packets/Motor.h"

#include <stddef.h>

bool CubeMarsToCAN26EncoderEstimates(const CubeMarsFrame *motor_frame,
                                      const CubeMarsConfig *config,
                                      bool position_is_motor_shaft,
                                      CANDevice_t destination,
                                      CANPacket_t *out_packet) {
    if (motor_frame == NULL || config == NULL || out_packet == NULL ||
        destination.deviceUUID == CAN_UUID_BROADCAST) {
        return false;
    }

    CubeMarsServoFeedback feedback = {0};
    if (!CubeMarsDecodeServoFeedback(motor_frame, config->controller_id,
                                      &feedback)) {
        return false;
    }

    float position_rev = 0.0f;
    float velocity_rev_s = 0.0f;
    if (!CubeMarsFeedbackToMechanical(&feedback, config,
                                      position_is_motor_shaft,
                                      &position_rev, &velocity_rev_s)) {
        return false;
    }

    CANDevice_t elbow = {0};
    elbow.deviceUUID = CAN_UUID_BLDC_ELBOW;
    elbow.motorDomain = true;

    CANPacket_t encoded = CANMotorPacket_BLDC_EncoderEstimates(
        elbow, destination, position_rev, velocity_rev_s);
    *out_packet = encoded;
    return true;
}

bool CAN26ReplyToEncoderEstimateRequest(const CANPacket_t *request,
                                          const CubeMarsFrame *latest_motor_frame,
                                          bool feedback_is_fresh,
                                          const CubeMarsConfig *config,
                                          bool position_is_motor_shaft,
                                          CANPacket_t *out_packet) {
    if (request == NULL || latest_motor_frame == NULL ||
        config == NULL || out_packet == NULL || !feedback_is_fresh ||
        request->device.deviceUUID != CAN_UUID_BLDC_ELBOW ||
        (request->command & 0x7Fu) !=
            CAN_COMMAND_ID__BLDC_ENCODER_ESTIMATE_GET ||
        request->contentsLength != 1u || request->contents[0] != 0u ||
        request->senderUUID == CAN_UUID_BROADCAST) {
        return false;
    }

    CANDevice_t destination = {0};
    destination.deviceUUID = request->senderUUID;
    return CubeMarsToCAN26EncoderEstimates(latest_motor_frame, config,
                                            position_is_motor_shaft,
                                            destination, out_packet);
}
