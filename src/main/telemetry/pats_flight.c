/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 */

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "platform.h"

#include "common/crc.h"
#include "common/axis.h"
#include "common/maths.h"

#ifdef USE_DSHOT_TELEMETRY
#include "drivers/dshot.h"
#include "drivers/motor.h"
#endif

#include "fc/runtime_config.h"
#include "flight/imu.h"
#include "sensors/acceleration.h"

#include "telemetry/pats_flight.h"

// #define PATS_FLIGHT_MOTOR_SPINUP_eRPM   (300)

static const float kQuatLimit = 0.7071067811865475244f;
// static const float spinupExpAlpha = 0.00498752080731768f;
// static uint16_t motorRPMavg[4] = {0,0,0,0};

#ifdef USE_DSHOT_TELEMETRY

static bool patsFlightMotorsHaveSpunUp(void)
{
    if (!useDshotTelemetry) {
        return false;
    }

    const unsigned motorCount = motorDeviceCount();
    bool allMotorsSpunUp = true;
    for (unsigned motor = 0; motor < motorCount; motor++) {
        // uint16_t erpm = getDshotTelemetry(motor);
        // motorRPMavg[motor] += (uint16_t)(spinupExpAlpha * (float)(erpm - motorRPMavg[motor]));
        // if (motorRPMavg[motor] <= PATS_FLIGHT_MOTOR_SPINUP_eRPM) {
        //     allMotorsSpunUp = false;
        // }
        if (getDshotMotorAverageRpm(motor) < PATS_FLIGHT_MOTOR_SPINUP_eRPM) {
            allMotorsSpunUp = false;
        }
    }
    return allMotorsSpunUp;
}
#endif

uint8_t patsArmingFailureReasonFromFlags(armingDisableFlags_e flags)
{
    if (!flags) {
        return PATS_ARMING_FAILURE_NONE;
    }
    if (flags & ARMING_DISABLED_NO_GYRO) {
        return PATS_ARMING_FAILURE_NO_GYRO;
    }
    if (flags & (ARMING_DISABLED_FAILSAFE | ARMING_DISABLED_RX_FAILSAFE | ARMING_DISABLED_BOXFAILSAFE)) {
        return PATS_ARMING_FAILURE_FAILSAFE;
    }
    if (flags & ARMING_DISABLED_BAD_RX_RECOVERY) {
        return PATS_ARMING_FAILURE_RX;
    }
    if (flags & ARMING_DISABLED_THROTTLE) {
        return PATS_ARMING_FAILURE_THROTTLE;
    }
    if (flags & ARMING_DISABLED_ANGLE) {
        return PATS_ARMING_FAILURE_ANGLE;
    }
    if (flags & (ARMING_DISABLED_CLI | ARMING_DISABLED_CMS_MENU | ARMING_DISABLED_MSP | ARMING_DISABLED_REBOOT_REQUIRED)) {
        return PATS_ARMING_FAILURE_CONFIG;
    }
    if (flags & (ARMING_DISABLED_BOOT_GRACE_TIME | ARMING_DISABLED_NOPREARM | ARMING_DISABLED_LOAD | ARMING_DISABLED_CALIBRATING
        | ARMING_DISABLED_GPS | ARMING_DISABLED_RESC | ARMING_DISABLED_RUNAWAY_TAKEOFF | ARMING_DISABLED_CRASH_DETECTED
        | ARMING_DISABLED_RPMFILTER | ARMING_DISABLED_DSHOT_BITBANG | ARMING_DISABLED_MOTOR_PROTOCOL | ARMING_DISABLED_BST
        | ARMING_DISABLED_PARALYZE | ARMING_DISABLED_ACC_CALIBRATION | ARMING_DISABLED_ARM_SWITCH)) {
        return PATS_ARMING_FAILURE_SYSTEM;
    }

    return PATS_ARMING_FAILURE_SYSTEM;
}

uint8_t patsFlightEncodeAcceleration6(float accelerationG, bool valid)
{
    if (!valid || !isfinite(accelerationG)) {
        return 0;
    }
    if (accelerationG < -7.50f) {
        return 1;
    }
    if (accelerationG > 7.50f) {
        return 63;
    }

    return (uint8_t)constrain((int)lroundf(accelerationG / 0.25f) + 32, 2, 62);
}

uint8_t patsFlightEncodeAcceleration5(float accelerationG, bool valid)
{
    if (!valid || !isfinite(accelerationG)) {
        return 0;
    }
    if (accelerationG < -3.50f) {
        return 1;
    }
    if (accelerationG > 3.50f) {
        return 31;
    }

    return (uint8_t)constrain((int)lroundf(accelerationG / 0.25f) + 16, 2, 30);
}

static patsDecodedAcceleration_t decodeAcceleration(uint8_t code, uint8_t negativeOverflowCode, uint8_t positiveOverflowCode, uint8_t zeroCode)
{
    patsDecodedAcceleration_t decoded = { .valueG = 0.0f, .status = PATS_ACCELERATION_INVALID };
    if (code == 0) {
        return decoded;
    }
    if (code == negativeOverflowCode) {
        decoded.valueG = ((int)negativeOverflowCode + 1 - zeroCode) * 0.25f;
        decoded.status = PATS_ACCELERATION_NEGATIVE_OVERFLOW;
        return decoded;
    }
    if (code == positiveOverflowCode) {
        decoded.valueG = ((int)positiveOverflowCode - 1 - zeroCode) * 0.25f;
        decoded.status = PATS_ACCELERATION_POSITIVE_OVERFLOW;
        return decoded;
    }
    decoded.valueG = ((int)code - zeroCode) * 0.25f;
    decoded.status = PATS_ACCELERATION_NUMERIC;
    return decoded;
}

patsDecodedAcceleration_t patsFlightDecodeAcceleration6(uint8_t code)
{
    return decodeAcceleration(code & 0x3F, 1, 63, 32);
}

patsDecodedAcceleration_t patsFlightDecodeAcceleration5(uint8_t code)
{
    return decodeAcceleration(code & 0x1F, 1, 31, 16);
}

static void normalizeQuaternion(float q[4])
{
    const float normSquared = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if (!isfinite(normSquared) || normSquared <= 0.0f) {
        q[0] = 1.0f;
        q[1] = q[2] = q[3] = 0.0f;
        return;
    }

    const float invNorm = 1.0f / sqrtf(normSquared);
    q[0] *= invNorm;
    q[1] *= invNorm;
    q[2] *= invNorm;
    q[3] *= invNorm;
}

static uint8_t encodeQuatComponent(float component)
{
    const float clamped = constrainf(component, -kQuatLimit, kQuatLimit);
    return (uint8_t)constrain((int)lroundf((clamped + kQuatLimit) * 255.0f / (2.0f * kQuatLimit)), 0, 255);
}

static float decodeQuatComponent(uint8_t encoded)
{
    return encoded * (2.0f * kQuatLimit / 255.0f) - kQuatLimit;
}

patsFlightPayload_t encodePatsFlightPayload(const patsFlightTelemetryState_t *state)
{
    patsFlightPayload_t payload = { 0 };
    if (!state) {
        payload.armingFailureReason = PATS_ARMING_FAILURE_SYSTEM;
        return payload;
    }
    const bool armed = state->armed;
    const bool allMotorsSpunUp = state->allMotorsSpunUp;
    const uint8_t armingFailureReason = state->armingFailureReason <= PATS_ARMING_FAILURE_SYSTEM
        ? state->armingFailureReason
        : PATS_ARMING_FAILURE_SYSTEM;
    payload.armed = armed;
    payload.armingFailureReason = armed ? PATS_ARMING_FAILURE_NONE : armingFailureReason;
    payload.allMotorsSpunUp = allMotorsSpunUp;

    float q[4];
    q[0] = state->quaternionW;
    q[1] = state->quaternionX;
    q[2] = state->quaternionY;
    q[3] = state->quaternionZ;
    normalizeQuaternion(q);

    uint8_t largestQuatIndex = 0;
    float largestAbs = fabsf(q[0]);
    for (uint8_t ii = 1; ii < 4; ++ii) {
        const float absComponent = fabsf(q[ii]);
        if (absComponent > largestAbs) {
            largestAbs = absComponent;
            largestQuatIndex = ii;
        }
    }

    if (q[largestQuatIndex] < 0.0f) {
        q[0] = -q[0];
        q[1] = -q[1];
        q[2] = -q[2];
        q[3] = -q[3];
    }

    uint8_t quatComponent[3];
    for (uint8_t ii = 0, jj = 0; ii < 4; ++ii) {
        if (ii != largestQuatIndex) {
            quatComponent[jj++] = encodeQuatComponent(q[ii]);
        }
    }

    uint8_t accelerationX = 0;
    uint8_t accelerationY = 0;
    uint8_t accelerationZ = 0;
    if (state->accelerationValid) {
        accelerationX = patsFlightEncodeAcceleration6(state->accelerationXG, true);
        accelerationY = patsFlightEncodeAcceleration6(state->accelerationYG, true);
        accelerationZ = patsFlightEncodeAcceleration5(state->accelerationZG, true);
    }

    payload.largestQuatIndex = largestQuatIndex;
    payload.quaternionComponent0 = quatComponent[0];
    payload.quaternionComponent1 = quatComponent[1];
    payload.quaternionComponent2 = quatComponent[2];
    payload.accelerationX = accelerationX;
    payload.accelerationY = accelerationY;
    payload.accelerationZ = accelerationZ;

    // for (size_t ii = 0; ii < 4; ii++) {
    //     payload.bytes[ii] = (uint8_t)(getDshotTelemetry(ii));
    //     // payload.bytes[ii] = (uint8_t)(motorRPMavg[ii]/256);
    // }

    // uint16_t erpmMotor0 = getDshotTelemetry(0); //16 bits each AAAABBBBCCCCZZZZ
    // uint16_t erpmMotor1 = getDshotTelemetry(1); //16 bits each DDDDEEEEFFFFZZZZ
    // uint16_t erpmMotor2 = getDshotTelemetry(2); //16 bits each 
    // uint16_t erpmMotor3 = getDshotTelemetry(3); //16 bits each

    uint16_t erpmMotor0 = getDshotMotorAverageRpm(0);
    uint16_t erpmMotor1 = getDshotMotorAverageRpm(1);
    uint16_t erpmMotor2 = getDshotMotorAverageRpm(2);
    uint16_t erpmMotor3 = getDshotMotorAverageRpm(3);
    
    uint8_t count_spun_up = 0;
    for (size_t ii = 0; ii < 4; ii++) {
        // if (motorRPMavg[ii] > PATS_FLIGHT_MOTOR_SPINUP_eRPM)
        if (getDshotMotorAverageRpm(ii) > PATS_FLIGHT_MOTOR_SPINUP_eRPM)
            count_spun_up++;
    }

    payload.bytes[0] = (uint8_t)(erpmMotor0 >> 8); 
    payload.bytes[1] = (uint8_t)((erpmMotor0 & 0x00E0) | ((count_spun_up & 0x04) == 0x04 ? 0x10 : 0x00) | ((erpmMotor1 & 0xF000) >> 12));
    payload.bytes[2] = (uint8_t)(((erpmMotor1 & 0x0FE0) >> 4) | (allMotorsSpunUp ? 0x01 : 0x00)); 
    payload.bytes[3] = (uint8_t)(erpmMotor2 >> 8); 
    payload.bytes[4] = (uint8_t)((erpmMotor2 & 0x00E0) | ((count_spun_up & 0x02) == 0x02 ? 0x10 : 0x00) | ((erpmMotor3 & 0xF000) >> 12));
    payload.bytes[5] = (uint8_t)(((erpmMotor3 & 0x0FE0) >> 4) | (count_spun_up & 0x01)); 

    return payload;
}

bool decodePatsFlightPayload(const uint8_t payload[PATS_FLIGHT_PAYLOAD_SIZE], decodedPatsFlightTelemetry_t *output)
{
    if (!payload || !output) {
        return false;
    }

    patsFlightPayload_t packed;
    memcpy(packed.bytes, payload, PATS_FLIGHT_PAYLOAD_SIZE);

    memset(output, 0, sizeof(*output));
    output->armed = packed.armed;
    output->armingFailureReason = packed.armingFailureReason;
    output->allMotorsSpunUp = packed.allMotorsSpunUp;
    if (output->armed) {
        output->armingFailureReason = PATS_ARMING_FAILURE_NONE;
    }
    output->largestQuatIndex = packed.largestQuatIndex;
    output->quatComponent[0] = packed.quaternionComponent0;
    output->quatComponent[1] = packed.quaternionComponent1;
    output->quatComponent[2] = packed.quaternionComponent2;
    output->accelerationXCode = packed.accelerationX;
    output->accelerationYCode = packed.accelerationY;
    output->accelerationZCode = packed.accelerationZ;

    float q[4] = { 0.0f, 0.0f, 0.0f, 0.0f };
    float stored[3] = {
        decodeQuatComponent(output->quatComponent[0]),
        decodeQuatComponent(output->quatComponent[1]),
        decodeQuatComponent(output->quatComponent[2]),
    };
    float storedSquareSum = 0.0f;
    for (uint8_t ii = 0, jj = 0; ii < 4; ++ii) {
        if (ii == output->largestQuatIndex) {
            continue;
        }
        q[ii] = stored[jj];
        storedSquareSum += stored[jj] * stored[jj];
        jj++;
    }
    q[output->largestQuatIndex] = sqrtf(MAX(0.0f, 1.0f - storedSquareSum));
    normalizeQuaternion(q);

    output->quaternionW = q[0];
    output->quaternionX = q[1];
    output->quaternionY = q[2];
    output->quaternionZ = q[3];
    output->accelerationX = patsFlightDecodeAcceleration6(output->accelerationXCode);
    output->accelerationY = patsFlightDecodeAcceleration6(output->accelerationYCode);
    output->accelerationZ = patsFlightDecodeAcceleration5(output->accelerationZCode);

    return true;
}

bool buildPatsFlightCrsfFrame(const patsFlightTelemetryState_t *state, uint8_t *destination, size_t destinationSize, size_t *frameSize)
{
    if (!destination || destinationSize < PATS_FLIGHT_CRSF_FRAME_SIZE) {
        return false;
    }

    const patsFlightPayload_t payload = encodePatsFlightPayload(state);
    destination[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    destination[1] = PATS_FLIGHT_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC;
    destination[2] = CRSF_FRAMETYPE_PATS_FLIGHT;
    memcpy(&destination[3], payload.bytes, PATS_FLIGHT_PAYLOAD_SIZE);
    destination[9] = crc8_dvb_s2_update(0, &destination[2], PATS_FLIGHT_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE);

    if (frameSize) {
        *frameSize = PATS_FLIGHT_CRSF_FRAME_SIZE;
    }
    return true;
}

bool decodePatsFlightCrsfFrame(const uint8_t *frame, size_t frameSize, decodedPatsFlightTelemetry_t *output)
{
    if (!frame || frameSize < PATS_FLIGHT_CRSF_FRAME_SIZE || !output) {
        return false;
    }
    if (frame[1] < PATS_FLIGHT_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC || frameSize < (size_t)frame[1] + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH) {
        return false;
    }
    if (frame[2] != CRSF_FRAMETYPE_PATS_FLIGHT) {
        return false;
    }

    const uint8_t crc = crc8_dvb_s2_update(0, &frame[2], frame[1] - CRSF_FRAME_LENGTH_CRC);
    if (crc != frame[1 + frame[1]]) {
        return false;
    }

    return decodePatsFlightPayload(&frame[3], output);
}

void patsFlightTelemetryStateFromFc(patsFlightTelemetryState_t *state)
{
    
    quaternion quat;
    getQuaternion(&quat);
    
    state->armed = ARMING_FLAG(ARMED);
    state->armingFailureReason = state->armed ? PATS_ARMING_FAILURE_NONE : patsArmingFailureReasonFromFlags(getArmingDisableFlags());
    state->allMotorsSpunUp = patsFlightMotorsHaveSpunUp();
    state->quaternionW = quat.w;
    state->quaternionX = quat.x;
    state->quaternionY = quat.y;
    state->quaternionZ = quat.z;
    state->accelerationXG = 0.0f;
    state->accelerationYG = 0.0f;
    state->accelerationZG = 0.0f;
    state->accelerationValid = false;

#if defined(USE_ACC)
    if (sensors(SENSOR_ACC) && acc.isAccelUpdatedAtLeastOnce && isfinite(acc.dev.acc_1G_rec) && acc.dev.acc_1G_rec > 0.0f) {
        state->accelerationXG = acc.accADC[X] * acc.dev.acc_1G_rec;
        state->accelerationYG = acc.accADC[Y] * acc.dev.acc_1G_rec;
        state->accelerationZG = acc.accADC[Z] * acc.dev.acc_1G_rec;
        state->accelerationValid = true;
    }
#endif
}
