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

#include "fc/runtime_config.h"
#include "flight/imu.h"
#include "sensors/acceleration.h"

#include "telemetry/pats_flight.h"

static const float kQuatLimit = 0.7071067811865475244f;

uint8_t patsArmingFailureReasonFromFlags(armingDisableFlags_e flags)
{
    if (!flags) {
        return PATS_ARMING_FAILURE_NONE_UNKNOWN;
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
    if (flags & ARMING_DISABLED_BOOT_GRACE_TIME) {
        return PATS_ARMING_FAILURE_BOOT_GRACE_TIME;
    }
    if (flags & ARMING_DISABLED_NOPREARM) {
        return PATS_ARMING_FAILURE_NOPREARM;
    }
    if (flags & ARMING_DISABLED_LOAD) {
        return PATS_ARMING_FAILURE_LOAD;
    }
    if (flags & ARMING_DISABLED_CALIBRATING) {
        return PATS_ARMING_FAILURE_CALIBRATING;
    }
    if (flags & (ARMING_DISABLED_CLI | ARMING_DISABLED_CMS_MENU | ARMING_DISABLED_MSP | ARMING_DISABLED_REBOOT_REQUIRED)) {
        return PATS_ARMING_FAILURE_CONFIG;
    }
    if (flags & (ARMING_DISABLED_GPS | ARMING_DISABLED_RESC)) {
        return PATS_ARMING_FAILURE_GPS_RESCUE;
    }
    if (flags & (ARMING_DISABLED_RUNAWAY_TAKEOFF | ARMING_DISABLED_CRASH_DETECTED)) {
        return PATS_ARMING_FAILURE_CRASH;
    }
    if (flags & (ARMING_DISABLED_RPMFILTER | ARMING_DISABLED_DSHOT_BITBANG | ARMING_DISABLED_MOTOR_PROTOCOL)) {
        return PATS_ARMING_FAILURE_MOTOR;
    }
    if (flags & (ARMING_DISABLED_BST | ARMING_DISABLED_PARALYZE | ARMING_DISABLED_ACC_CALIBRATION | ARMING_DISABLED_ARM_SWITCH)) {
        return PATS_ARMING_FAILURE_SYSTEM;
    }

    return PATS_ARMING_FAILURE_UNKNOWN;
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
    patsFlightPayload_t payload = { { 0 } };

    const bool armed = state && state->armed;
    const uint8_t armingFailureReason = state && state->armingFailureReason <= PATS_ARMING_FAILURE_UNKNOWN
        ? state->armingFailureReason
        : PATS_ARMING_FAILURE_UNKNOWN;
    const uint8_t armingStatus = ((armed ? 1u : 0u) << 4) | (armed ? 0 : armingFailureReason);

    float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
    if (state && state->attitudeValid) {
        q[0] = state->quaternionW;
        q[1] = state->quaternionX;
        q[2] = state->quaternionY;
        q[3] = state->quaternionZ;
    }
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
    if (state && state->accelerationValid) {
        accelerationX = patsFlightEncodeAcceleration6(state->accelerationXG, true);
        accelerationY = patsFlightEncodeAcceleration6(state->accelerationYG, true);
        accelerationZ = patsFlightEncodeAcceleration5(state->accelerationZG, true);
    }

    const uint64_t packed =
        ((uint64_t)(armingStatus & 0x1F) << 43) |
        ((uint64_t)(largestQuatIndex & 0x03) << 41) |
        ((uint64_t)(quatComponent[0] & 0xFF) << 33) |
        ((uint64_t)(quatComponent[1] & 0xFF) << 25) |
        ((uint64_t)(quatComponent[2] & 0xFF) << 17) |
        ((uint64_t)(accelerationX & 0x3F) << 11) |
        ((uint64_t)(accelerationY & 0x3F) << 5) |
        ((uint64_t)(accelerationZ & 0x1F));

    for (size_t ii = 0; ii < PATS_FLIGHT_PAYLOAD_SIZE; ++ii) {
        payload.bytes[ii] = (uint8_t)((packed >> (40 - 8 * ii)) & 0xFF);
    }

    return payload;
}

bool decodePatsFlightPayload(const uint8_t payload[PATS_FLIGHT_PAYLOAD_SIZE], decodedPatsFlightTelemetry_t *output)
{
    if (!payload || !output) {
        return false;
    }

    uint64_t packed = 0;
    for (size_t ii = 0; ii < PATS_FLIGHT_PAYLOAD_SIZE; ++ii) {
        packed = (packed << 8) | payload[ii];
    }

    memset(output, 0, sizeof(*output));
    const uint8_t armingStatus = (packed >> 43) & 0x1F;
    output->armed = (armingStatus & 0x10) != 0;
    output->armingFailureReason = armingStatus & 0x0F;
    if (output->armed) {
        output->armingFailureReason = 0;
    }
    output->largestQuatIndex = (packed >> 41) & 0x03;
    output->quatComponent[0] = (packed >> 33) & 0xFF;
    output->quatComponent[1] = (packed >> 25) & 0xFF;
    output->quatComponent[2] = (packed >> 17) & 0xFF;
    output->accelerationXCode = (packed >> 11) & 0x3F;
    output->accelerationYCode = (packed >> 5) & 0x3F;
    output->accelerationZCode = packed & 0x1F;

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
    if (!state) {
        return;
    }

    quaternion quat;
    getQuaternion(&quat);

    state->armed = ARMING_FLAG(ARMED);
    state->armingFailureReason = state->armed ? PATS_ARMING_FAILURE_NONE_UNKNOWN : patsArmingFailureReasonFromFlags(getArmingDisableFlags());
    state->quaternionW = quat.w;
    state->quaternionX = quat.x;
    state->quaternionY = quat.y;
    state->quaternionZ = quat.z;
    state->attitudeValid = true;
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
