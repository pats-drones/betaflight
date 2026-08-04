/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 */

#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "fc/runtime_config.h"
#include "rx/crsf_protocol.h"

#define PATS_FLIGHT_PAYLOAD_SIZE ((size_t)CRSF_FRAME_PATS_FLIGHT_PAYLOAD_SIZE)
#define PATS_FLIGHT_CRSF_FRAME_SIZE (PATS_FLIGHT_PAYLOAD_SIZE + (size_t)CRSF_FRAME_LENGTH_NON_PAYLOAD)

typedef enum {
    PATS_ARMING_FAILURE_NONE_UNKNOWN = 0,
    PATS_ARMING_FAILURE_NO_GYRO = 1,
    PATS_ARMING_FAILURE_FAILSAFE = 2,
    PATS_ARMING_FAILURE_RX = 3,
    PATS_ARMING_FAILURE_THROTTLE = 4,
    PATS_ARMING_FAILURE_ANGLE = 5,
    PATS_ARMING_FAILURE_BOOT_GRACE_TIME = 6,
    PATS_ARMING_FAILURE_NOPREARM = 7,
    PATS_ARMING_FAILURE_LOAD = 8,
    PATS_ARMING_FAILURE_CALIBRATING = 9,
    PATS_ARMING_FAILURE_CONFIG = 10,
    PATS_ARMING_FAILURE_GPS_RESCUE = 11,
    PATS_ARMING_FAILURE_CRASH = 12,
    PATS_ARMING_FAILURE_MOTOR = 13,
    PATS_ARMING_FAILURE_SYSTEM = 14,
    PATS_ARMING_FAILURE_UNKNOWN = 15,
} patsArmingFailureReason_e;

typedef enum {
    PATS_ACCELERATION_INVALID = 0,
    PATS_ACCELERATION_NEGATIVE_OVERFLOW,
    PATS_ACCELERATION_NUMERIC,
    PATS_ACCELERATION_POSITIVE_OVERFLOW,
} patsAccelerationStatus_e;

typedef struct {
    float valueG;
    patsAccelerationStatus_e status;
} patsDecodedAcceleration_t;

typedef struct {
    bool armed;
    uint8_t armingFailureReason;

    float quaternionW;
    float quaternionX;
    float quaternionY;
    float quaternionZ;
    bool attitudeValid;

    // Calibrated, board-aligned accelerometer proper acceleration in g, including gravity.
    // These are body/sensor axes from acc.accADC[] after alignment/trims, not a yaw-aligned world frame.
    float accelerationXG;
    float accelerationYG;
    float accelerationZG;
    bool accelerationValid;
} patsFlightTelemetryState_t;

typedef struct {
    uint8_t bytes[PATS_FLIGHT_PAYLOAD_SIZE];
} patsFlightPayload_t;

typedef struct {
    bool armed;
    uint8_t armingFailureReason;
    uint8_t largestQuatIndex;
    uint8_t quatComponent[3];
    uint8_t accelerationXCode;
    uint8_t accelerationYCode;
    uint8_t accelerationZCode;

    float quaternionW;
    float quaternionX;
    float quaternionY;
    float quaternionZ;
    patsDecodedAcceleration_t accelerationX;
    patsDecodedAcceleration_t accelerationY;
    patsDecodedAcceleration_t accelerationZ;
} decodedPatsFlightTelemetry_t;

uint8_t patsArmingFailureReasonFromFlags(armingDisableFlags_e flags);
uint8_t patsFlightEncodeAcceleration6(float accelerationG, bool valid);
uint8_t patsFlightEncodeAcceleration5(float accelerationG, bool valid);
patsDecodedAcceleration_t patsFlightDecodeAcceleration6(uint8_t code);
patsDecodedAcceleration_t patsFlightDecodeAcceleration5(uint8_t code);

patsFlightPayload_t encodePatsFlightPayload(const patsFlightTelemetryState_t *state);
bool decodePatsFlightPayload(const uint8_t payload[PATS_FLIGHT_PAYLOAD_SIZE], decodedPatsFlightTelemetry_t *output);
bool buildPatsFlightCrsfFrame(const patsFlightTelemetryState_t *state, uint8_t *destination, size_t destinationSize, size_t *frameSize);
bool decodePatsFlightCrsfFrame(const uint8_t *frame, size_t frameSize, decodedPatsFlightTelemetry_t *output);

void patsFlightTelemetryStateFromFc(patsFlightTelemetryState_t *state);

#ifdef __cplusplus
static_assert(PATS_FLIGHT_PAYLOAD_SIZE == 6, "PATS flight payload must stay 6 bytes");
static_assert(PATS_FLIGHT_CRSF_FRAME_SIZE == 10, "PATS flight CRSF frame must stay 10 bytes");
#else
_Static_assert(PATS_FLIGHT_PAYLOAD_SIZE == 6, "PATS flight payload must stay 6 bytes");
_Static_assert(PATS_FLIGHT_CRSF_FRAME_SIZE == 10, "PATS flight CRSF frame must stay 10 bytes");
#endif
