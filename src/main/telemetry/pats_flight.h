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

// PATS flight payload: quaternion (26), acceleration (17), armed (1), primary arming failure (3), all motors spun up (1).
typedef enum {
    PATS_ARMING_FAILURE_NONE = 0,
    PATS_ARMING_FAILURE_NO_GYRO = 1,
    PATS_ARMING_FAILURE_FAILSAFE = 2,
    PATS_ARMING_FAILURE_RX = 3,
    PATS_ARMING_FAILURE_THROTTLE = 4,
    PATS_ARMING_FAILURE_ANGLE = 5,
    PATS_ARMING_FAILURE_CONFIG = 6,
    PATS_ARMING_FAILURE_SYSTEM = 7,
} patsArmingFailureReason_e;

typedef union {
    struct {
        uint8_t quaternionComponent0 : 8;
        uint8_t quaternionComponent1 : 8;
        uint8_t quaternionComponent2 : 8;
        uint8_t accelerationX : 6,
                largestQuatIndex : 2;
        uint8_t accelerationY : 6,
                allMotorsSpunUp : 1,
                armed : 1;
        uint8_t accelerationZ : 5,
                armingFailureReason : 3;
    } __attribute__ ((__packed__));
    uint8_t bytes[PATS_FLIGHT_PAYLOAD_SIZE];
} __attribute__ ((__packed__)) patsFlightPayload_t;

uint8_t patsArmingFailureReasonFromFlags(armingDisableFlags_e flags);
uint8_t patsFlightEncodeAcceleration6(float accelerationG, bool valid);
uint8_t patsFlightEncodeAcceleration5(float accelerationG, bool valid);

patsFlightPayload_t encodePatsFlightPayload(void);

#ifdef __cplusplus
static_assert(PATS_FLIGHT_PAYLOAD_SIZE == 6, "PATS flight payload must stay 6 bytes");
static_assert(PATS_FLIGHT_CRSF_FRAME_SIZE == 10, "PATS flight CRSF frame must stay 10 bytes");
#else
_Static_assert(PATS_FLIGHT_PAYLOAD_SIZE == 6, "PATS flight payload must stay 6 bytes");
_Static_assert(PATS_FLIGHT_CRSF_FRAME_SIZE == 10, "PATS flight CRSF frame must stay 10 bytes");
#endif
