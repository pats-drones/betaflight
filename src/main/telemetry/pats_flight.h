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

typedef union {
    struct {
        uint32_t quaternionComponent0 : 10,
                 quaternionComponent1 : 10,
                 quaternionComponent2 : 10,
                 largestQuatIndex : 2;
        uint16_t motor0SpunUp : 1,
                 motor1SpunUp : 1,
                 motor2SpunUp : 1,
                 motor3SpunUp : 1,
                 minDshotCommand : 4,
                 maxDshotCommand : 4,
                 avgDshotCommand : 4;
    } __attribute__ ((__packed__));
    uint8_t bytes[PATS_FLIGHT_PAYLOAD_SIZE];
} __attribute__ ((__packed__)) patsFlightPayload_t;


uint8_t patsFlightEncodeAcceleration6(float accelerationG);
uint8_t patsFlightEncodeAcceleration5(float accelerationG);

patsFlightPayload_t encodePatsFlightPayload(void);

