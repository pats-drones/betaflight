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

// PATS flight payload: quaternion (32), all motors spin states up (4), dshot min/max/avg commands (12).
typedef union {
    struct {
        uint32_t quaternionComponent0 : 10,
                 quaternionComponent1 : 10,
                 quaternionComponent2 : 10,
                 largestQuatIndex : 2;
        uint8_t motor0spunUp : 1,
                motor1spunUp : 1,
                motor2spunUp : 1,
                motor3spunUp : 1,
                minDshotCommand : 4;
        uint8_t maxDshotCommand : 4,
                avgDshotCommand : 4;
    } __attribute__ ((__packed__));
    uint8_t bytes[PATS_FLIGHT_PAYLOAD_SIZE];
} __attribute__ ((__packed__)) patsFlightPayload_t;


patsFlightPayload_t encodePatsFlightPayload(void);
uint16_t encodeQuatComponent(float component, int nbits);
uint8_t quantizeMotorCommand(float motor_val, int nbits);