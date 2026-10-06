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
#include <flight/mixer.h>
#include "sensors/acceleration.h"

#include "telemetry/pats_flight.h"

static const float kQuatLimit = 0.7071067811865475244f;


static uint8_t encodeQuatComponent(float component)
{
    const float clamped = constrainf(component, -kQuatLimit, kQuatLimit);
    return (uint8_t)constrain((int)lroundf((clamped + kQuatLimit) * 255.0f / (2.0f * kQuatLimit)), 0, 255);
}

static uint8_t quantizeMotorCommand(float motor_val, int nbits) {
    const float low = getMotorOutputLow();
    const float high = getMotorOutputHigh();
    float normalized = constrainf((motor_val - low) / (high - low), 0.0f, 1.0f);
    return (uint8_t)lrintf(normalized * powf(2.0f,nbits - 1));
}

patsFlightPayload_t encodePatsFlightPayload(void)
{
    patsFlightPayload_t payload = { 0 };
    const uint16_t motorSpinupErpm = motorConfig()->patsFlightMotorSpinupErpm;
    const bool motor0SpunUp = getDshotMotorAverageRpm(0) > motorSpinupErpm;
    const bool motor1SpunUp = getDshotMotorAverageRpm(1) > motorSpinupErpm;
    const bool motor2SpunUp = getDshotMotorAverageRpm(2) > motorSpinupErpm;
    const bool motor3SpunUp = getDshotMotorAverageRpm(3) > motorSpinupErpm;

    float min_motor;
    float max_motor;
    float avg_motor;
    for (size_t ii = 0; ii < 4; ii++) {
        // getDshotMotorAverageRpm(ii)
        float currMotor = motor[ii];
        if (min_motor > currMotor) 
            min_motor = currMotor;
        if (max_motor < currMotor)
            max_motor = currMotor;
        avg_motor += currMotor;
    }
    avg_motor /= 4;

    float q[4];
    quaternion quat;
    getQuaternion(&quat);

    const float normSquared = quat.w * quat.w + quat.x * quat.x + quat.y * quat.y + quat.z * quat.z;
    const float invNorm = 1.0f / sqrtf(normSquared);

    q[0] = quat.w * invNorm;
    q[1] = quat.x * invNorm;
    q[2] = quat.y * invNorm;
    q[3] = quat.z * invNorm;

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


    payload.motor0SpunUp = motor0SpunUp;
    payload.motor1SpunUp = motor1SpunUp;
    payload.motor2SpunUp = motor2SpunUp;
    payload.motor3SpunUp = motor3SpunUp;
    payload.largestQuatIndex = largestQuatIndex;
    payload.quaternionComponent0 = quatComponent[0];
    payload.quaternionComponent1 = quatComponent[1];
    payload.quaternionComponent2 = quatComponent[2];
    payload.minDshotCommand = quantizeMotorCommand(min_motor, 4);
    payload.maxDshotCommand = quantizeMotorCommand(max_motor, 4);
    payload.avgDshotCommand = quantizeMotorCommand(avg_motor, 4);

    // uint16_t erpmMotor0 = getDshotMotorAverageRpm(0);
    // uint16_t erpmMotor1 = getDshotMotorAverageRpm(1);
    // uint16_t erpmMotor2 = getDshotMotorAverageRpm(2);
    // uint16_t erpmMotor3 = getDshotMotorAverageRpm(3);

    // payload.bytes[0] = (uint8_t)(erpmMotor0 >> 8); 
    // payload.bytes[1] = (uint8_t)((erpmMotor0 & 0x00E0) | (motor0SpunUp ? 0x10 : 0x00) | ((erpmMotor1 & 0xF000) >> 12));
    // payload.bytes[2] = (uint8_t)(((erpmMotor1 & 0x0FE0) >> 4) | (motor1SpunUp ? 0x01 : 0x00)); 
    // payload.bytes[3] = (uint8_t)(erpmMotor2 >> 8); 
    // payload.bytes[4] = (uint8_t)((erpmMotor2 & 0x00E0) | (motor2SpunUp ? 0x10 : 0x00) | ((erpmMotor3 & 0xF000) >> 12));
    // payload.bytes[5] = (uint8_t)(((erpmMotor3 & 0x0FE0) >> 4) | (motor3SpunUp ? 0x01 : 0x00)); 

    return payload;
}
