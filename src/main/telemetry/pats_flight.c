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

#include "platform.h"

#include "common/axis.h"
#include "common/maths.h"

#ifdef USE_DSHOT_TELEMETRY
#include "drivers/dshot.h"
#include "drivers/motor.h"
#endif

#include "fc/runtime_config.h"
#include "flight/imu.h"
#include "flight/mixer.h"
#include "sensors/acceleration.h"

#include "telemetry/pats_flight.h"

static const float kQuatLimit = 0.7071067811865475244f;

uint16_t encodeQuatComponent(float component, int nbits)
{
    const float clamped = constrainf(component, -kQuatLimit, kQuatLimit);
    int nsteps =  (1<<nbits) - 1;
    return (uint16_t)constrain((int)lroundf((clamped + kQuatLimit) * ((float)nsteps)  / (2.0f * kQuatLimit)), 0, nsteps);
}

uint8_t quantizeMotorCommand(float motor_val, int nbits) {
    const float low = getMotorOutputLow();
    const float high = getMotorOutputHigh();
    float normalized = constrainf((motor_val - low) / (high - low), 0.0f, 1.0f);
    return (uint8_t)lrintf(normalized * (float)((1<<nbits) - 1));
}

patsFlightPayload_t encodePatsFlightPayload(void)
{
    patsFlightPayload_t payload = { 0 };
    const uint16_t motorThresh = motorConfig()->pats_eRPM_thresh;
    payload.motor0spunUp = getDshotMotorAverageRpm(0) > motorThresh;
    payload.motor1spunUp = getDshotMotorAverageRpm(1) > motorThresh;
    payload.motor2spunUp = getDshotMotorAverageRpm(2) > motorThresh;
    payload.motor3spunUp = getDshotMotorAverageRpm(3) > motorThresh;

    float min_motor = motor[0];
    float max_motor = motor[0];
    float avg_motor = motor[0];
    for (size_t ii = 1; ii < 4; ii++) {
        // getDshotMotorAverageRpm(ii)
        float currMotor = motor[ii];
        if (min_motor > currMotor) 
            min_motor = currMotor;
        if (max_motor < currMotor)
            max_motor = currMotor;
        avg_motor += currMotor;
    }
    avg_motor /= 4;

    payload.minDshotCommand = quantizeMotorCommand(min_motor, 4);
    payload.maxDshotCommand = quantizeMotorCommand(max_motor, 4);
    payload.avgDshotCommand = quantizeMotorCommand(avg_motor, 4);

    quaternion quat;
    getQuaternion(&quat);

    float q[4];
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

    uint16_t quatComponent[3];
    for (uint8_t ii = 0, jj = 0; ii < 4; ++ii) {
        if (ii != largestQuatIndex) {
            quatComponent[jj++] = encodeQuatComponent(q[ii], 10);
        }
    }

    payload.largestQuatIndex = largestQuatIndex;
    payload.quaternionComponent0 = quatComponent[0];
    payload.quaternionComponent1 = quatComponent[1];
    payload.quaternionComponent2 = quatComponent[2];
    return payload;
}
