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
#include "sensors/acceleration.h"

#include "telemetry/pats_flight.h"

static const float kQuatLimit = 0.7071067811865475244f;

static bool patsFlightMotorsHaveSpunUp(void)
{
    if (!useDshotTelemetry) {
        return false;
    }

    const unsigned motorCount = motorDeviceCount();
    for (unsigned motor = 0; motor < motorCount; motor++) {
        if (getDshotMotorAverageRpm(motor) < PATS_FLIGHT_MOTOR_SPINUP_eRPM) {
            return false;
        }
    }
    return motorCount != 0;
}

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

static uint8_t encodeAcceleration(float accelerationG, bool valid, uint8_t maximumCode, uint8_t zeroCode)
{
    if (!valid || !isfinite(accelerationG)) {
        return 0;
    }

    const float maximumAccelerationG = (maximumCode - 1 - zeroCode) * 0.25f;
    if (accelerationG < -maximumAccelerationG) {
        return 1;
    }
    if (accelerationG > maximumAccelerationG) {
        return maximumCode;
    }

    return (uint8_t)constrain((int)lroundf(accelerationG / 0.25f) + zeroCode, 2, maximumCode - 1);
}

uint8_t patsFlightEncodeAcceleration6(float accelerationG, bool valid)
{
    return encodeAcceleration(accelerationG, valid, 63, 32);
}

uint8_t patsFlightEncodeAcceleration5(float accelerationG, bool valid)
{
    return encodeAcceleration(accelerationG, valid, 31, 16);
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

patsFlightPayload_t encodePatsFlightPayload(void)
{
    patsFlightPayload_t payload = { 0 };
    const bool armed = ARMING_FLAG(ARMED);

    payload.armed = armed;
    payload.armingFailureReason = armed ? PATS_ARMING_FAILURE_NONE : patsArmingFailureReasonFromFlags(getArmingDisableFlags());
    payload.allMotorsSpunUp = patsFlightMotorsHaveSpunUp();

    quaternion quat;
    getQuaternion(&quat);

    float q[4] = { quat.w, quat.x, quat.y, quat.z };
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
#if defined(USE_ACC)
    if (sensors(SENSOR_ACC) && acc.isAccelUpdatedAtLeastOnce && isfinite(acc.dev.acc_1G_rec) && acc.dev.acc_1G_rec > 0.0f) {
        accelerationX = patsFlightEncodeAcceleration6(acc.accADC[X] * acc.dev.acc_1G_rec, true);
        accelerationY = patsFlightEncodeAcceleration6(acc.accADC[Y] * acc.dev.acc_1G_rec, true);
        accelerationZ = patsFlightEncodeAcceleration5(acc.accADC[Z] * acc.dev.acc_1G_rec, true);
    }
#endif

    payload.largestQuatIndex = largestQuatIndex;
    payload.quaternionComponent0 = quatComponent[0];
    payload.quaternionComponent1 = quatComponent[1];
    payload.quaternionComponent2 = quatComponent[2];
    payload.accelerationX = accelerationX;
    payload.accelerationY = accelerationY;
    payload.accelerationZ = accelerationZ;
    return payload;
}
