/*
 * This file is part of Cleanflight and Betaflight.
 *
 * Cleanflight and Betaflight are free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * Cleanflight and Betaflight are distributed in the hope that they
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

extern "C" {
#include "platform.h"

#include "config/feature.h"

#include "drivers/motor.h"
#include "drivers/pwm_output.h"
#include "drivers/time.h"
#ifdef USE_DSHOT_BITBANG
#include "drivers/dshot_bitbang.h"
#endif

#include "fc/init.h"
#include "fc/rc_controls.h"
#include "fc/runtime_config.h"

#include "flight/mixer.h"

#include "pg/motor.h"
}

static void configureDshot300(void)
{
    motorDevConfig_t *motorDevConfig = &motorConfigMutable()->dev;

    motorDevConfig->motorPwmProtocol = PWM_TYPE_DSHOT300;
    motorDevConfig->useUnsyncedPwm = false;
#ifdef USE_DSHOT_DMAR
    motorDevConfig->useBurstDshot = DSHOT_DMAR_OFF;
#else
    motorDevConfig->useBurstDshot = 0;
#endif
    motorDevConfig->useDshotTelemetry = false;
    motorDevConfig->useDshotEdt = false;
#ifdef USE_DSHOT_BITBANG
    motorDevConfig->useDshotBitbang = DSHOT_BITBANG_OFF;
    motorDevConfig->useDshotBitbangedTimer = DSHOT_BITBANGED_TIMER_AUTO;
#endif

    motorShutdown();

    uint16_t idlePulse = motorConfig()->mincommand;
    if (featureIsEnabled(FEATURE_3D)) {
        idlePulse = flight3DConfig()->neutral3d;
    }

    motorDevInit(motorDevConfig, idlePulse, getMotorCount());
    motorEnable();
}

static void runConstantDshotStream(void)
{
    const uint8_t motorCount = motorDeviceCount();
    if (motorCount == 0) {
        while (true) {
            delay(100);
        }
    }

    float outputs[MAX_SUPPORTED_MOTORS];
    for (uint8_t i = 0; i < motorCount; i++) {
        outputs[i] = 1000.0f;
    }

    while (true) {
        motorWriteAll(outputs);
        delayMicroseconds(50);
    }
}

int main(void)
{
    init();
    configureDshot300();
    runConstantDshotStream();
    return 0;
}
