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

#include <stdbool.h>
#include <stdint.h>

#include "platform.h"

#include "drivers/time.h"
#include "drivers/motor.h"

#include "fc/init.h"

#include "flight/mixer.h"

#include "pg/motor.h"

int main(void)
{
    init();

#ifdef USE_MOTOR
    motorShutdown();

    motorConfigMutable()->dev.motorPwmProtocol = PWM_TYPE_DSHOT300;
    motorConfigMutable()->dev.useDshotTelemetry = 0;
    motorConfigMutable()->dev.useDshotEdt = 0;
#ifdef USE_DSHOT_BITBANG
    motorConfigMutable()->dev.useDshotBitbang = 0;
    motorConfigMutable()->dev.useDshotBitbangedTimer = 0;
#endif

    motorDevInit(&motorConfig()->dev, motorConfig()->mincommand, getMotorCount());
    motorEnable();

    const uint16_t dshotCommandValue = 1000;
    const uint8_t motorCount = motorDeviceCount();
    float motorOutputs[MAX_SUPPORTED_MOTORS];

    for (uint8_t i = 0; i < motorCount && i < MAX_SUPPORTED_MOTORS; i++) {
        motorOutputs[i] = (float)dshotCommandValue;
    }

    while (true) {
        motorWriteAll(motorOutputs);
        delay(1);
    }
#else
    while (true) {
        // Nothing to do if motors are not supported on this build.
    }
#endif

    return 0;
}
