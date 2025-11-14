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
#include "drivers/io.h"
#if defined(USE_TIMER_MGMT)
#include "drivers/dma_reqmap.h"
#endif

#include "fc/init.h"

#include "pg/motor.h"
#if defined(USE_TIMER_MGMT)
#include "pg/timerio.h"
#endif
#if defined(USE_I2C)
#include "pg/bus_i2c.h"
#endif

int main(void)
{
    // Configure the timer resource table so only PB6 is mapped for motor output.
#if defined(USE_TIMER_MGMT) && (MAX_TIMER_PINMAP_COUNT > 0)
    for (unsigned i = 0; i < MAX_TIMER_PINMAP_COUNT; i++) {
        timerIOConfigMutable(i)->ioTag = IO_TAG_NONE;
        timerIOConfigMutable(i)->index = 0;
        timerIOConfigMutable(i)->dmaopt = DMA_OPT_UNUSED;
    }

    timerIOConfigMutable(0)->ioTag = IO_TAG(PB6);
    timerIOConfigMutable(0)->index = 1;
    timerIOConfigMutable(0)->dmaopt = DMA_OPT_UNUSED;
#endif

#if defined(USE_I2C)
    for (int i = 0; i < I2CDEV_COUNT; i++) {
        i2cConfigMutable(i)->ioTagScl = IO_TAG_NONE;
        i2cConfigMutable(i)->ioTagSda = IO_TAG_NONE;
    }
#endif

#ifdef USE_MOTOR
    motorDevConfig_t *motorDevConfig = &motorConfigMutable()->dev;

    motorDevConfig->motorPwmProtocol = PWM_TYPE_DSHOT300;
    motorDevConfig->useBurstDshot = DSHOT_DMAR_OFF;
    motorDevConfig->useDshotTelemetry = 0;
    motorDevConfig->useDshotEdt = 0;
#ifdef USE_DSHOT_BITBANG
    motorDevConfig->useDshotBitbang = DSHOT_BITBANG_OFF;
    motorDevConfig->useDshotBitbangedTimer = DSHOT_BITBANGED_TIMER_AUTO;
#endif

    motorDevConfig->ioTags[0] = IO_TAG(PB6);
    for (uint8_t i = 1; i < MAX_SUPPORTED_MOTORS; i++) {
        motorDevConfig->ioTags[i] = IO_TAG_NONE;
    }
    for (uint8_t i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        motorDevConfig->motorOutputReordering[i] = i;
    }
#endif

    init();

#ifdef USE_MOTOR
    const uint16_t dshotCommandValue = 1000;
    const uint8_t motorCount = motorDeviceCount();
    float motorOutputs[MAX_SUPPORTED_MOTORS];

    for (uint8_t i = 0; i < MAX_SUPPORTED_MOTORS; i++) {
        motorOutputs[i] = 0.0f;
    }

    for (uint8_t i = 0; i < motorCount && i < MAX_SUPPORTED_MOTORS; i++) {
        const uint8_t reorderedIndex = motorConfig()->dev.motorOutputReordering[i];
        if (reorderedIndex < MAX_SUPPORTED_MOTORS &&
            motorConfig()->dev.ioTags[reorderedIndex] != IO_TAG_NONE) {
            motorOutputs[i] = (float)dshotCommandValue;
        }
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
