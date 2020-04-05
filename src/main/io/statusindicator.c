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

#include "drivers/light_led.h"
#include "drivers/time.h"

#include "io/ledstrip.h"
#include "drivers/pwm_output.h"
#include "flight/mixer.h"
#include "fc/rc_modes.h"
#include "fc/runtime_config.h"
#include "rx/rx.h"

#include "statusindicator.h"

static uint32_t warningLedTimer = 0;

typedef enum {
    WARNING_LED_OFF = 0,
    WARNING_LED_ON,
    WARNING_LED_FLASH
} warningLedState_e;

static warningLedState_e warningLedState = WARNING_LED_OFF;

void warningLedResetTimer(void) {
    uint32_t now = millis();
    warningLedTimer = now + 500000;
}

void warningLedEnable(void)
{
    warningLedState = WARNING_LED_ON;
}

void warningLedDisable(void)
{
    warningLedState = WARNING_LED_OFF;
}

void warningLedFlash(void)
{
    warningLedState = WARNING_LED_FLASH;
}

void warningLedRefresh(void)
{
    switch (warningLedState) {
        case WARNING_LED_OFF:
            LED0_OFF;
            break;
        case WARNING_LED_ON:
            LED0_ON;
            break;
        case WARNING_LED_FLASH :{
            static bool led_strip_toggle = false;
            led_strip_toggle = !led_strip_toggle;
#ifdef USE_LED_STRIP
            if (rxIsReceivingSignal() && ((getArmingDisableFlags() & ARMING_DISABLED_ARM_SWITCH)) ){
                if (led_strip_toggle)
                    ledStripEnable();
                else
                    ledStripDisable(false);
            }
#endif
            LED0_TOGGLE;
            break;
        }
    }

    uint32_t now = micros();
    warningLedTimer = now + 500000;
}

void warningLedUpdate(void)
{
    uint32_t now = micros();
#ifdef USE_DSHOT
    //https://github.com/pats-drones/pats/issues/222
    if (warningLedState == WARNING_LED_FLASH && rxIsReceivingSignal() && ((getArmingDisableFlags() & ARMING_DISABLED_ARM_SWITCH))) {
        pwmWriteDshotCommand(ALL_MOTORS, getMotorCount(), 1, false);
    }
#endif

    if ((int32_t)(now - warningLedTimer) < 0) {
        return;
    }

    warningLedRefresh();
}
