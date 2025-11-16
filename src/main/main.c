/*
 * Minimal standalone program to continuously bit-bang a DShot300 signal on PB6/PB7/PB8/PB10.
 * Generates a constant throttle command with value 1000 (telemetry disabled) on every pin.
 */

#include <stdint.h>
#include "stm32f4xx.h"

#ifndef DSHOT_THROTTLE_VALUE
#define DSHOT_THROTTLE_VALUE 1000U
#endif

#ifndef DSHOT_FRAME_GAP_US
#define DSHOT_FRAME_GAP_US 60U
#endif

static void enableDwtCycleCounter(void)
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static inline void delayTicks(uint32_t ticks)
{
    const uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < ticks) {
        __NOP();
    }
}

static uint32_t getCoreClockHz(void)
{
    extern uint32_t SystemCoreClock;
    if (SystemCoreClock != 0U) {
        return SystemCoreClock;
    }

    return 168000000U;
}

static uint16_t buildDshotFrame(uint16_t throttle)
{
    throttle &= 0x7FFU;
    uint16_t packet = (uint16_t)(throttle << 1); // Telemetry bit cleared

    uint16_t checksum = 0;
    uint16_t checksumData = packet;
    for (int i = 0; i < 3; i++) {
        checksum ^= (checksumData & 0xFU);
        checksumData >>= 4;
    }
    checksum &= 0xFU;

    packet = (uint16_t)((packet << 4) | checksum);
    return packet;
}

static const uint8_t dshotPins[] = { 6U, 7U, 8U, 10U };
static uint16_t dshotPinMask = 0U;

static void initGpio(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    (void)RCC->AHB1ENR;

    for (unsigned i = 0; i < (sizeof(dshotPins) / sizeof(dshotPins[0])); i++) {
        const uint32_t pin = dshotPins[i];
        const uint32_t pinMask = 1U << pin;
        dshotPinMask |= (uint16_t)pinMask;

        GPIOB->MODER &= ~(3U << (pin * 2U));
        GPIOB->MODER |= (1U << (pin * 2U));

        GPIOB->OTYPER &= ~pinMask;
        GPIOB->OSPEEDR |= (3U << (pin * 2U));
        GPIOB->PUPDR &= ~(3U << (pin * 2U));
    }

    GPIOB->BSRRH = dshotPinMask;
}

static void sendDshotFrame(uint16_t packet, uint32_t bitTicks, uint32_t oneHighTicks, uint32_t zeroHighTicks)
{
    for (int bit = 0; bit < 16; bit++) {
        const uint32_t start = DWT->CYCCNT;
        const uint32_t highTicks = (packet & 0x8000U) ? oneHighTicks : zeroHighTicks;

        GPIOB->BSRRL = dshotPinMask;
        while ((DWT->CYCCNT - start) < highTicks) {
            __NOP();
        }

        GPIOB->BSRRH = dshotPinMask;
        while ((DWT->CYCCNT - start) < bitTicks) {
            __NOP();
        }

        packet <<= 1;
    }
}

int main(void)
{
    enableDwtCycleCounter();
    initGpio();

    const uint32_t coreClockHz = getCoreClockHz();
    const uint32_t bitTicks = coreClockHz / 300000U;
    const uint32_t oneHighTicks = (bitTicks * 2U) / 3U;
    const uint32_t zeroHighTicks = bitTicks / 3U;
    const uint32_t frameGapTicks = (uint32_t)(((uint64_t)coreClockHz * DSHOT_FRAME_GAP_US) / 1000000ULL);

    const uint16_t packet = buildDshotFrame(DSHOT_THROTTLE_VALUE);

    while (1) {
        sendDshotFrame(packet, bitTicks, oneHighTicks, zeroHighTicks);
        delayTicks(frameGapTicks);
    }
}
