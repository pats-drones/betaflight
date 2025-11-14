/*
 * Minimal standalone program to continuously bit-bang a DShot300 signal on PB6.
 * Generates a constant throttle command with value 1000 (telemetry disabled).
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

static void initPb6Gpio(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    (void)RCC->AHB1ENR;

    GPIOB->MODER &= ~(3U << (6U * 2U));
    GPIOB->MODER |= (1U << (6U * 2U));

    GPIOB->OTYPER &= ~(1U << 6U);
    GPIOB->OSPEEDR |= (3U << (6U * 2U));
    GPIOB->PUPDR &= ~(3U << (6U * 2U));

    GPIOB->BSRR = (uint32_t)(1U << (6U + 16U));
}

static void sendDshotFrame(uint16_t packet, uint32_t bitTicks, uint32_t oneHighTicks, uint32_t zeroHighTicks)
{
    for (int bit = 0; bit < 16; bit++) {
        const uint32_t start = DWT->CYCCNT;
        const uint32_t highTicks = (packet & 0x8000U) ? oneHighTicks : zeroHighTicks;

        GPIOB->BSRR = (1U << 6U);
        while ((DWT->CYCCNT - start) < highTicks) {
            __NOP();
        }

        GPIOB->BSRR = (1U << (6U + 16U));
        while ((DWT->CYCCNT - start) < bitTicks) {
            __NOP();
        }

        packet <<= 1;
    }
}

int main(void)
{
    enableDwtCycleCounter();
    initPb6Gpio();

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
