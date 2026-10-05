#ifndef INC_GLOVE_WATCHDOG_H_
#define INC_GLOVE_WATCHDOG_H_

#include "main.h"

#ifndef BOWIE_IWDG_ENABLE
#define BOWIE_IWDG_ENABLE 1
#endif

/* LSI ~32 kHz, /64 prescaler, 2047 ticks => about 4.1 seconds. */
#define BOWIE_IWDG_PRESCALER 4U
#define BOWIE_IWDG_RELOAD    2047U

static inline void bowie_watchdog_init(void)
{
#if BOWIE_IWDG_ENABLE
    uint32_t started_ms;

    IWDG->KR = 0xCCCCU; /* Start IWDG first, then allow register updates. */
    IWDG->KR = 0x5555U;
    IWDG->PR = BOWIE_IWDG_PRESCALER;
    IWDG->RLR = BOWIE_IWDG_RELOAD;

    started_ms = HAL_GetTick();
    while (((IWDG->SR & (IWDG_SR_PVU | IWDG_SR_RVU)) != 0U) &&
           ((uint32_t)(HAL_GetTick() - started_ms) < 100U))
    {
    }
    IWDG->KR = 0xAAAAU;
#endif
}

static inline void bowie_watchdog_feed(void)
{
#if BOWIE_IWDG_ENABLE
    IWDG->KR = 0xAAAAU;
#endif
}

#endif /* INC_GLOVE_WATCHDOG_H_ */
