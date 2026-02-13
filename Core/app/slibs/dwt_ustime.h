#ifndef __DWT_USTIME_H__
#define __DWT_USTIME_H__

#include "main.h"
#include <stdint.h>

typedef struct{
    uint32_t count;
    uint32_t ms_tick;
} DWT_Timestamp;

#define NON_BLOCK_DELAY(t)                      \
    do{                                         \
        static uint32_t lastmm = 0;             \
        if (HAL_GetTick() - lastmm < t) return; \
        lastmm = HAL_GetTick();                 \
    }while (0)

#define SYS_CLOCK_MHZ 72U
static const float INV_SYS_CLOCK_US = 1.0f / SYS_CLOCK_MHZ;
static const uint32_t CAL_FACTOR = ((uint32_t)(((1ULL<<32) + SYS_CLOCK_MHZ / 2) / SYS_CLOCK_MHZ));
static const uint32_t EDGE = (uint32_t)((1ULL<<32) * INV_SYS_CLOCK_US / 1000U);

static inline void dwt_ustime_init(void)
{
    // Enable the DWT counter
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
}

static inline DWT_Timestamp getTick(){
    DWT_Timestamp return_Val;
    return_Val.count = DWT->CYCCNT;
    return_Val.ms_tick = uwTick;
    return return_Val;
}
/**
 *@return end - start in microseconds
*/
static inline uint32_t getDistance(const DWT_Timestamp start, const DWT_Timestamp end){
    if (end.ms_tick - start.ms_tick > (EDGE - 1)){
        return (end.ms_tick - start.ms_tick) * 1000U;
    }
    else {
        return ((uint64_t)(end.count - start.count) * CAL_FACTOR) >> 32;
    }
}

#endif /* __DWT_USTIME_H__ */