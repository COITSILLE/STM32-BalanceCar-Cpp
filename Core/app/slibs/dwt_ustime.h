#ifndef __DWT_USTIME_H__
#define __DWT_USTIME_H__

#include "main.h"
#include <stdint.h>

typedef uint64_t timestamp_t;

#define NON_BLOCK_DELAY(t)                      \
    do{                                         \
        static uint32_t lastmm = 0;             \
        if (HAL_GetTick() - lastmm < t) return; \
        lastmm = HAL_GetTick();                 \
    }while (0)

#define SYS_CLOCK_MHZ 72U
#define mstick(x) ((uint32_t)((x) >> 32U))
#define ustick(x) ((uint32_t)((x) & 0xFFFFFFFFULL))
static const float INV_SYS_CLOCK_US = 1.0f / SYS_CLOCK_MHZ;
static const uint32_t CAL_FACTOR = ((uint32_t)(((1ULL<<32) + SYS_CLOCK_MHZ / 2) / SYS_CLOCK_MHZ));
static const uint32_t EDGE = (uint32_t)((1ULL<<32) * INV_SYS_CLOCK_US / 1000U);

static inline void dwt_ustime_init(void){
    // Enable the DWT counter
    if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    }
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    DWT->CYCCNT = 0;
}

static inline timestamp_t getTick(){
    return ((uint64_t)uwTick << 32U) | (uint64_t)DWT->CYCCNT;
}
/**
 *@return end - start in microseconds
*/
static inline uint32_t getDistance(const timestamp_t start, const timestamp_t end){
    if (mstick(end) - mstick(start) > (EDGE - 1)){
        return (mstick(end) - mstick(start)) * 1000U;
    }
    else {
        return ((uint64_t)(ustick(end) - ustick(start)) * CAL_FACTOR) >> 32U;
    }
}

#endif /* __DWT_USTIME_H__ */