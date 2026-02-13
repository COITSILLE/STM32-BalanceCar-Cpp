#ifndef __FPU_H__
#define __FPU_H__

#include "main.h"

static inline void EnableFPU(void)
{
    SCB->CPACR |= (3UL << 20) | (3UL << 22);
    __DSB(); 
    __ISB(); 
}

#endif /* __FPU_H__ */