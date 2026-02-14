#ifndef FLASH_H
#define FLASH_H 

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define FLASH_SIZE_REG_VAL_KB (*((uint16_t *)FLASHSIZE_BASE))
#define FLASH_PAGE_END (FLASH_BASE + (FLASH_SIZE_REG_VAL_KB * 1024U))

//from end distribute
typedef enum {
    PAGE0 = 0,
    PAGE1 = 1,
    PAGE2 = 2,
    PAGE3 = 3,
    PAGE4 = 4, 
    PAGE5 = 5,
    FLASH_PAGE_MAX = PAGE5
    //this can be extended
} FLASHAddrs;

static inline uint32_t FLASH_GET_PAGE_ADDR(FLASHAddrs page) {
    return FLASH_PAGE_END - (uint32_t)(page + 1) * FLASH_PAGE_SIZE;
}

int Flash_WriteType(void *data, uint8_t type_size, uint16_t len, FLASHAddrs addr);
int Flash_ReadType(void *data, uint8_t type_size, uint16_t len, FLASHAddrs addr);

// Generic
#ifdef __cplusplus
} /* End extern "C"*/

template <typename T>
static inline int Flash_Write(T *data, uint16_t len, FLASHAddrs addr) {
    return Flash_WriteType((void *)data, sizeof(T), len, addr);
}

template <typename T>
static inline int Flash_Read(T *data, uint16_t len, FLASHAddrs addr) {
    return Flash_ReadType((void *)data, sizeof(T), len, addr);
}

#else

#define  Flash_Write(data, len, addr) _Generic((data),\
    uint8_t *: Flash_WriteType(data, sizeof(uint8_t), len, addr),\
    int8_t *: Flash_WriteType(data, sizeof(int8_t), len, addr),\
    uint16_t *: Flash_WriteType(data, sizeof(uint16_t), len, addr),\
    int16_t *: Flash_WriteType(data, sizeof(int16_t), len, addr),\
    uint32_t *: Flash_WriteType(data, sizeof(uint32_t), len, addr),\
    int32_t *: Flash_WriteType(data, sizeof(int32_t), len, addr),\
    float *: Flash_WriteType(data, sizeof(float), len, addr),\
    uint64_t *: Flash_WriteType(data, sizeof(uint64_t), len, addr),\
    double *: Flash_WriteType(data, sizeof(double), len, addr)\
)

#define Flash_Read(data, len, addr) _Generic((data),\
    uint8_t *: Flash_ReadType(data, sizeof(uint8_t), len, addr),\
    int8_t *: Flash_ReadType(data, sizeof(int8_t), len, addr),\
    uint16_t *: Flash_ReadType(data, sizeof(uint16_t), len, addr),\
    int16_t *: Flash_ReadType(data, sizeof(int16_t), len, addr),\
    uint32_t *: Flash_ReadType(data, sizeof(uint32_t), len, addr),\
    int32_t *: Flash_ReadType(data, sizeof(int32_t), len, addr),\
    float *: Flash_ReadType(data, sizeof(float), len, addr),\
    uint64_t *: Flash_ReadType(data, sizeof(uint64_t), len, addr),\
    double *: Flash_ReadType(data, sizeof(double), len, addr)\
)
#endif /* __cplusplus */

#endif /* FLASH_H */