#include "flash.h"

int Flash_WriteType(void *data, uint8_t type_size, uint16_t len, FlashAddrs addr){
    if (addr > FLASH_PAGE_MAX || data == NULL || len == 0)
        return -1; 

    uint32_t page_addr = FLASH_GET_PAGE_ADDR(addr);

    HAL_FLASH_Unlock();
    __disable_irq();

    uint32_t error_addr;
    FLASH_EraseInitTypeDef FlashErase;
    FlashErase.TypeErase = FLASH_TYPEERASE_PAGES;
    FlashErase.PageAddress = page_addr;
    FlashErase.NbPages = 1;
    if (HAL_FLASHEx_Erase(&FlashErase, &error_addr) != HAL_OK) {
        HAL_FLASH_Lock();
        __enable_irq();
        return error_addr;
    }

    uint8_t *pDataI = (uint8_t *)data;
    uint32_t PROGRAM_LEN = 0;
    uint8_t read_size = 0;
    switch (type_size) {
        case sizeof(uint8_t):
        case sizeof(uint16_t):
            PROGRAM_LEN = FLASH_TYPEPROGRAM_HALFWORD;
            read_size = sizeof(uint16_t);
            break;
        case sizeof(uint32_t):
            PROGRAM_LEN = FLASH_TYPEPROGRAM_WORD;
            read_size = type_size;
            break;
        case sizeof(uint64_t):
            PROGRAM_LEN = FLASH_TYPEPROGRAM_DOUBLEWORD;
            read_size = type_size;
            break;
        default:
            HAL_FLASH_Lock();
            __enable_irq();
            return -1;
    }

    for (uint16_t i = 0; i < len; i++){
        uint64_t dataF = 0;
        for (uint8_t j = 0; j < type_size; j++){
            dataF |= (uint64_t)pDataI[j + i * type_size] << (8 * j);
        }
        if (HAL_FLASH_Program(PROGRAM_LEN, page_addr + (i * read_size), dataF) != HAL_OK) {
            HAL_FLASH_Lock();
            __enable_irq();
            return -1;
        }
    } 

    HAL_FLASH_Lock();
    __enable_irq();
    return 0;
}
int Flash_ReadType(void *data, uint8_t type_size, uint16_t len, FlashAddrs addr){
    if (addr > FLASH_PAGE_MAX || data == NULL || len == 0)
        return -1;

    uint32_t page_addr = FLASH_GET_PAGE_ADDR(addr);

    uint8_t *pDataI = (uint8_t *)data;
    uint8_t read_size = type_size == sizeof(uint8_t) ? sizeof(uint16_t) : type_size;
    for (uint16_t i = 0; i < len; i++){
        for (uint8_t j = 0; j < type_size; j++){
            pDataI[j + i * type_size] = *(uint8_t *)(page_addr + i * read_size + j);
        } 
    }
    return 0;
}