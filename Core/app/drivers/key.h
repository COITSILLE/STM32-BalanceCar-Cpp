#ifndef KEY_H
#define KEY_H
#include "main.h"
#include <stdint.h>

typedef enum{
    KEY_PDETECT = 2U,
    KEY_RDETECT = 3U,
    KEY_WAITRELEASED = 4U,
    KEY_RELEASED = 5U
} KeyState;

typedef enum{
    RISING_EDGE,
    FALLING_EDGE,
    BOTH_EDGE
} KeyEdgeType;

#ifdef __cplusplus
// C++ 代码放这里
class Key{
private:
    KeyState state;
    uint32_t debounce_time;
    GPIO_TypeDef *GPIOx;
    uint16_t GPIO_Pin;
    GPIO_PinState valid_pin_state;
    KeyEdgeType edge_type;
    void (*KeyCallback)(void);
public:
    Key(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, GPIO_PinState valid_pin_state = GPIO_PIN_RESET,
        KeyEdgeType edge_type = RISING_EDGE, void (*KeyCallback)(void) = NULL);
    void proc();
};
#endif

#endif