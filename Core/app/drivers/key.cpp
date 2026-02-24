#include "key.h"

static const uint8_t DEBOUNCE_DURATION = 20U;

Key::Key(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin, 
            GPIO_PinState valid_pin_state, KeyEdgeType edge_type, void (*KeyCallback)(void))
: GPIOx(GPIOx), GPIO_Pin(GPIO_Pin), valid_pin_state(valid_pin_state), edge_type(edge_type), KeyCallback(KeyCallback)
{
    this->state = KEY_RELEASED;
    this->debounce_time = 0;
}
void Key::proc(){
    GPIO_PinState state = HAL_GPIO_ReadPin(this->GPIOx, this->GPIO_Pin);
    if (state == this->valid_pin_state) {
        switch (this->state) {
            case KEY_WAITRELEASED:
                break;
            case KEY_RELEASED:
                this->state = KEY_PDETECT;
                this->debounce_time = HAL_GetTick();
                break;
            case KEY_PDETECT:
                if (HAL_GetTick() - this->debounce_time > DEBOUNCE_DURATION) {
                    this->state = KEY_WAITRELEASED;
                    if (this->KeyCallback != NULL && (this->edge_type == RISING_EDGE || this->edge_type == BOTH_EDGE)) this->KeyCallback();
                }
                break;
            case KEY_RDETECT:
                this->state = KEY_WAITRELEASED;
                break;
            default:
                break;
            }
    }
    else{
        switch (this->state) {
            case KEY_RELEASED:
                break;
            case KEY_WAITRELEASED:
                this->state = KEY_RDETECT;
                this->debounce_time = HAL_GetTick();
                break;
            case KEY_RDETECT:
                if (HAL_GetTick() - this->debounce_time > DEBOUNCE_DURATION) {
                    this->state = KEY_RELEASED;
                    if (this->KeyCallback != NULL && (this->edge_type == FALLING_EDGE || this->edge_type == BOTH_EDGE)) this->KeyCallback();
                }
                break;
            case KEY_PDETECT:
                this->state = KEY_RELEASED;
                break;
            default:
                break;
            }
    }
}