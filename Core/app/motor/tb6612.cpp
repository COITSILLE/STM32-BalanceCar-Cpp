#include "tb6612.h"

#ifdef __cplusplus
void TB6612_Motor::init(TB6612GPIOParams init_params){
    this->gpio_params = init_params;
}

void TB6612_Motor::stby(bool state){
    HAL_GPIO_WritePin(this->gpio_params.StbyGPIOx, this->gpio_params.StbyPin, (state ? GPIO_PIN_SET : GPIO_PIN_RESET));
}

void TB6612_Motor::setDuty(float duty){
    float _duty_;
    if (duty < 0) _duty_ = 0;
    else if (duty > 1) _duty_ = 1;
    else  _duty_ = duty;
    
    this->pwm.SetDuty(_duty_);
}

void TB6612_Motor::setControl(MotorState state){
    switch (state) {
        case BRAKE:
            HAL_GPIO_WritePin(this->gpio_params.Ctrl1GPIOx, this->gpio_params.Ctrl1Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(this->gpio_params.Ctrl2GPIOx, this->gpio_params.Ctrl2Pin, GPIO_PIN_SET);
            break;
        case FORWARD:
            HAL_GPIO_WritePin(this->gpio_params.Ctrl1GPIOx, this->gpio_params.Ctrl1Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(this->gpio_params.Ctrl2GPIOx, this->gpio_params.Ctrl2Pin, GPIO_PIN_RESET);
            break;
        case BACKWARD:
            HAL_GPIO_WritePin(this->gpio_params.Ctrl1GPIOx, this->gpio_params.Ctrl1Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(this->gpio_params.Ctrl2GPIOx, this->gpio_params.Ctrl2Pin, GPIO_PIN_SET);
            break;
        case SLIDE:
            HAL_GPIO_WritePin(this->gpio_params.Ctrl1GPIOx, this->gpio_params.Ctrl1Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(this->gpio_params.Ctrl2GPIOx, this->gpio_params.Ctrl2Pin, GPIO_PIN_RESET);
            break;
    }
}

#endif