#include "tb6612.h"

#ifdef __cplusplus
void TB6612_Motor::init(TB6612_GPIOParams init_params){
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    this->gpio_params = init_params;

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    HAL_GPIO_WritePin(this->gpio_params.Ctrl1GPIOx, this->gpio_params.Ctrl1Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(this->gpio_params.Ctrl2GPIOx, this->gpio_params.Ctrl2Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(this->gpio_params.StbyGPIOx, this->gpio_params.StbyPin, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin = this->gpio_params.Ctrl1Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(this->gpio_params.Ctrl1GPIOx, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = this->gpio_params.Ctrl2Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(this->gpio_params.Ctrl2GPIOx, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = this->gpio_params.StbyPin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(this->gpio_params.StbyGPIOx, &GPIO_InitStruct);
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

void TB6612_Motor::setControl(MOTOR_STATE state){
    switch (state) {
        case MOTOR_BRAKE:
            HAL_GPIO_WritePin(this->gpio_params.Ctrl1GPIOx, this->gpio_params.Ctrl1Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(this->gpio_params.Ctrl2GPIOx, this->gpio_params.Ctrl2Pin, GPIO_PIN_SET);
            break;
        case MOTOR_FORWARD:
            HAL_GPIO_WritePin(this->gpio_params.Ctrl1GPIOx, this->gpio_params.Ctrl1Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(this->gpio_params.Ctrl2GPIOx, this->gpio_params.Ctrl2Pin, GPIO_PIN_RESET);
            break;
        case MOTOR_BACKWARD:
            HAL_GPIO_WritePin(this->gpio_params.Ctrl1GPIOx, this->gpio_params.Ctrl1Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(this->gpio_params.Ctrl2GPIOx, this->gpio_params.Ctrl2Pin, GPIO_PIN_SET);
            break;
        case MOTOR_SLIDE:
            HAL_GPIO_WritePin(this->gpio_params.Ctrl1GPIOx, this->gpio_params.Ctrl1Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(this->gpio_params.Ctrl2GPIOx, this->gpio_params.Ctrl2Pin, GPIO_PIN_RESET);
            break;
    }
}

#endif