#ifndef __TB6612_H__
#define __TB6612_H__
#include "main.h"
#include "pwm.h"
#include "motor_state.h"
#include <stdint.h>

struct TB6612GPIOParams{
    GPIO_TypeDef *Ctrl1GPIOx;
    uint16_t Ctrl1Pin;
    GPIO_TypeDef *Ctrl2GPIOx;
    uint16_t Ctrl2Pin;

    GPIO_TypeDef *StbyGPIOx;
    uint16_t StbyPin;
};

class TB6612_Motor{
private:
    TB6612GPIOParams gpio_params;
    PWM pwm;
public:
    TB6612_Motor(PWM pwm) : pwm(pwm){};
    void init(TB6612GPIOParams init_params);
    void stby(bool state);
    void setControl(MotorState state);
    void setDuty(float duty);
};
#endif