#ifndef __TB6612_H__
#define __TB6612_H__
#include "main.h"
#include "pwm.h"
#include "motor_state.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C"{ 
#endif

#ifdef __cplusplus
}

struct TB6612_GPIOParams{
    GPIO_TypeDef *Ctrl1GPIOx;
    uint16_t Ctrl1Pin;
    GPIO_TypeDef *Ctrl2GPIOx;
    uint16_t Ctrl2Pin;

    GPIO_TypeDef *StbyGPIOx;
    uint16_t StbyPin;
};

class TB6612_Motor{
private:
    TB6612_GPIOParams gpio_params;
    PWM pwm;
public:
    TB6612_Motor(PWM pwm) : pwm(pwm){};
    void init(TB6612_GPIOParams init_params);
    void stby(bool state);
    void setControl(MOTOR_STATE state);
    void setDuty(float duty);
};
#endif

#endif 