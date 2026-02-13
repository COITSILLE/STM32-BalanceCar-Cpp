#ifndef __MOTOR_ENCODER_H__
#define __MOTOR_ENCODER_H__ 

#include "main.h"
#include "dwt_ustime.h"
#include "smath.h"
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
struct MotorEncoder_InitParams{
    GPIO_TypeDef *PhraseA_GPIOx;
    uint16_t PhraseA_Pin;

    GPIO_TypeDef *PhraseB_GPIOx;
    uint16_t PhraseB_Pin;

    
} ;

class MotorEncoder {
public:
    MotorEncoder(float GEAR_RATIO, uint8_t UNIT_RING, uint8_t EDGE_PER_PERIOD);
    void init(MotorEncoder_InitParams init_params);
    float getAngVelocity();
    void irq_handler();
private:
    const float GEAR_RATIO;
    const float UNIT_RING;
    const float EDGE_PER_PERIOD;
    struct{
        GPIO_TypeDef *PhraseA_GPIOx;
        uint16_t PhraseA_Pin;

        GPIO_TypeDef *PhraseB_GPIOx;
        uint16_t PhraseB_Pin;
    } gpio_params;
    struct{
        DWT_Timestamp t0 = {0};
        DWT_Timestamp t1 = {UINT32_MAX, UINT32_MAX};

        uint32_t dt;
        int8_t dir;
    } time_params;
};
#endif /* __cplusplus */

#endif