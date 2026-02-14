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
    /**
     * @brief Initialize the encoder object
     * @param GEAR_RATIO Motor gear ratio
     * @param UNIT_RING Number of encoder units per ring
     * @param EDGE_PER_PERIOD Number of edges per period (typically 2 for both rising and falling edges)
     * @attention Designed for STM32F10x, only GPIO_Pin_0 ~ GPIO_PIN_5 are supported
     */
    MotorEncoder(float GEAR_RATIO, uint8_t UNIT_RING, uint8_t EDGE_PER_PERIOD);
    /**
     * @brief Initialize the encoder GPIO pins
     * @param init_params.PhraseA_GPIOx GPIO port for Phrase A
     * @param init_params.PhraseA_Pin GPIO pin for Phrase A (must be GPIO_Pin_0 ~ GPIO_Pin_5), for counting pulses and timing
     * @param init_params.PhraseB_GPIOx GPIO port for Phrase B
     * @param init_params.PhraseB_Pin GPIO pin for Phrase B, only for direction detection
     */
    void init(MotorEncoder_InitParams init_params);
    /**
     * @brief Get the angular velocity in rad/s
     */
    float getAngVelocity();
    /**
     * @brief Interrupt handler for the encoder signal. Should be called in the EXTI IRQHandler corresponding to PhraseA_Pin.
     */
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