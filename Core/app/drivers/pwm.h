#ifndef PWM_H
#define PWM_H 

#include "main.h"
#include "smath.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}

class PWM{
private:
    const int APB1_FREQ;
    const int UNIT_FREQ;
    TIM_HandleTypeDef *PWM_TIM;
    int Channel;
public:
    /**
     * @brief Constructor for the PWM class
     * @param PWMTIM Pointer to the TIM_HandleTypeDef structure for the PWM timer
     * @param Channel The channel number (1-4) of the PWM signal
     * @param UNIT_FREQ The unit frequency in Hz (default is 4000000), which means the timer counts up at 4 MHz. This value is used to calculate the prescaler and ARR register value. It should be chosen based on the desired resolution and the maximum PWM frequency. For example, with a UNIT_FREQ of 4 MHz and a pwm_freq of 20 kHz, the timer will count up to 200 (4000000/20000) for each PWM period, giving a resolution of 0.5% (1/200).
     * @param APB1_FREQ The APB1 clock frequency in Hz (default is 72000000)
     */
    PWM(TIM_HandleTypeDef *PWMTIM, int Channel, int UNIT_FREQ = 4000000, int APB1_FREQ = 72000000);
    /**
     * @brief Initialize the PWM with the specified frequency and start the PWM signal generation
     * @param pwm_freq Desired PWM frequency in Hz
     * @attention This function will automatically calculate the prescaler and ARR register value. Noting that this will affect the resolution of the PWM signal, so both pwm_freq and UNITFREQ should be chosen wisely.
     */
    void init(int pwm_freq);
    void SetDuty(float duty_ratio);
};
#endif
#endif /* PWM_H */