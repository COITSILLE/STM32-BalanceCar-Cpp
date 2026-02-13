#include "pwm.h"

#ifdef __cplusplus
PWM::PWM(TIM_HandleTypeDef *PWMTIM, int Channel, int UNIT_FREQ, int APB1_FREQ)
: APB1_FREQ(APB1_FREQ),
  UNIT_FREQ(UNIT_FREQ),
  PWM_TIM(PWMTIM),
  Channel(Channel)
{

}
void PWM::init(int pwm_freq){
    if (pwm_freq <= 0 || pwm_freq > this->UNIT_FREQ || this->APB1_FREQ % this->UNIT_FREQ != 0){
        return;
    }
    __HAL_TIM_SET_PRESCALER(this->PWM_TIM, (this->APB1_FREQ / this->UNIT_FREQ) - 1);
    __HAL_TIM_SET_AUTORELOAD(this->PWM_TIM, this->UNIT_FREQ / pwm_freq - 1);

    HAL_TIM_PWM_Start(this->PWM_TIM, this->Channel);
    __HAL_TIM_SET_COMPARE(this->PWM_TIM, this->Channel, 0);
}
void PWM::SetDuty(float duty){
    __HAL_TIM_SET_COMPARE(this->PWM_TIM, this->Channel, (duty * __HAL_TIM_GET_AUTORELOAD(this->PWM_TIM)));
}

#endif

