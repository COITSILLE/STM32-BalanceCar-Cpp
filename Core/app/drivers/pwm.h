#ifndef PWM_H
#define PWM_H 

#include "main.h"
#include "tim.h"

#ifdef __cplusplus
extern "C" {
#endif


typedef struct{
    TIM_HandleTypeDef *PWM_TIM;
    int Channel;
}PWM_Typedef;

void PWM_FreqInit(TIM_HandleTypeDef *PWMTIM, int pwm_freq);
void PWM_InstanceInit(PWM_Typedef *pwm, TIM_HandleTypeDef *PWM_TIM, int Channel);
void PWM_SetDuty(PWM_Typedef *pwm, float duty_ratio);

#ifdef __cplusplus
}
class PWM{
private:
    const int APB1_FREQ;
    const int UNIT_FREQ;
    TIM_HandleTypeDef *PWM_TIM;
    int Channel;
public:
    PWM(TIM_HandleTypeDef *PWMTIM, int Channel, int UNIT_FREQ = 1000000, int APB1_FREQ = 72000000);
    void init(int pwm_freq);
    void SetDuty(float duty_ratio);
};
#endif
#endif /* PWM_H */