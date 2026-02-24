#include "encoder.h"

MotorEncoder::MotorEncoder(float GEAR_RATIO, uint8_t UNIT_RING, uint8_t EDGE_PER_PERIOD) : 
GEAR_RATIO(GEAR_RATIO), UNIT_RING(UNIT_RING), EDGE_PER_PERIOD(EDGE_PER_PERIOD)
{
}
void MotorEncoder::init(MotorEncoder_InitParams init_params)
{
    gpio_params = {init_params.PhraseA_GPIOx, init_params.PhraseA_Pin,
                   init_params.PhraseB_GPIOx, init_params.PhraseB_Pin};
}


void MotorEncoder::irq_handler(){
    this->time_params.t0 = this->time_params.t1;
    this->time_params.t1 = getTick();
    if (HAL_GPIO_ReadPin(this->gpio_params.PhraseA_GPIOx, this->gpio_params.PhraseA_Pin) 
        == HAL_GPIO_ReadPin(this->gpio_params.PhraseB_GPIOx, this->gpio_params.PhraseB_Pin)){
        if (this->time_params.dir > 0){
            this->time_params.dir = 0;
        }
        else {
            this->time_params.dir = -1;
        }
        
    }
    else {
        if (this->time_params.dir < 0){
            this->time_params.dir = 0;
        }
        else {
            this->time_params.dir = 1;
        }
    }

}

float MotorEncoder::getAngVelocity(){
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    int8_t dir_cpy = this->time_params.dir;
    timestamp_t t0_cpy = this->time_params.t0;
    timestamp_t t1_cpy = this->time_params.t1;
    __enable_irq();
    __set_PRIMASK(primask);

    uint32_t dt = MAX(getDistance(t0_cpy, t1_cpy), getDistance(t1_cpy, getTick()));

    return dir_cpy / (dt * 1.0e-6f) * (2 * PI / (UNIT_RING * EDGE_PER_PERIOD)) / GEAR_RATIO;
}

//DO change the 2 IRQ function's names according to the actual GPIO_Pins of PhraseA you are using!

