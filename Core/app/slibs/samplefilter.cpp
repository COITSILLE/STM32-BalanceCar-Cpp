#include "samplefilter.h"

float FirstOrderFilter::get(float value){
    this->last_value = this->alpha * value + (1.0 - this->alpha) * this->last_value;

    return this->last_value;
}


float AdaptiveFirstOrderFilter::get(float value){
    
    if (ABS(value - this->last_value) > this->tolerance && this->alpha < 1){
        this->alpha_cpy = this->alpha + this->alpha_add;
    }
    else {
        this->alpha_cpy = this->alpha;
    }
    this->last_value = this->alpha_cpy * value + (1.0 - this->alpha_cpy) * this->last_value;
    return this->last_value;
}
