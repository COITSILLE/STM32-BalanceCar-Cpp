#include "samplefilter.h"

float FirstOrderFilter::get(float value){
    this->last_value = this->alpha * value + (1.0 - this->alpha) * this->last_value;

    return this->last_value;
}


float AdaptiveFirstOrderFilter::get(float value){
    if (ABS(value - this->last_value) > this->tolerance && this->adaptive_alpha < 1){
        this->adaptive_alpha = this->alpha + this->alpha_increment;
    }
    else {
        this->adaptive_alpha = this->alpha;
    }
    this->last_value = this->adaptive_alpha * value + (1.0 - this->adaptive_alpha) * this->last_value;
    return this->last_value;
}
