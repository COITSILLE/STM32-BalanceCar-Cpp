#include "pid.h"
#include "dwt_ustime.h"
#include <stdint.h>

#ifdef __cplusplus
PID::PID(){
    this->_K_p_ = 0.0;
    this->_K_i_ = 0.0;
    this->_K_d_ = 0.0;
    this->_MaxOutput_ = 0.0;
    this->_MinOutput_ = 0.0;
    this->_KiLimit_ = 0.0;
    this->_sp_ = 0.0;
}

#define assert_num(x) x == x ? x : 0
void PID::setTarget(float sp){
    this->_sp_ = assert_num(sp);
}

void PID::setFactors(float kp, float ki, float kd){
    this->_K_p_ = assert_num(kp);
    this->_K_i_ = assert_num(ki);
    this->_K_d_ = assert_num(kd);
}

void PID::setLimit(float output_limit, float ki_limit){
    this->_MaxOutput_ = assert_num(output_limit);
    this->_MinOutput_ = -assert_num(output_limit);
    this->_KiLimit_ = assert_num(ki_limit);
}
void PID::setLimit(float max_output, float min_output, float ki_limit){
    this->_MaxOutput_ = assert_num(max_output);
    this->_MinOutput_ = assert_num(min_output);
    this->_KiLimit_ = assert_num(ki_limit);
}


float PID::getOutput(float fb){
    DWT_Timestamp time_now = getTick();
    float dt = getDistance(last_time, time_now) * 1.0e-6;
    float err = this->_sp_ - fb;

    float err_i = this->last_err_i + (err + this->last_err) * dt * 0.5;
    float err_d = (err - this->last_err) / dt;
    
    if (err_i > this->_KiLimit_ || err_i < -this->_KiLimit_){
        err_i = this->_KiLimit_ * (err_i > 0 ? 1 : -1);
    }
    
    this->last_err = err;
    this->last_err_i = err_i;
    this->last_time = time_now;

    float result = this->_K_p_ * err + this->_K_d_ * err_d + this->_K_i_ * err_i;
    if (result > this->_MaxOutput_){
        result = this->_MaxOutput_;
    }
    else if (result < this->_MinOutput_){
        result = this->_MinOutput_;
    }
    return result;
}

float PID::getOutput(float fb, float fb_d){
    DWT_Timestamp time_now = getTick();
    float dt = getDistance(last_time, time_now) * 1.0e-6;
    float err = this->_sp_ - fb;

    static float last_SP_d = 0.0;
    float SP_d = this->_sp_ - last_SP_d / dt;
    last_SP_d = this->_sp_;

    float err_d = SP_d - fb_d;
    float err_i = this->last_err_i + (err + this->last_err) * dt * 0.5;
    
    if (err_i > this->_KiLimit_ || err_i < -this->_KiLimit_){
        err_i = this->_KiLimit_ * (err_i > 0 ? 1 : -1);
    }

    this->last_err = err;
    this->last_err_i = err_i;
    this->last_time = time_now;

    float result = this->_K_p_ * err + this->_K_d_ * err_d + this->_K_i_ * err_i;
    if (result > this->_MaxOutput_){
        result = this->_MaxOutput_;
    }
    else if (result < this->_MinOutput_){
        result = this->_MinOutput_;
    }
    return result;
}

void PID::reset(){
    this->last_time = getTick();
    this->last_err = 0.0;
    this->last_err_i = 0.0;
}
#endif