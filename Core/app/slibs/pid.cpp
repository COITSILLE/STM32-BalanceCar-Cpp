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
    this->_SP_ = 0.0;
}

#define assert_num(x) x == x ? x : 0
void PID::setSP(float sp){
    this->_SP_ = assert_num(sp);
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

/**
 *@param FB value from the sensor
 *@param CO Output of PID
*/
float PID::getCO(float FB){
    DWT_Timestamp time_now = getTick();
    float dt = getDistance(last_time, time_now) * 1.0e-6;
    float err = this->_SP_ - FB;

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

/**
 * @brief Replace the error_d with the one passed in. Only useful when SP is 0 and doesn't change, and using
        the normal GetCO function may cause differentiating an integrated value, which causes noise.
 * @attention Normally the err_d passed in, say "theta", since error equals SP - FB, should be "-theta"
*/
float PID::getCO(float FB, float FB_d){
    DWT_Timestamp time_now = getTick();
    float dt = getDistance(last_time, time_now) * 1.0e-6;
    float err = this->_SP_ - FB;

    static float last_SP_d = 0.0;
    float SP_d = this->_SP_ - last_SP_d / dt;
    last_SP_d = this->_SP_;

    float err_d = SP_d - FB_d;
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