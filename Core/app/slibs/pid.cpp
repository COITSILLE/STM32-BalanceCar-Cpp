#include "pid.h"
#include "dwt_ustime.h"
#include <stdint.h>

#ifdef __cplusplus
PID::PID() : 
    last_time(0),
    last_err(0.0f),
    last_err_i(0.0f),
    last_sp(0.0f),
    kp(0.0f),
    ki(0.0f),
    kd(0.0f),
    max_output(0.0f),
    min_output(0.0f),
    ki_limit(0.0f),
    sp(0.0f)
{}

#define assert_num(x) (x == x ? x : 0)
void PID::setTarget(float sp){
    this->sp = assert_num(sp);
}

void PID::setFactors(float kp, float ki, float kd){
    this->kp = assert_num(kp);
    this->ki = assert_num(ki);
    this->kd = assert_num(kd);
}

void PID::setLimit(float output_limit, float ki_limit){
    this->max_output = assert_num(output_limit);
    this->min_output = -assert_num(output_limit);
    this->ki_limit = assert_num(ki_limit);
}
void PID::setLimit(float max_output, float min_output, float ki_limit){
    this->max_output = assert_num(max_output);
    this->min_output = assert_num(min_output);
    this->ki_limit = assert_num(ki_limit);
}


float PID::getOutput(float fb, float tau){
    timestamp_t time_now = getTick();
    float dt = getDistance(last_time, time_now) * 1.0e-6;
    float err = this->sp - fb;

    float err_i = this->last_err_i + (err + this->last_err) * dt * 0.5;

    float d_err = err - this->last_err;
    float alpha = dt / (tau + dt);
    float err_d = alpha * d_err / dt + (1 - alpha) * this->last_err_d;
    this->last_err_d = err_d;
    
    if (err_i > this->ki_limit || err_i < -this->ki_limit){
        err_i = this->ki_limit * (err_i > 0 ? 1 : -1);
    }
    
    this->last_err = err;
    this->last_err_i = err_i;
    this->last_time = time_now;

    float res = this->kp * err + this->kd * err_d + this->ki * err_i;
    if (res > this->max_output){
        res = this->max_output;
    }
    else if (res < this->min_output){
        res = this->min_output;
    }
    return res;
}

float PID::getOutput_DiffAhead(float fb, float fb_d){
    timestamp_t time_now = getTick();
    float dt = getDistance(last_time, time_now) * 1.0e-6;
    float err = this->sp - fb;

    float err_i = this->last_err_i + (err + this->last_err) * dt * 0.5;
    
    if (err_i > this->ki_limit || err_i < -this->ki_limit){
        err_i = this->ki_limit * (err_i > 0 ? 1 : -1);
    }

    this->last_err = err;
    this->last_err_i = err_i;
    this->last_time = time_now;

    float res = this->kp * err - this->kd * fb_d + this->ki * err_i;
    if (res > this->max_output){
        res = this->max_output;
    }
    else if (res < this->min_output){
        res = this->min_output;
    }
    return res;
}

void PID::reset(){
    this->last_time = getTick();
    this->last_err = 0.0;
    this->last_err_i = 0.0;
    this->last_err_d = 0.0;
    this->last_sp = 0.0;
}
#endif