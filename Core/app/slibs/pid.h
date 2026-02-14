#ifndef PID_H
#define PID_H 

#include "main.h"
#include "dwt_ustime.h"
#include <array>
#include <stdint.h>

#ifdef __cplusplus
class PID{
private:
    DWT_Timestamp last_time;
    float last_err;
    float last_err_i;

    float _K_p_;
    float _K_i_;
    float _K_d_;
    float _MaxOutput_;
    float _MinOutput_;
    float _KiLimit_;
    float _sp_;
public:
    PID();
    void setFactors(float kp, float ki, float kd);
    void setLimit(float output_limit, float ki_limit);
    void setLimit(float max_output, float min_output, float ki_limit);
    void setTarget(float sp);
    /**
     *@param FB feedback, value from the sensor
     */
    float getOutput(float fb);
    /**
     * @brief Replace FB_d with the one passed in. Only useful when SP is 0 and doesn't change, and using
        the normal GetOutput function may cause differentiating an integrated value, which causes noise.
     * @attention Normally the err_d passed in, say "theta", since error equals SP - FB, should be "-theta"
     */
    float getOutput(float fb, float fb_d);
    void reset();
    
    float K_p() const { return _K_p_; };
    float K_i() const { return _K_i_; };
    float K_d() const { return _K_d_; };
    float MaxOutput() const { return _MaxOutput_; };
    float MinOutput() const { return _MinOutput_; };
    float KiLimit() const { return _KiLimit_; };
    float sp() const { return _sp_; };
};


#endif
#endif