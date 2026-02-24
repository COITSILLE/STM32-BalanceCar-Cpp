#ifndef PID_H
#define PID_H 

#include "main.h"
#include "dwt_ustime.h"
#include <stdint.h>

#ifdef __cplusplus
class PID{
private:
    timestamp_t last_time;
    float last_err;
    float last_err_i;
    float last_sp;
    float last_err_d;

public:
    float kp;
    float ki;
    float kd;
    float max_output;
    float min_output;
    float ki_limit;
    float sp;

    PID();
    void setFactors(float kp, float ki, float kd);
    void setLimit(float output_limit, float ki_limit);
    void setLimit(float max_output, float min_output, float ki_limit);
    void setTarget(float sp);
    float getOutput(float fb);
    /**
     *@param fb feedback
     *@param tau time constant for the low-pass filter applied to the derivative term, in seconds.
     */
    float getOutput_IncompDiff(float fb, float tau = 0);
    /**
     * @brief only take the derivative of the feedback, which is useful when the setpoint is constant (e.g., 0) and differentiating the error may cause noise due to the integral term.
     */
    float getOutput_DiffAhead(float fb);
    /**
     * @brief Replace FB_d with the one passed in. Only useful when SP is 0 and doesn't change, and using
        the normal GetOutput function may cause differentiating an integrated value, which causes noise.
     */
    float getOutput_DiffAhead(float fb, float fb_d);
    void reset();
};


#endif
#endif