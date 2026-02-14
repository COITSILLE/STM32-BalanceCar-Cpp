#ifndef FILTER_H
#define FILTER_H

#include "main.h"
#include "smath.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
class FirstOrderFilter{
private:
    float alpha;
    volatile float last_value;
public:
    FirstOrderFilter(float alpha) : alpha(alpha), last_value(0){};
    float get(float value);
};
class AdaptiveFirstOrderFilter{
private:
    float alpha;
    float tolerance;
    float alpha_increment;
    
    volatile float adaptive_alpha;

    volatile float last_value;
public:
    AdaptiveFirstOrderFilter(float alpha, float tolerance, float alphaIncrement)
        : alpha(alpha), tolerance(tolerance), alpha_increment(alphaIncrement), adaptive_alpha(alpha), last_value(0){};
    float get(float value);
};

#endif /* __cplusplus */
#endif /* FILTER_H */