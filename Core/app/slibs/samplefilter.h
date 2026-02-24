#ifndef FILTER_H
#define FILTER_H

#include "main.h"
#include "smath.h"
#include <stdint.h>
class FirstOrderFilter{
private:
    float alpha;
    float last_value;
public:
    FirstOrderFilter(float alpha) : alpha(alpha), last_value(0){};
    float get(float value);
};
class AdaptiveFirstOrderFilter{
private:
    float alpha;
    float tolerance;
    float alpha_increment;
    
    float adaptive_alpha;

    float last_value;
public:
    AdaptiveFirstOrderFilter(float alpha, float tolerance, float alphaIncrement)
        : alpha(alpha), tolerance(tolerance), alpha_increment(alphaIncrement), adaptive_alpha(alpha), last_value(0){};
    float get(float value);
};

#endif /* FILTER_H */