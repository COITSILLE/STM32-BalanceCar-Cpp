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
    float alpha_add;
    
    volatile float alpha_cpy;

    volatile float last_value;
public:
    AdaptiveFirstOrderFilter(float alpha, float tolerance, float alpha_add)
        : alpha(alpha), tolerance(tolerance), alpha_add(alpha_add), alpha_cpy(alpha), last_value(0){};
    float get(float value);
};
float FirstOrderFilter_Get(FirstOrderFilter *filter, float value);
void FirstOrderFilter_Init(FirstOrderFilter *filter, float alpha);
float AdaptiveFirstOrderFilter_Get(AdaptiveFirstOrderFilter *filter, float value);
void AdaptiveFirstOrderFilter_Init(AdaptiveFirstOrderFilter *filter, float alpha, float tolerance, float alpha_add);

#endif /* __cplusplus */
#endif /* FILTER_H */