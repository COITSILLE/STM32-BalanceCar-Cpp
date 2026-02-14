/** 
 * @brief fast math functions for float type, using some approximations and look-up tables.
*/
#include "smath.h"

float s_inv_sqrt(float x){
    long i;  
    float x2, y;  
    const float threehalfs = 1.5F;  
    
    x2 = x * 0.5F;  
    y = x;  
    i = * ( long * ) &y;                       // evil floating point bit level hacking  
    i = 0x5f3759df - ( i >> 1 );               // what the fuck?  
    y = * ( float * ) &i;  
    y = y * ( threehalfs - ( x2 * y * y ) );   // 1st iteration  
    y = y * ( threehalfs - ( x2 * y * y ) );   // 2nd iteration, this can be removed  
 
    return y;  
}

float s_sqrt(float x){
    return x * s_inv_sqrt(x);
}

float s_sin(float x){
    const float sin_table_rad[] = {
    0.0f,                                   // sin(0)
    0.09983341664682815230681419841062f,    // sin(0.1)
    0.19866933079506121545941262711889f,    // sin(0.2)
    0.29552020666133957510532074568503f,    // sin(0.3)
    0.38941834230865049166631175679570f,    // sin(0.4)
    0.47942553860420300027328793521557f,    // sin(0.5)
    0.56464247339503535720094544565866f,    // sin(0.6)
    0.64421768723769105367261435139872f,    // sin(0.7)
    0.71735609089952276162717461058139f,    // sin(0.8)
    0.78332690962748338846138231573655f,    // sin(0.9)
    0.84147098480789650665250232163030f,    // sin(1.0)
    0.89120736006143533995180257787172f,    // sin(1.1)
    0.93203908596722634967013443549482f,    // sin(1.2)
    0.96355818541719296470032086941907f,    // sin(1.3)
    0.98544972998846018065947445622696f,    // sin(1.4)
    0.99749498660405443094172337114149f,    // sin(1.5)
    0.99957360304150516405189620723992f,    // sin(1.6)
    };
    if (x < 0){
        while(x < 0) x += TWO_PI;
    }
    else{
        while(x > TWO_PI) x -= TWO_PI;
    }
    int8_t sign = 1;
    if (x > PI){
        sign = -1;
        x -= PI;
    }
    x = x > HALF_PI? (PI - x) : x;

    float idx_float = x * 10.0f;
    int idx_low = (int)idx_float;
    int idx_high = idx_low + 1;
    
    if (idx_high >= 17) {
        return sign * 1.0f;  // sin(π/2) = 1
    }
    
    float t = idx_float - idx_low;
    
    // 线性插值
    float y = sin_table_rad[idx_low] * (1.0f - t) + sin_table_rad[idx_high] * t;
    
    return sign * y;
}

float s_cos(float x){
    return s_sin(x + HALF_PI);
}

float s_tan(float x){
    return s_sin(x) / s_cos(x);
}

float s_atan(float x){
    if (x > 1.0f) {
        return HALF_PI - s_atan(1.0f / x);
    } else if (x < -1.0f) {
        return -HALF_PI - s_atan(1.0f / x);
    }
    return x * QUART_PI + 0.273082f * x * (1.0f - ABS(x));
}

float s_atan2(float y, float x){
    float ratio = y / x; 
    float result;
    
    if (x > 0) {
        result = s_atan(ratio);
    } 
    else if (x < 0) {
        result = s_atan(ratio);
        result += (y >= 0) ? PI : -PI;
    } 
    else {  // x == 0
        result = (y > 0) ? HALF_PI : (y < 0) ? -HALF_PI : 0.0f;
    }
    return result;
}
