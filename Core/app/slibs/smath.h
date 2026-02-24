#ifndef SMATH_H
#define SMATH_H

#include <stdint.h>

#ifdef __FPU_PRESENT
#include <math.h>
#endif

#define PI 3.14159265358979323846f
#define TWO_PI 6.28318530717958647692f
#define HALF_PI 1.57079632679489661923f
#define QUART_PI 0.785398163397448309616f
#define INV_PI 0.318309886183790671538f
#define INV_TWO_PI 0.159154943091895335769f
#define RAD_TO_DEG 57.2957795130823208768f
#define DEG_TO_RAD 0.01745329251994329577f

#define ABS(x) ((x) >= 0 ? (x) : -(x))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define ROUND(x) ((x) >= 0 ? (int)((x) + 0.5f) : (int)((x) - 0.5f))

#ifdef __cplusplus
extern "C" {
#endif

float s_inv_sqrt(float x);
float s_sqrt(float x);
float s_sin(float x);
float s_cos(float x);
float s_tan(float x);
float s_atan(float x);
float s_atan2(float y, float x);

#ifdef __cplusplus
}
#endif


#endif