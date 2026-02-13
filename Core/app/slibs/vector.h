#ifndef __VECTOR_H__
#define __VECTOR_H__

#ifdef __cplusplus
extern "C" {
#endif
#include <stdint.h>
typedef struct{
    float x;
    float y;
} Vec2_t;

typedef struct {
    float x;
    float y;
    float z;
} Vec3_t;
#ifdef __cplusplus
}
static inline Vec2_t operator+(const Vec2_t& a, const Vec2_t& b){
    return {a.x + b.x, a.y + b.y};
}
static inline Vec3_t operator+(const Vec3_t& a, const Vec3_t& b){
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}
static inline Vec2_t operator-(const Vec2_t& a, const Vec2_t& b){
    return {a.x - b.x, a.y - b.y};
}
static inline Vec3_t operator-(const Vec3_t& a, const Vec3_t& b){
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}
#else
static inline Vec2_t vec2_add(const Vec2_t* a, const Vec2_t* b){
    return (Vec2_t){a->x + b->x, a->y + b->y};
}
static inline Vec3_t vec3_add(const Vec3_t* a, const Vec3_t* b){
    return (Vec3_t){a->x + b->x, a->y + b->y, a->z + b->z};
}
static inline Vec2_t vec2_sub(const Vec2_t* a, const Vec2_t* b){
    return (Vec2_t){a->x - b->x, a->y - b->y};
}
static inline Vec3_t vec3_sub(const Vec3_t* a, const Vec3_t* b){
    return (Vec3_t){a->x - b->x, a->y - b->y, a->z - b->z};
}
#endif /* __cplusplus */


#endif /*__VECTOR_H__*/ 