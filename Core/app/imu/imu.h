#ifndef __IMU_H__
#define __IMU_H__


#ifdef __cplusplus
extern "C" {

#include "dwt_ustime.h"
#include "smath.h"
#include "vector.h"
#include <stdint.h>

#endif

#ifdef __cplusplus
}

template<typename Sensor_i, typename _InitParams_>
class IMU_Base{
public:
    using Vec3_t = ::Vec3_t;
    void init(_InitParams_ params){static_cast<Sensor_i*>(this)->init(params);};
    void readAccel(Vec3_t& accel_vec) {static_cast<Sensor_i*>(this)->readAccel(accel_vec);};
    void readGyro(Vec3_t& gyro_vec) {static_cast<Sensor_i*>(this)->readGyro(gyro_vec);};
    void readAccelGyro(Vec3_t& accel_vec, Vec3_t& gyro_vec) {static_cast<Sensor_i*>(this)->readAccelGyro(accel_vec, gyro_vec);};
    Vec3_t getEulerAngles(Vec3_t& accel_vec, Vec3_t& gyro_vec, float k = 0.95);
    void getOffset(float sample_times, float expected_g);
    void setOffset(Vec3_t accel_offset, Vec3_t gyro_offset);
    Vec3_t getOffsetAccel() const { return this->offset.accel; }
    Vec3_t getOffsetGyro() const { return this->offset.gyro; }
    //TODO： Magnetometer support
protected:
    Vec3_t last_euler_angles = {0.0f, 0.0f, 0.0f};
    struct{
        Vec3_t accel;
        Vec3_t gyro;
    } offset;
    struct{
        float g = 9.8f;
        float call_period;
    } params;
};

#ifndef __IMU_IPP__
#include "imu.ipp"
#endif

#endif /* __cplusplus */

#endif /* __IMU_H__ */