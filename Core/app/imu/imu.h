#ifndef __IMU_H__
#define __IMU_H__


#include "dwt_ustime.h"
#include "smath.h"
#include "vector.h"
#include <stdint.h>

template<typename Sensor_i, typename _InitParams_>
class IMU_Base{
public:
    using Vec3_t = ::Vec3_t;
    void init(const _InitParams_& init_params){static_cast<Sensor_i*>(this)->init(init_params);};
    void readAccel(Vec3_t& accel_vec) {static_cast<Sensor_i*>(this)->readAccel(accel_vec);};
    void readGyro(Vec3_t& gyro_vec) {static_cast<Sensor_i*>(this)->readGyro(gyro_vec);};
    /**
     * @brief Get the Euler angles using complementary filter
     * @param accel_vec Accelerometer data
     * @param gyro_vec Gyroscope data
     * @param k filter coefficient (default 0.95)
     * @return Vec3_t Euler angles (roll, pitch, yaw)
     * @attention This function is expected to be run at a constant rate. It is recommended to use the data ready interrupt to call this function.
     */
    Vec3_t getEulerAngles(Vec3_t& accel_vec, Vec3_t& gyro_vec, float k = 0.95);
    /**
     * @brief Calculate calibration values for accelerometer and gyroscope
     * @param sample_times Number of samples to take for averaging
     * @param expected_g Expected gravity value (typically 9.8 m/s²)
     * @attention DO NOT run this function while the sensor is moving
     */
    void calibrate(float sample_times, float expected_g);
    /**
     * @brief Calibrate gyroscope Z-axis only
     * @param sample_times Number of samples to take for averaging
     */
    void calibrateZ(float sample_times);
    void setCalibration(const Vec3_t& accel_calibration, const Vec3_t& gyro_calibration);

    Vec3_t getCalibrationAccel() const { return this->calibration.accel; }
    Vec3_t getCalibrationGyro() const { return this->calibration.gyro; }
    //TODO： Magnetometer support
protected:
    timestamp_t last_time = 0;
    Vec3_t last_euler_angles = {0.0f, 0.0f, 0.0f};
    struct{
        Vec3_t accel = {0.0f, 0.0f, 0.0f};
        Vec3_t gyro = {0.0f, 0.0f, 0.0f};
    } calibration;
    struct{
        float g = 9.8f;
        float sampling_period = 0.0f;
    } params;
};

#ifndef __IMU_IPP__
#include "imu.ipp"
#endif

#endif /* __IMU_H__ */