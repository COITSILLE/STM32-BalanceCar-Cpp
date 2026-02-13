#ifndef __IMU_IPP__
#define __IMU_IPP__

#include "dwt_ustime.h"
#include "vector.h"
#ifdef __cplusplus
#include "imu.h"
/**
 * @brief  Get the Euler angles
 * @attention This function is expected to be ran at a constant rate. It is recommened to use the data ready 
             interrupt to call this function.
*/
template<typename Sensor_i, typename _InitParams_>
typename IMU_Base<Sensor_i, _InitParams_>::Vec3_t IMU_Base<Sensor_i, _InitParams_>::getEulerAngles(Vec3_t& accel_vec, Vec3_t& gyro_vec, float k){
    static DWT_Timestamp last_time = getTick();
    DWT_Timestamp time_now = getTick();
    float dt = getDistance(last_time, time_now) * 1.0e-6;
    //From Accel
    //float Inv_Pi = 180.0 / M_PI;
    float roll_accel = s_atan2(accel_vec.y, accel_vec.z);
    float pitch_accel = - s_atan2(accel_vec.x, s_sqrt(accel_vec.z * accel_vec.z + accel_vec.y * accel_vec.y));
    //From Gyro 
    float roll_gyro = this->last_euler_angles.x + gyro_vec.x * dt;
    float pitch_gyro = this->last_euler_angles.y + gyro_vec.y * dt;
    float yaw_gyro = this->last_euler_angles.z + gyro_vec.z * dt;

    this->last_euler_angles.x = k * roll_gyro + (1 - k) * roll_accel;
    this->last_euler_angles.y = k * pitch_gyro + (1 - k) * pitch_accel;
    this->last_euler_angles.z = yaw_gyro;

    last_time = time_now;

    return {this->last_euler_angles.x, this->last_euler_angles.y, this->last_euler_angles.z};

}
/**
 * @brief  Get the offset of the sensor
 * @param  SampleTimes: Number of samples to take
 * @param  Expected_g: Expected g value
 * @attention DO NOT run this function while the sensor is moving
*/
template<typename Sensor_i, typename _InitParams_>
void IMU_Base<Sensor_i, _InitParams_>::getOffset(float sample_times, float expected_g){
    this->params.g = expected_g;
    Vec3_t Accel;
    Vec3_t Gyro;
    Vec3_t Accel_Sum = {0,0,0};
    Vec3_t Gyro_Sum = {0,0,0};
    this->offset.accel = {0.0f, 0.0f, 0.0f};
    this->offset.gyro = {0.0f, 0.0f, 0.0f};

    for (uint8_t i = 0; i < sample_times; i++){
        this->readAccel(Accel);
        this->readGyro(Gyro);
        Accel_Sum = Accel_Sum + Accel;
        Gyro_Sum = Gyro_Sum + Gyro;
        HAL_Delay(this->params.call_period + 1);
    }

    this->offset.accel.x = (float)Accel_Sum.x / sample_times;
    this->offset.accel.y = (float)Accel_Sum.y / sample_times;
    this->offset.accel.z = (float)Accel_Sum.z / sample_times - expected_g;
    this->offset.gyro.x = (float)Gyro_Sum.x / sample_times;
    this->offset.gyro.y = (float)Gyro_Sum.y / sample_times;
    this->offset.gyro.z = (float)Gyro_Sum.z / sample_times;
}

template<typename Sensor_i, typename _InitParams_>
void IMU_Base<Sensor_i, _InitParams_>::calibrateZ(float sample_times){
    Vec3_t Gyro;
    Vec3_t Gyro_Sum = {0,0,0};
    this->offset.accel.z = 0.0f;

    for (uint8_t i = 0; i < sample_times; i++){
        this->readGyro(Gyro);
        Gyro_Sum = Gyro_Sum + Gyro;
        HAL_Delay(this->params.call_period + 1);
    }

    this->offset.gyro.z = (float)Gyro_Sum.z / sample_times;
}

template<typename Sensor_i, typename _InitParams_>
void IMU_Base<Sensor_i, _InitParams_>::setOffset(Vec3_t accel_offset, Vec3_t gyro_offset){
    this->offset.accel = accel_offset;
    this->offset.gyro = gyro_offset;
}

#endif

#endif /* __IMU_IPP__ */