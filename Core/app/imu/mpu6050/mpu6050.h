#ifndef MPU6050_H
#define MPU6050_H

#include "vector.h"
#ifdef __cplusplus
extern "C" {
    
#include "main.h"
#include "i2c.h"
#include <stdint.h>

#endif

#ifdef __cplusplus
}
#include "imu.h"
struct InitParams{
    uint8_t accelFS;
    uint16_t gyroFS;
    uint8_t _DLPF_CFG_;
    uint8_t _SMPLRT_DIV_;

    bool use_INT;
};
class MPU6050 : public IMU_Base<MPU6050, InitParams> {
public:
    MPU6050(I2C_HandleTypeDef *hi2c, uint16_t address = 0xD0)
        : _hi2c_(hi2c), _address_(address) {} //typical address: 0xD0
    /**
     * @brief Initialize the MPU6050
     * @param init_params.accelFS Accelerometer Full Scale: 2/4/8/16g
     * @param init_params.gyroFS Gyroscope Full Scale: 250/500/1000/2000 dps
     * @param init_params._DLPF_CFG Digital Low Pass Filter Configuration: 0-7 (Recommended: 3)
     * @param init_params._SMPLRT_DIV Sample Rate Division: 0-7 (Recommended: 4 when DLPF_CFG = 3)
     * @param init_params.use_INT Enable interrupt output
     */
    void init(InitParams init_params);
    void reset();
    void readAccel(Vec3_t& accel_vec);
    void readGyro(Vec3_t& gyro_vec);

    void readAccelGyro(Vec3_t& accel_vec, Vec3_t& gyro_vec);
    void readAccelGyro_IT_Start(uint8_t *buffer);
    void readAccelGyro_IT_CpltHandler(uint8_t *buffer, Vec3_t& accel_vec, Vec3_t& gyro_vec);
private:
    I2C_HandleTypeDef *_hi2c_;
    const uint16_t _address_;
    void setReg(uint8_t reg, uint8_t value);
    float AccelFactor;
    float GyroFactor;
};


#endif

#endif // MPU6050_H
