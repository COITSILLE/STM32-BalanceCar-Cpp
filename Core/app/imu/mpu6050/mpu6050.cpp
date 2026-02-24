#include "mpu6050.h"

#ifdef __cplusplus
extern "C" {
#endif

#define PWR_MGMT_1 0x6B
#define ACCEL_OUT 0x3B
#define GYRO_OUT 0x43
#define DLPF_CFG 0x1A
#define SMPLRT_DIV 0x19
#define INT_ENABLE 0x38
#define INT_PIN_CFG 0x37

#ifdef __cplusplus
}
void MPU6050::reset(){
    uint8_t cmd[2] = {0x6B, 0b10000000};
    HAL_I2C_Master_Transmit(this->_hi2c_, this->_address_, cmd, sizeof(cmd), HAL_MAX_DELAY);
}
void MPU6050::setReg(uint8_t reg, uint8_t value){
    uint8_t cmd[2] = {reg, value};
    HAL_I2C_Master_Transmit(this->_hi2c_, this->_address_, cmd, sizeof(cmd), HAL_MAX_DELAY);
}

void MPU6050::init(InitParams init_params){
    this->setReg(PWR_MGMT_1, 0b00000000);// Wake up device

    this->AccelFactor = (init_params.accelFS * this->params.g) / 32768.0f;
    this->GyroFactor = DEG_TO_RAD * init_params.gyroFS / 32768.0f;

    if (init_params._DLPF_CFG_ == 0 || init_params._DLPF_CFG_ == 7){
        this->params.sampling_period = (init_params._SMPLRT_DIV_ + 1) / 8000.0;
    }
    else {
        this->params.sampling_period = (init_params._SMPLRT_DIV_ + 1) / 1000.0;
    }

    switch (init_params.accelFS) {
        case 2:
            this->setReg(0x1C, 0b00000000);// Accel +/- 2g
            break;
        case 4:
            this->setReg(0x1C, 0b00001000);// Accel +/- 4g
            break;
        case 8:
            this->setReg(0x1C, 0b00010000);// Accel +/- 8g
            break;
        case 16:
            this->setReg(0x1C, 0b00011000);// Accel +/- 16g
            break;
        default:
            setReg(0x1C, 0b00000000);// Accel +/- 2g
            break;
    }

    switch (init_params.gyroFS) {
        case 250:
            this->setReg(0x1B, 0b00000000);// Gyro +/- 250 deg/s
            break;
        case 500:
            this->setReg(0x1B, 0b00001000);// Gyro +/- 500 deg/s
            break;
        case 1000:
            this->setReg(0x1B, 0b00010000);// Gyro +/- 1000 deg/s
            break;
        case 2000:
            this->setReg(0x1B, 0b00011000);// Gyro +/- 2000 deg/s
            break;
        default:
            this->setReg(0x1B, 0b00000000);// Gyro +/- 250 deg/s
            break;
    
    }
    this->setReg(DLPF_CFG, init_params._DLPF_CFG_& 0b00000111);// DLPF 
    this->setReg(SMPLRT_DIV, init_params._SMPLRT_DIV_ & 0b00000111);// Sample Rate division

    if (init_params.use_INT){
        this->setReg(INT_ENABLE, 0b00000001);//Enable data ready interrupt
        this->setReg(INT_PIN_CFG, 0b00010000);//low active, push-pull, 50us signal, clear IT flag after any reading
    }
}


void MPU6050::readAccel(Vec3_t& accel_vec){
    uint8_t rawdata[6];

    HAL_I2C_Mem_Read(this->_hi2c_, this->_address_, ACCEL_OUT, I2C_MEMADD_SIZE_8BIT,
        rawdata, sizeof(rawdata), HAL_MAX_DELAY);

    accel_vec.x = (float)(int16_t)((rawdata[0] << 8) | rawdata[1]) * this->AccelFactor - this->calibration.accel.x;
    accel_vec.y = (float)(int16_t)((rawdata[2] << 8) | rawdata[3]) * this->AccelFactor - this->calibration.accel.y;
    accel_vec.z = (float)(int16_t)((rawdata[4] << 8) | rawdata[5]) * this->AccelFactor - this->calibration.accel.z;
}

void MPU6050::readGyro(Vec3_t& gyro_vec){
    uint8_t rawdata[6];

    HAL_I2C_Mem_Read(this->_hi2c_, this->_address_, GYRO_OUT, I2C_MEMADD_SIZE_8BIT,
        rawdata, sizeof(rawdata), HAL_MAX_DELAY);

    gyro_vec.x = ((float)(int16_t)((rawdata[0] << 8) | rawdata[1]) * this->GyroFactor) - this->calibration.gyro.x;
    gyro_vec.y = ((float)(int16_t)((rawdata[2] << 8) | rawdata[3]) * this->GyroFactor) - this->calibration.gyro.y;
    gyro_vec.z = ((float)(int16_t)((rawdata[4] << 8) | rawdata[5]) * this->GyroFactor) - this->calibration.gyro.z;
}

void MPU6050::readAccelGyro(Vec3_t& accel_vec, Vec3_t& gyro_vec){
    uint8_t rawdata[14];

    HAL_I2C_Mem_Read(this->_hi2c_, this->_address_, ACCEL_OUT, I2C_MEMADD_SIZE_8BIT,
        rawdata, sizeof(rawdata), HAL_MAX_DELAY);

    accel_vec.x = (float)(int16_t)((rawdata[0] << 8) | rawdata[1]) * this->AccelFactor - this->calibration.accel.x;
    accel_vec.y = (float)(int16_t)((rawdata[2] << 8) | rawdata[3]) * this->AccelFactor - this->calibration.accel.y;
    accel_vec.z = (float)(int16_t)((rawdata[4] << 8) | rawdata[5]) * this->AccelFactor - this->calibration.accel.z;

    gyro_vec.x = ((float)(int16_t)((rawdata[8] << 8) | rawdata[9]) * this->GyroFactor) - this->calibration.gyro.x;
    gyro_vec.y = ((float)(int16_t)((rawdata[10] << 8) | rawdata[11]) * this->GyroFactor) - this->calibration.gyro.y;
    gyro_vec.z = ((float)(int16_t)((rawdata[12] << 8) | rawdata[13]) * this->GyroFactor) - this->calibration.gyro.z;
}

void MPU6050::readAccelGyro_IT_Start(uint8_t *buffer){
    HAL_I2C_Mem_Read_IT(this->_hi2c_, this->_address_, ACCEL_OUT, I2C_MEMADD_SIZE_8BIT,
        buffer, 14);
}

void MPU6050::readAccelGyro_IT_CpltHandler(uint8_t *rawdata, Vec3_t& accel_vec, Vec3_t& gyro_vec){
    accel_vec.x = (float)(int16_t)((rawdata[0] << 8) | rawdata[1]) * this->AccelFactor - this->calibration.accel.x;
    accel_vec.y = (float)(int16_t)((rawdata[2] << 8) | rawdata[3]) * this->AccelFactor - this->calibration.accel.y;
    accel_vec.z = (float)(int16_t)((rawdata[4] << 8) | rawdata[5]) * this->AccelFactor - this->calibration.accel.z;

    gyro_vec.x = ((float)(int16_t)((rawdata[8] << 8) | rawdata[9]) * this->GyroFactor) - this->calibration.gyro.x;
    gyro_vec.y = ((float)(int16_t)((rawdata[10] << 8) | rawdata[11]) * this->GyroFactor) - this->calibration.gyro.y;
    gyro_vec.z = ((float)(int16_t)((rawdata[12] << 8) | rawdata[13]) * this->GyroFactor) - this->calibration.gyro.z;
}


#endif /* __cplusplus */