#include "app_main.h"
#include "cpp_wrapper.h"
#include "sstrtotype.h"
#include "stm32f3xx_hal_adc_ex.h"
#include <sys/types.h>


#ifdef __cplusplus
#define VOLTAGE_COEFF 4.12f
// Function prototypes
void key1_callback();
void key_imuoffset_callback();
void Task_GetMotorSpeed(uint8_t runtime);
void Task_PIDt(uint8_t runtime);
void Task_PIDv(uint8_t runtime);
void Task_PIDr(uint8_t runtime);
void Task_IMU(uint8_t runtime);
void Task_UpdateMotor(uint8_t runtime);
void Task_GetBatteryVoltage(uint16_t runtime);
void Task_UART(uint16_t runtime);
void Task_OLED(uint16_t runtime);
void Task_key1(uint8_t runtime); void key1_callback();
void Task_key_imuoffset(uint8_t runtime); void key_imuoffset_callback();

//global variables
float raw_angvel_l = 0;
float raw_angvel_r = 0;
float angvel = 0;
float theta = 0;

uint16_t adc_voltages[2];
float bat_voltage;
float exp_voltage_v;
float exp_voltage_t;
float exp_voltage;
float exp_voltage_rot;
float exp_theta;
const float g = 9.8f;
const float R_w = 0.325f;
const float L = 0.25f;

Vec3_t Accel, Gyro, EurAngs;

uint32_t Task_IMU_runtime, Task_PIDv_runtime, Task_UpdateMotor_runtime;
//debug
volatile uint8_t UARTRcvBfr[40];
//bit0:pid mode flag; bit1:imu offset flag
volatile bool debug_mode = 1;
volatile bool oled_flag = 1;
volatile bool uart_flag = 1;
volatile bool imu_ready_flag = 0;
volatile bool imu_cplt_flag = 1;
struct{
    volatile float ch[4];
    uint8_t tail[4];
}msg;
const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};
//float Accel_[3], Gyro_[3], EurAngs_[3];

//declare objects
MPU6050 imu(&hi2c1);
uint8_t imu_buffer[14];
Key key1(GPIOA, GPIO_PIN_8, GPIO_PIN_SET, RISING_EDGE, key1_callback);
Key key_imuoffset(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET, RISING_EDGE, key_imuoffset_callback);

PWM motor_l_pwm(&htim3, TIM_CHANNEL_2, 36000000);
PWM motor_r_pwm(&htim3, TIM_CHANNEL_1, 36000000);

TB6612_Motor motor_l(motor_l_pwm);
TB6612_Motor motor_r(motor_r_pwm);

MotorEncoder motor_l_encoder(20.049, 13, 2);
MotorEncoder motor_r_encoder(20.049, 13, 2);

PID theta_pid;
PID angvel_pid;
PID velocity_pid;
PID rotate_pid;

SSD1306_I2C oled(&hi2c1);

AdaptiveFirstOrderFilter voltage_filter(0.08, 1, 0.92);


void app_main(){
    dwt_ustime_init();    

    imu.init({
        2, 250, 
        3, 4,
         1});
    float offsets_data[6];
    Flash_Read(offsets_data, 6, PAGE0);
    imu.setOffset({offsets_data[0], offsets_data[1], offsets_data[2]},
         {offsets_data[3], offsets_data[4], offsets_data[5]});

    motor_l_pwm.init(6000);
    motor_r_pwm.init(6000);
    motor_l.init({
        GPIOA, GPIO_PIN_11,
        GPIOA, GPIO_PIN_12,
        GPIOA, GPIO_PIN_5,
    });
    motor_r.init({
        GPIOB, GPIO_PIN_15,
        GPIOB, GPIO_PIN_14,
        GPIOA, GPIO_PIN_5,
    });
    motor_r_encoder.init({
        .PhraseA_GPIOx = GPIOA,
        .PhraseA_Pin = GPIO_PIN_0,
        .PhraseB_GPIOx = GPIOC,
        .PhraseB_Pin = GPIO_PIN_14,
    });
    motor_l_encoder.init({
        .PhraseA_GPIOx = GPIOA,
        .PhraseA_Pin = GPIO_PIN_1,
        .PhraseB_GPIOx = GPIOC,
        .PhraseB_Pin = GPIO_PIN_15,
    });

    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc_voltages, sizeof(adc_voltages)/sizeof(uint16_t));
    
    float theta_pid_factors[6];
    Flash_Read(theta_pid_factors, 6, PAGE1);
    theta_pid.setFactors(theta_pid_factors[0], theta_pid_factors[1], theta_pid_factors[2]);
    theta_pid.setSP(0);
    theta_pid.setLimit(theta_pid_factors[3], theta_pid_factors[4], theta_pid_factors[5]);
    
    float velocity_pid_factors[6];
    Flash_Read(velocity_pid_factors, 6, PAGE2);
    velocity_pid.setFactors(velocity_pid_factors[0], velocity_pid_factors[1], velocity_pid_factors[2]);
    velocity_pid.setSP(0);
    velocity_pid.setLimit(velocity_pid_factors[3], velocity_pid_factors[4], velocity_pid_factors[5]);

    float rotate_pid_factors[6];
    Flash_Read(rotate_pid_factors, 6, PAGE4);
    rotate_pid.setFactors(rotate_pid_factors[0], rotate_pid_factors[1], rotate_pid_factors[2]);
    rotate_pid.setSP(0);
    rotate_pid.setLimit(rotate_pid_factors[3], rotate_pid_factors[4], rotate_pid_factors[5]);

    oled.init();
    oled.clear();
    oled.setString("HELLO", {0, 0}, ascii12, 10, 10);
    oled.showFrame();
    HAL_Delay(1000);
    oled.clear();
    oled.showFrame();

    motor_l.stby(GPIO_PIN_RESET);
    motor_r.stby(GPIO_PIN_RESET);
    
    msg.tail[0] = 0x00;
    msg.tail[1] = 0x00;
    msg.tail[2] = 0x80;
    msg.tail[3] = 0x7f;

    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)UARTRcvBfr, sizeof(UARTRcvBfr));
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
    Task_GetBatteryVoltage(0);
    motor_l.stby(0);
    motor_r.stby(0);
}

void loop(){
    Task_IMU(1);

    Task_GetMotorSpeed(8);
    Task_PIDv(15);
    
    Task_PIDt(5);

    Task_PIDr(5);

    Task_UpdateMotor(6);

    

    Task_key1(10);
    Task_key_imuoffset(10);
    Task_UART(20);
    
    Task_GetBatteryVoltage(5000);
    Task_OLED(500);
}

//
void Task_GetMotorSpeed(uint8_t runtime){
    NON_BLOCK_DELAY(runtime);
    raw_angvel_l = -motor_l_encoder.getAngVelocity();
    raw_angvel_r = motor_r_encoder.getAngVelocity();
    angvel = (raw_angvel_r + raw_angvel_l) * 0.50f;
}


void Task_PIDt(uint8_t runtime){
    NON_BLOCK_DELAY(runtime);
    exp_voltage_t = -theta_pid.getCO(EurAngs.y, -Gyro.y);
}
void Task_PIDv(uint8_t runtime){
    NON_BLOCK_DELAY(runtime);
    // DWT_Timestamp last_time = {0};
    // float FB = R_w * angvel - (L + R_w) * Gyro.y;
    // theta_pid.setSP(s_atan(velocity_pid.getCO(FB) / g));
    exp_voltage_v = velocity_pid.getCO(R_w * angvel);
    //Task_PIDv_runtime = getDistance(last_time, getTick());
    //last_time = getTick();
}
void Task_PIDr(uint8_t runtime){
    NON_BLOCK_DELAY(runtime);
    exp_voltage_rot = rotate_pid.getCO(Gyro.z);
}
void Task_IMU(uint8_t runtime){
    NON_BLOCK_DELAY(runtime);
    if (imu_ready_flag && imu_cplt_flag){
        imu_ready_flag = 0;
        imu_cplt_flag = 0;
        imu.readAccelGyro_IT_start(imu_buffer);
    }
    //imu.readAccelGyro(Accel, Gyro);
    //EurAngs = imu.getEulerAngles(Accel, Gyro);
}
void Task_UpdateMotor(uint8_t runtime){
    NON_BLOCK_DELAY(runtime);
    //DWT_Timestamp time_now = getTick();
    exp_voltage = exp_voltage_t - exp_voltage_v;
    float exp_voltage_l = exp_voltage + exp_voltage_rot;
    float exp_voltage_r = exp_voltage - exp_voltage_rot;

    if (exp_voltage_l < 0){
        motor_l.setControl(MOTOR_BACKWARD);
    }
    else {
        motor_l.setControl(MOTOR_FORWARD);
    }

    if (exp_voltage_r < 0){
        motor_r.setControl(MOTOR_BACKWARD);
    }
    else {
        motor_r.setControl(MOTOR_FORWARD);
    }

    float duty_l;
    if (ABS(exp_voltage_l) > bat_voltage){
        duty_l = 0.99f;
    }
    else{
        duty_l = exp_voltage_l / bat_voltage;
    }

    float duty_r;
    if (ABS(exp_voltage_r) > bat_voltage){
        duty_r = 0.99f;
    }
    else{
        duty_r = exp_voltage_r / bat_voltage;
    }
    
    motor_l.setDuty(ABS(duty_l));
    motor_r.setDuty(ABS(duty_r));
    //Task_UpdateMotor_runtime = getDistance(time_now, getTick());
}

void Task_GetBatteryVoltage(uint16_t runtime){
  NON_BLOCK_DELAY(runtime);

  float vref_div_4095 = (1.2 / adc_voltages[0]);
  float raw_bat_voltage = adc_voltages[1] * vref_div_4095 * VOLTAGE_COEFF;
  bat_voltage = voltage_filter.get(raw_bat_voltage);
}
void Task_UART(uint16_t runtime){
    NON_BLOCK_DELAY(runtime);
    if ((!(debug_mode)) && uart_flag){
        msg.ch[0] = exp_voltage;
        msg.ch[1] = angvel;
        msg.ch[2] = EurAngs.y;
        msg.ch[3] = theta_pid.SP();
        HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&msg, sizeof(msg));
        uart_flag = 0;
  }
}

void Task_OLED(uint16_t runtime){
    NON_BLOCK_DELAY(runtime);
    char msg[20];
    if (debug_mode){
        oled.clear();
        char digit_msg[16] = "\0";
        float args[3] = {exp_voltage, EurAngs.y, bat_voltage};
        s_joinf(digit_msg, ',', args, 3, 2);
        sprintf(msg, "%d,%s", debug_mode, digit_msg);
        oled.setString(msg, {0, 40}, ascii12, 10, 10);
        oled.showFrame();
        oled_flag = 1;
    }
    else {
        if (oled_flag){
        oled.clear();
        oled.showFrame();
        oled_flag = 0;
        }
    }
}

void Task_key1(uint8_t runtime){
    NON_BLOCK_DELAY(runtime);
    key1.proc();
}
void key1_callback(){ 
    debug_mode = !debug_mode;

    motor_l.stby(!debug_mode);
    motor_r.stby(!debug_mode);
}

void Task_key_imuoffset(uint8_t runtime){
    NON_BLOCK_DELAY(runtime);
    key_imuoffset.proc();
}
void key_imuoffset_callback(){
    if (debug_mode){
        imu.getOffset(200, g);
        Vec3_t accel_offset = imu.getOffsetAccel();
        Vec3_t gyro_offset = imu.getOffsetGyro();
        float offsets[6] = {
            accel_offset.x, accel_offset.y, accel_offset.z,
            gyro_offset.x, gyro_offset.y, gyro_offset.z
        };
        Flash_Write(offsets, 6, PAGE0);
        HAL_UART_Transmit(&huart2, (uint8_t *)"IMU offset updated", 18, 1000);
        HAL_UART_Transmit(&huart2, tail, 4, 1000);
    }
}

//UART debug command analiysis
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size){
    if (huart == &huart2){
        char msg[65];
        
        memset(msg, 0, sizeof(msg));
        sprintf(msg, "%6s","rcved");
        HAL_UART_Transmit(&huart2, (uint8_t *)msg, sizeof(msg), 1000);
        HAL_UART_Transmit(&huart2, tail, 4, 1000);

        if (debug_mode){
            float pidparams[3];
            //divide cmd and data
            char *cmd = s_strtok((char *)UARTRcvBfr, ' ');
            char *data = s_strtok(NULL, ' ');
            if (s_strtok(NULL, ' ') != NULL){
                sprintf(msg, "%15s","too many args");
                HAL_UART_Transmit(&huart2, (uint8_t *)msg, sizeof(msg), 1000);
                HAL_UART_Transmit(&huart2, tail, 4, 1000);
                return;
            }

            uint8_t cmd_flag = 0;
            if (cmd == NULL){
                sprintf(msg, "error");
                HAL_UART_Transmit(&huart2, (uint8_t *)msg, sizeof(msg), 1000);
                HAL_UART_Transmit(&huart2, tail, 4, 1000);
                return;
            }
            else if (strcmp(cmd, "pid1") == 0) {
                cmd_flag = 4;
            }
            else if (strcmp(cmd, "pid2") == 0){
                cmd_flag = 1;
            }
            else if (strcmp(cmd, "sp2") == 0){
                cmd_flag = 2;
            }
            else if (strcmp(cmd, "limit2") == 0){
                cmd_flag = 3;
            }
            else if (strcmp(cmd, "limit1") == 0){
                cmd_flag = 5;
            }
            else if (strcmp(cmd, "pid3") == 0){
                cmd_flag = 6;
            }
            else if (strcmp(cmd, "sp3") == 0){
                cmd_flag = 7;
            }
            else if (strcmp(cmd, "limit3") == 0){
                cmd_flag = 8;
            }
            else {
                sprintf(msg, "%15s","unknown cmd");
                HAL_UART_Transmit(&huart2, (uint8_t *)msg, sizeof(msg), 1000);
                HAL_UART_Transmit(&huart2, tail, 4, 1000);
                return;
            }
            //divide the command
            char* token = s_strtok(data, ',');
            for (uint8_t i = 0; token != NULL; i++){
                pidparams[i] = s_atof(token);
                token = s_strtok(NULL, ',');
            }

            switch (cmd_flag) {
                case 1:
                    velocity_pid.setFactors(pidparams[0], pidparams[1], pidparams[2]);
                    break;
                case 2:
                    velocity_pid.setSP(pidparams[0]);
                    break;
                case 3:
                    velocity_pid.setLimit(pidparams[0], pidparams[1], pidparams[2]);
                    break;
                case 4:
                    theta_pid.setFactors(pidparams[0], pidparams[1], pidparams[2]);
                    break;
                case 5:
                    theta_pid.setLimit(pidparams[0], pidparams[1], pidparams[2]);
                    break;
                case 6:
                    rotate_pid.setFactors(pidparams[0], pidparams[1], pidparams[2]);
                    break;
                case 7:
                    rotate_pid.setSP(pidparams[0]);
                    break;
                case 8:
                    rotate_pid.setLimit(pidparams[0], pidparams[1], pidparams[2]);
                    break;
            }
            switch (cmd_flag) {
                case 1:
                case 2:
                case 3:
                {
                    float paramsv[6] = {
                        velocity_pid.K_p(),
                        velocity_pid.K_i(),
                        velocity_pid.K_d(),
                        velocity_pid.MaxOutput(),
                        velocity_pid.MinOutput(),
                        velocity_pid.KiLimit()
                    };
                    Flash_Write(paramsv, 6, PAGE2);
                    char digit_msg[40] = "\0";
                    s_joinf(digit_msg, ',', paramsv, 6, 3);
                    sprintf(msg, "pidv:%s", digit_msg);
                    break;
                }
                case 4:
                case 5:
                {
                    float paramst[6] = {
                        theta_pid.K_p(),
                        theta_pid.K_i(),
                        theta_pid.K_d(),
                        theta_pid.MaxOutput(),
                        theta_pid.MinOutput(),
                        theta_pid.KiLimit()
                    };
                    Flash_Write(paramst, 6, PAGE1);
                    char digit_msg[40] = "\0";
                    s_joinf(digit_msg, ',', paramst, 6, 3);
                    sprintf(msg, "pidt:%s", digit_msg);
                    break;
                }
                case 6:
                case 7:
                case 8:
                {
                    float paramsr[6] = {
                        rotate_pid.K_p(),
                        rotate_pid.K_i(),
                        rotate_pid.K_d(),
                        rotate_pid.MaxOutput(),
                        rotate_pid.MinOutput(),
                        rotate_pid.KiLimit()
                    };
                    Flash_Write(paramsr, 6, PAGE4);
                    char digit_msg[40] = "\0";
                    s_joinf(digit_msg, ',', paramsr, 6, 3);
                    sprintf(msg, "pidr:%s", digit_msg);
                    break;
                }
            }

            memset((uint8_t *)UARTRcvBfr, 0, sizeof(UARTRcvBfr));
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, sizeof(msg), 1000);
            HAL_UART_Transmit(&huart2, tail, 4, 1000);
        }
        
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)UARTRcvBfr, sizeof(UARTRcvBfr));
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if (huart == &huart2){
        uart_flag = 1;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
    if (GPIO_Pin == GPIO_PIN_0){
        motor_r_encoder.irq_handler();
    }
    else if (GPIO_Pin == GPIO_PIN_1){
        motor_l_encoder.irq_handler();
    }
    else if (GPIO_Pin == GPIO_PIN_15) {
        imu_ready_flag = 1;
    }
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c){
    if (hi2c == &hi2c1){
        //DWT_Timestamp time_now = getTick();
        imu.readAccelGyro_IT_cplt_handler(imu_buffer, Accel, Gyro);
        EurAngs = imu.getEulerAngles(Accel, Gyro);   
        imu_cplt_flag = 1;
        //Task_IMU_runtime = getDistance(time_now, getTick());
    }
}

#endif