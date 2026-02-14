#include "app_main.h"
#include "cpp_wrapper.h"
#include "sstrtotype.h"

#ifdef __cplusplus

// ==================== 函数原型声明 ====================
void Task_CheckLiftingUp(uint8_t runtime);
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
void Task_key_gyrooffset(uint8_t runtime); void key_gyrooffset_callback();

// ==================== 全局变量 ====================

// 物理常量
const float g = 9.8f;      // 重力加速度 (m/s²)
const float R_w = 0.033f;  // 轮子半径 (m)
const float L = 0.026f;    // 车体重心到轮轴距离 (m)

// 运动状态变量
float raw_angvel = 0;  // 原始角速度 (rad/s)
float linvel = 0;      // 线速度 (m/s)

// 电池电压相关
const float VOLTAGE_COEFF = 4.12f;  // 电压转换系数 (实验确定)
uint16_t adc_voltages[2];  // ADC采样值 [0]:内部参考电压 [1]:电池电压
float bat_voltage;         // 电池电压 (V)

// 控制输出变量
float exp_voltage;      // 期望输出电压 (V)
float exp_voltage_rot;  // 转向控制电压 (V)

// IMU传感器数据
Vec3_t Accel, Gyro, EurAngs;  // 加速度、角速度、欧拉角

// 调试和性能监测
//uint32_t Task_runtime1, Task_runtime2, Task_runtime3;  // 任务运行时间统计 (us)

// UART通信相关
volatile uint8_t UARTRcvBfr[40];  // UART接收缓冲区
struct {
    volatile float ch[6];  // 数据：[0]电压 [1]线速度 [2]俯仰角 [3]角度设定值 [4]角速度 [5]原始角速度
    uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f}; // vofa+ justfloat格式数据包尾
} msg;
const uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7f};  // （给字面量）数据包尾

// 系统状态标志
volatile bool debug_mode = 1;       // 调试模式标志（1:调试 0:运行）
volatile bool oled_flag = 1;        // OLED显示更新标志
volatile bool uart_flag = 1;        // UART发送完成标志
volatile bool imu_ready_flag = 0;   // IMU数据就绪标志（来自外部中断）
volatile bool imu_cplt_flag = 1;    // IMU数据读取完成标志

// ==================== 对象实例化 ====================

// IMU传感器对象
MPU6050 imu(&hi2c1);
uint8_t imu_buffer[14];  // IMU DMA读取缓冲区

// 按键对象
Key key1(GPIOA, GPIO_PIN_8, GPIO_PIN_SET, RISING_EDGE, key1_callback);                       // 调试模式切换按键
Key key_imuoffset(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET, RISING_EDGE, key_imuoffset_callback);   // IMU零偏校准按键
Key key_gyrooffset(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET, RISING_EDGE, key_gyrooffset_callback); // 陀螺仪Z轴校准按键

// PWM对象
PWM motor_l_pwm(&htim3, TIM_CHANNEL_2, 36000000);  // 左电机PWM (定时器频率36MHz)
PWM motor_r_pwm(&htim3, TIM_CHANNEL_1, 36000000);  // 右电机PWM

// 电机驱动对象
TB6612_Motor motor_l(motor_l_pwm);  // 左电机
TB6612_Motor motor_r(motor_r_pwm);  // 右电机

// 编码器对象 (减速比20.049, 霍尔13线, 双边沿2)
MotorEncoder motor_l_encoder(20.049, 13, 2);  // 左轮编码器
MotorEncoder motor_r_encoder(20.049, 13, 2);  // 右轮编码器

// PID控制器对象
PID theta_pid;     // 角度环PID
PID velocity_pid;  // 速度环PID
PID rotate_pid;    // 转向环PID

// OLED显示对象
SSD1306_I2C oled(&hi2c1);

// 滤波器对象
FirstOrderFilter linvel_filter(0.35);                  // 线速度一阶滤波器 (截止频率系数0.35)
AdaptiveFirstOrderFilter voltage_filter(0.08, 1, 0.92); // 电池电压自适应滤波器 (alpha=0.08, 容差=1V, alpha增量=0.92)

// ==================== 主初始化函数 ====================

void app_main() {
    // 初始化微秒级定时器
    dwt_ustime_init();

    // ---------- IMU初始化 ----------
    imu.init({
        .accelFS = 2,        // 加速度计量程 ±2g
        .gyroFS = 250,       // 陀螺仪量程 ±250°/s
        ._DLPF_CFG_ = 3,      // 数字低通滤波器配置
        ._SMPLRT_DIV_ = 4,    // 采样率分频
        .use_INT = 1         // 使能数据就绪中断
    });
    
    // 从Flash读取IMU零偏数据
    float offsets_data[6];
    Flash_Read(offsets_data, 6, PAGE0);
    imu.setCalibration(
        {offsets_data[0], offsets_data[1], offsets_data[2]},  // 加速度计零偏 (x, y, z)
        {offsets_data[3], offsets_data[4], offsets_data[5]}   // 陀螺仪零偏 (x, y, z)
    );

    // ---------- 电机和编码器初始化 ----------
    // PWM频率设置为6kHz
    motor_l_pwm.init(6000);
    motor_r_pwm.init(6000);
    
    // 左电机GPIO配置
    motor_l.init({
        .Ctrl1GPIOx = GPIOA, .Ctrl1Pin = GPIO_PIN_11,  // 控制引脚1
        .Ctrl2GPIOx = GPIOA, .Ctrl2Pin = GPIO_PIN_12,  // 控制引脚2
        .StbyGPIOx = GPIOA,  .StbyPin = GPIO_PIN_5     // 待机控制引脚
    });
    
    // 右电机GPIO配置
    motor_r.init({
        .Ctrl1GPIOx = GPIOB, .Ctrl1Pin = GPIO_PIN_15,  // 控制引脚1
        .Ctrl2GPIOx = GPIOB, .Ctrl2Pin = GPIO_PIN_14,  // 控制引脚2
        .StbyGPIOx = GPIOA,  .StbyPin = GPIO_PIN_5     // 待机控制引脚
    });
    
    // 右轮编码器GPIO配置
    motor_r_encoder.init({
        .PhraseA_GPIOx = GPIOA, .PhraseA_Pin = GPIO_PIN_0,  // A相引脚
        .PhraseB_GPIOx = GPIOC, .PhraseB_Pin = GPIO_PIN_14  // B相引脚
    });
    
    // 左轮编码器GPIO配置
    motor_l_encoder.init({
        .PhraseA_GPIOx = GPIOA, .PhraseA_Pin = GPIO_PIN_1,  // A相引脚
        .PhraseB_GPIOx = GPIOC, .PhraseB_Pin = GPIO_PIN_15  // B相引脚
    });

    // ---------- ADC初始化 ----------
    HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);  // ADC校准
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc_voltages, sizeof(adc_voltages) / sizeof(uint16_t));

    // ---------- PID参数初始化 ----------
    // 角度环PID (从Flash读取)
    float theta_pid_factors[6];  // [0-2]:Kp Ki Kd  [3-5]:MaxOut MinOut KiLimit
    Flash_Read(theta_pid_factors, 6, PAGE1);
    theta_pid.setFactors(theta_pid_factors[0], theta_pid_factors[1], theta_pid_factors[2]);
    theta_pid.setTarget(0);  // 设定值为0°（竖直）
    theta_pid.setLimit(theta_pid_factors[3], theta_pid_factors[4], theta_pid_factors[5]);

    // 速度环PID (从Flash读取)
    float velocity_pid_factors[6];
    Flash_Read(velocity_pid_factors, 6, PAGE2);
    velocity_pid.setFactors(velocity_pid_factors[0], velocity_pid_factors[1], velocity_pid_factors[2]);
    velocity_pid.setTarget(0);  // 设定值为0 m/s（静止）
    velocity_pid.setLimit(velocity_pid_factors[3], velocity_pid_factors[4], velocity_pid_factors[5]);

    // 转向环PID (从Flash读取)
    float rotate_pid_factors[6];
    Flash_Read(rotate_pid_factors, 6, PAGE4);
    rotate_pid.setFactors(rotate_pid_factors[0], rotate_pid_factors[1], rotate_pid_factors[2]);
    rotate_pid.setTarget(0);  // 设定值为0 rad/s（不旋转）
    rotate_pid.setLimit(rotate_pid_factors[3], rotate_pid_factors[4], rotate_pid_factors[5]);

    // ---------- OLED初始化 ----------
    oled.init();
    oled.clear();
    oled.setString("HELLO", {.x = 0, .y = 0}, ascii12, 10, 10);  // 显示启动信息
    oled.showFrame();
    HAL_Delay(1000);
    oled.clear();
    oled.showFrame();
    // ---------- UART初始化 ----------
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)UARTRcvBfr, sizeof(UARTRcvBfr));
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);  // 禁用半传输中断

    // ---------- 启动前准备 ----------
    Task_GetBatteryVoltage(0);  // 读取初始电池电压
    motor_l.stby(0); // 进入待机状态，等待主循环控制
    motor_r.stby(0);
}

// ==================== 主循环函数 ====================

void loop() {
    // 传感器数据采集
    Task_IMU(1);              // 1ms: IMU数据读取
    Task_GetMotorSpeed(5);    // 5ms: 编码器速度读取

    // PID控制计算
    Task_PIDv(20);            // 20ms: 速度环PID
    Task_PIDt(5);             // 5ms: 角度环PID（直立控制）
    Task_PIDr(5);             // 5ms: 旋转环PID（转向控制）

    // 电机输出
    Task_UpdateMotor(6);      // 6ms: 更新电机PWM输出

    // 人机交互
    Task_key1(10);            // 10ms: 模式切换按键扫描
    Task_key_imuoffset(10);   // 10ms: IMU校准按键扫描
    Task_key_gyrooffset(10);  // 10ms: 陀螺仪Z轴校准按键扫描
    // Task_UART(10);         // UART数据发送，用于实时调试

    // 系统监控
    Task_GetBatteryVoltage(5000);  // 5000ms: 电池电压检测
    Task_OLED(500);                // 500ms: OLED显示更新
}

// ==================== 任务函数实现 ====================

/**
 * @brief IMU数据读取任务（中断触发方式）
 */
void Task_IMU(uint8_t runtime) {
    NON_BLOCK_DELAY(runtime);
    // 当IMU数据就绪且上次读取完成时，启动新的DMA读取
    if (imu_ready_flag && imu_cplt_flag) {
        imu_ready_flag = 0;
        imu_cplt_flag = 0;
        imu.readAccelGyro_IT_start(imu_buffer);
    }
}

/**
 * @brief 电机速度获取任务
 */
void Task_GetMotorSpeed(uint8_t runtime) {
    NON_BLOCK_DELAY(runtime);
    
    // 获取左右轮编码器速度（左轮取负值以统一方向）
    float raw_angvel_l = -motor_l_encoder.getAngVelocity();
    float raw_angvel_r = motor_r_encoder.getAngVelocity();

    // 计算平均角速度
    float raw_angvel = (raw_angvel_r + raw_angvel_l) * 0.50f;
    
    // 计算线速度并补偿车体旋转
    linvel = linvel_filter.get(R_w * raw_angvel - L * Gyro.y);
}

// ---------- PID控制任务 ----------

/**
 * @brief 角度环PID任务（直立控制）
 */
void Task_PIDt(uint8_t runtime) {
    NON_BLOCK_DELAY(runtime);

    // 计算期望电压输出
    exp_voltage = -theta_pid.getOutput(EurAngs.y, Gyro.y);
}

/**
 * @brief 速度环PID任务（前进后退控制）
 */
void Task_PIDv(uint8_t runtime) {
    NON_BLOCK_DELAY(runtime);

    // 速度环输出作为角度环的设定值（串级控制）
    // arctan(a/g) 将速度误差转换为期望倾角
    theta_pid.setTarget(-s_atan(velocity_pid.getOutput(linvel) / g));
}

/**
 * @brief 旋转环PID任务（转向控制）
 */
void Task_PIDr(uint8_t runtime) {
    NON_BLOCK_DELAY(runtime);
    
    // 计算旋转控制电压（基于Z轴角速度）
    exp_voltage_rot = rotate_pid.getOutput(Gyro.z);
}

// ---------- 电机控制任务 ----------

/**
 * @brief 电机输出更新任务
 */
void Task_UpdateMotor(uint8_t runtime)  {
    NON_BLOCK_DELAY(runtime);
    
    // 差速控制：左右轮电压 = 直立电压 ± 旋转电压
    float exp_voltage_l = exp_voltage + exp_voltage_rot;  // 左轮期望电压
    float exp_voltage_r = exp_voltage - exp_voltage_rot;  // 右轮期望电压

    // 根据电压正负设置电机方向
    if (exp_voltage_l < 0) {
        motor_l.setControl(MOTOR_BACKWARD);  // 左轮后退
    } else {
        motor_l.setControl(MOTOR_FORWARD);   // 左轮前进
    }

    if (exp_voltage_r < 0) {
        motor_r.setControl(MOTOR_BACKWARD);  // 右轮后退
    } else {
        motor_r.setControl(MOTOR_FORWARD);   // 右轮前进
    }

    // 计算PWM占空比
    float duty_l = exp_voltage_l / bat_voltage;
    float duty_r = exp_voltage_r / bat_voltage;

    // 设置PWM占空比
    motor_l.setDuty(ABS(duty_l));
    motor_r.setDuty(ABS(duty_r));
}

/**
 * @brief 电池电压采集任务
 */
void Task_GetBatteryVoltage(uint16_t runtime){
  NON_BLOCK_DELAY(runtime);

  float vref_div_4095 = (1.2 / adc_voltages[0]);// 内部参考电压除以ADC分辨率
  float raw_bat_voltage = adc_voltages[1] * vref_div_4095 * VOLTAGE_COEFF;
  bat_voltage = voltage_filter.get(raw_bat_voltage);
}

/**
 * @brief UART调试数据发送任务
 */
void Task_UART(uint16_t runtime){
    NON_BLOCK_DELAY(runtime);
    if ((!(debug_mode)) && uart_flag){
        msg.ch[0] = exp_voltage;
        msg.ch[1] = linvel;
        msg.ch[2] = EurAngs.y;
        msg.ch[3] = theta_pid.sp();
        msg.ch[4] = Gyro.y;
        msg.ch[5] = raw_angvel;
        HAL_UART_Transmit_DMA(&huart2, (uint8_t *)&msg, sizeof(msg));
        uart_flag = 0;
  }
}

/**
 * @brief OLED显示更新任务
 */
void Task_OLED(uint16_t runtime){
    NON_BLOCK_DELAY(runtime);
    char oled_msg[20];
    if (debug_mode){
        oled.clear();
        char digit_msg[16] = "\0";
        float args[3] = {exp_voltage, EurAngs.y, bat_voltage};
        s_joinf(digit_msg, ',', args, 3, 2);
        sprintf(oled_msg, "%d,%s", debug_mode, digit_msg);
        oled.setString(oled_msg, {0, 40}, ascii12, 10, 10);
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

/**
 * @brief 模式切换按键任务
 */
void Task_key1(uint8_t runtime){
    NON_BLOCK_DELAY(runtime);
    key1.proc();
}

/**
 * @brief 模式切换按键回调函数
 */
void key1_callback(){ 
    debug_mode = !debug_mode;

    velocity_pid.reset();
    theta_pid.reset();

    motor_l.stby(!debug_mode);
    motor_r.stby(!debug_mode);
}

/**
 * @brief IMU零偏校准按键任务
 */
void Task_key_imuoffset(uint8_t runtime) {
    NON_BLOCK_DELAY(runtime);
    key_imuoffset.proc();
}

/**
 * @brief IMU零偏校准按键回调函数
 * @note 仅在调试模式下生效，校准时需保持传感器静止
 */
void key_imuoffset_callback() {
    if (debug_mode) {
        // 采集200次样本计算零偏
        imu.calibrate(200, g);
        
        Vec3_t accel_offset = imu.getCalibrationAccel();
        Vec3_t gyro_offset = imu.getCalibrationGyro();
        
        // 保存到Flash
        float offsets[6] = {
            accel_offset.x, accel_offset.y, accel_offset.z,  // 加速度计零偏
            gyro_offset.x, gyro_offset.y, gyro_offset.z      // 陀螺仪零偏
        };
        Flash_Write(offsets, 6, PAGE0);
        
        // 通过UART发送确认信息
        HAL_UART_Transmit(&huart2, (uint8_t *)"IMU offset updated", 18, 1000);
        HAL_UART_Transmit(&huart2, tail, 4, 1000);
    }
}

/**
 * @brief 陀螺仪Z轴校准按键任务
 */
void Task_key_gyrooffset(uint8_t runtime) {
    NON_BLOCK_DELAY(runtime);
    key_gyrooffset.proc();
}

/**
 * @brief 陀螺仪Z轴校准按键回调函数
 * @note 仅在调试模式下生效，用于单独校准Z轴陀螺仪
 */
void key_gyrooffset_callback() {
    if (debug_mode) {
        // 采集200次样本校准Z轴
        imu.calibrateZ(200);
        
        // 通过UART发送确认信息
        HAL_UART_Transmit(&huart2, (uint8_t *)"Gyro Z offset updated", 22, 1000);
        HAL_UART_Transmit(&huart2, tail, 4, 1000);
    }
}

// ==================== HAL回调函数 ====================

/**
 * @brief UART接收完成回调函数（处理调试命令、遥控）
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)  {
    if (huart == &huart2) {
        char msg[65];
        memset(msg, 0, sizeof(msg));

        if (debug_mode) {
            // ---------- 调试模式：完整命令解析 ----------
            sprintf(msg, "%6s", "rcved");
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, sizeof(msg), 1000);
            HAL_UART_Transmit(&huart2, tail, 4, 1000);
            
            float pidparams[3];
            
            // 解析命令和数据
            char *cmd = s_strtok((char *)UARTRcvBfr, ' ');
            char *data = s_strtok(NULL, ' ');
            
            // 参数数量检查
            if (s_strtok(NULL, ' ') != NULL) {
                sprintf(msg, "%15s", "too many args");
                HAL_UART_Transmit(&huart2, (uint8_t *)msg, sizeof(msg), 1000);
                HAL_UART_Transmit(&huart2, tail, 4, 1000);
                return;
            }

            // 命令标志枚举
            enum CmdFlag : uint8_t {
                CMD_NONE = 0,
                CMD_PIDT = 4,    // 角度环PID参数
                CMD_SPT = 9,     // 角度环设定值
                CMD_LIMITT = 5,  // 角度环限幅
                CMD_PIDV = 1,    // 速度环PID参数
                CMD_SPV = 2,     // 速度环设定值
                CMD_LIMITV = 3,  // 速度环限幅
                CMD_PIDR = 6,    // 旋转环PID参数
                CMD_SPR = 7,     // 旋转环设定值
                CMD_LIMITR = 8   // 旋转环限幅
            };
            
            uint8_t cmd_flag = CMD_NONE;
            
            // 命令解析
            if (cmd == NULL) {
                sprintf(msg, "error");
                HAL_UART_Transmit(&huart2, (uint8_t *)msg, sizeof(msg), 1000);
                HAL_UART_Transmit(&huart2, tail, 4, 1000);
                return;
            }
            // 角度环命令 (pid1/sp1/limit1)
            else if (strcmp(cmd, "pid1") == 0) {
                cmd_flag = CMD_PIDT;
            }
            else if (strcmp(cmd, "sp1") == 0) {
                cmd_flag = CMD_SPT;
            }
            else if (strcmp(cmd, "limit1") == 0) {
                cmd_flag = CMD_LIMITT;
            }
            // 速度环命令 (pid2/sp2/limit2)
            else if (strcmp(cmd, "pid2") == 0) {
                cmd_flag = CMD_PIDV;
            }
            else if (strcmp(cmd, "sp2") == 0) {
                cmd_flag = CMD_SPV;
            }
            else if (strcmp(cmd, "limit2") == 0) {
                cmd_flag = CMD_LIMITV;
            }
            // 旋转环命令 (pid3/sp3/limit3)
            else if (strcmp(cmd, "pid3") == 0) {
                cmd_flag = CMD_PIDR;
            }
            else if (strcmp(cmd, "sp3") == 0) {
                cmd_flag = CMD_SPR;
            }
            else if (strcmp(cmd, "limit3") == 0) {
                cmd_flag = CMD_LIMITR;
            }
            else {
                sprintf(msg, "%15s", "unknown cmd");
                HAL_UART_Transmit(&huart2, (uint8_t *)msg, sizeof(msg), 1000);
                HAL_UART_Transmit(&huart2, tail, 4, 1000);
                return;
            }
            
            // 解析数据参数（逗号分隔）
            char *token = s_strtok(data, ',');
            for (uint8_t i = 0; token != NULL; i++) {
                pidparams[i] = s_atof(token);
                token = s_strtok(NULL, ',');
            }

            // 执行命令
            switch (cmd_flag) {
                case CMD_PIDT:   // 角度环PID: pid1 kp,ki,kd
                    theta_pid.setFactors(pidparams[0], pidparams[1], pidparams[2]);
                    break;
                case CMD_SPT:    // 角度环设定值: sp1 value
                    theta_pid.setTarget(pidparams[0]);
                    break;
                case CMD_LIMITT: // 角度环限幅: limit1 max,min,ki_limit
                    theta_pid.setLimit(pidparams[0], pidparams[1], pidparams[2]);
                    break;
                    
                case CMD_PIDV:   // 速度环PID: pid2 kp,ki,kd
                    velocity_pid.setFactors(pidparams[0], pidparams[1], pidparams[2]);
                    break;
                case CMD_SPV:    // 速度环设定值: sp2 value
                    velocity_pid.setTarget(pidparams[0]);
                    break;
                case CMD_LIMITV: // 速度环限幅: limit2 max,min,ki_limit
                    velocity_pid.setLimit(pidparams[0], pidparams[1], pidparams[2]);
                    break;
                    
                case CMD_PIDR:   // 旋转环PID: pid3 kp,ki,kd
                    rotate_pid.setFactors(pidparams[0], pidparams[1], pidparams[2]);
                    break;
                case CMD_SPR:    // 旋转环设定值: sp3 value
                    rotate_pid.setTarget(pidparams[0]);
                    break;
                case CMD_LIMITR: // 旋转环限幅: limit3 max,min,ki_limit
                    rotate_pid.setLimit(pidparams[0], pidparams[1], pidparams[2]);
                    break;
            }
            
            // 保存到Flash并返回确认信息
            switch (cmd_flag) {
                case CMD_PIDT:   // 角度环参数更新
                case CMD_SPT:
                case CMD_LIMITT:
                {
                    float paramst[6] = {
                        theta_pid.K_p(),       // Kp
                        theta_pid.K_i(),       // Ki
                        theta_pid.K_d(),       // Kd
                        theta_pid.MaxOutput(), // 最大输出
                        theta_pid.MinOutput(), // 最小输出
                        theta_pid.KiLimit()    // 积分限幅
                    };
                    Flash_Write(paramst, 6, PAGE1);
                    char digit_msg[40] = "\0";
                    s_joinf(digit_msg, ',', paramst, 6, 3);
                    sprintf(msg, "pidt:%s", digit_msg);
                    break;
                }
                
                case CMD_PIDV:   // 速度环参数更新
                case CMD_SPV:
                case CMD_LIMITV:
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
                
                case CMD_PIDR:   // 旋转环参数更新
                case CMD_SPR:
                case CMD_LIMITR:
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

            // 清空接收缓冲区
            memset((uint8_t *)UARTRcvBfr, 0, sizeof(UARTRcvBfr));
            
            // 发送确认信息
            HAL_UART_Transmit(&huart2, (uint8_t *)msg, sizeof(msg), 1000);
            HAL_UART_Transmit(&huart2, tail, 4, 1000);
        }
        else {
            // ---------- 运行模式：仅支持速度和旋转设定值修改 ----------
            char *cmd = s_strtok((char *)UARTRcvBfr, ' ');
            float sp = s_atof(s_strtok(NULL, ' '));
            
            if (strcmp(cmd, "sp2") == 0) {
                velocity_pid.setTarget(sp);  // 设置速度设定值
            }
            else if (strcmp(cmd, "sp3") == 0) {
                rotate_pid.setTarget(sp);    // 设置旋转设定值
            }
            
            memset((uint8_t *)UARTRcvBfr, 0, sizeof(UARTRcvBfr));
        }
    }
    
    // 重新启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart2, (uint8_t *)UARTRcvBfr, sizeof(UARTRcvBfr));
    __HAL_DMA_DISABLE_IT(&hdma_usart2_rx, DMA_IT_HT);
}

/**
 * @brief UART发送完成回调函数
 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart2) {
        uart_flag = 1;  // 标记发送完成，允许下次发送
    }
}

/**
 * @brief GPIO外部中断回调函数
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_0) {
        motor_r_encoder.irq_handler();  // 右轮编码器中断
    }
    else if (GPIO_Pin == GPIO_PIN_1) {
        motor_l_encoder.irq_handler();  // 左轮编码器中断
    }
    else if (GPIO_Pin == GPIO_PIN_15) {
        imu_ready_flag = 1;  // IMU数据就绪中断
    }
}

/**
 * @brief I2C DMA接收完成回调函数
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    if (hi2c == &hi2c1) {
        // 处理IMU数据
        imu.readAccelGyro_IT_cplt_handler(imu_buffer, Accel, Gyro);
        
        // 计算欧拉角
        EurAngs = imu.getEulerAngles(Accel, Gyro);
        
        // 标记读取完成
        imu_cplt_flag = 1;
    }
}

#endif