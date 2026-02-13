// cpp_wrapper.h
#ifndef CPP_WRAPPER_H
#define CPP_WRAPPER_H

#ifdef __cplusplus
extern "C" {
#endif

// 需要的C语言头文件
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "sstrtotype.h"

#ifdef __cplusplus
}
// C++头文件
#include <array>
#include <cstdint>
#include "app_main.h"
#include "ssd1306.h"
#include "key.h"
#include "mpu6050.h"
#include "pwm.h"
#include "encoder.h"
#include "pid.h"
#include "tb6612.h"
#include "flash.h"
#include "samplefilter.h"
#endif



#endif /* __CPP_WRAPPER_H */