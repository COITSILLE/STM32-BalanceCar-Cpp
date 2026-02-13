#ifndef INC_SSD1306_H
#define INC_SSD1306_H

#include "i2c.h"
#include "oled.h"
#include <stdint.h>

#ifdef __cplusplus

class SSD1306_I2C : public OLED_Algorithms<128, 8>{
private:
    I2C_HandleTypeDef *_hi2c_;
    const uint16_t _address_;

    void sendCmd(uint8_t cmd) override;
    
public:
    void init() override;
    void showFrame() override;
    SSD1306_I2C(I2C_HandleTypeDef *hi2c, uint16_t address = 0x78)
        : OLED_Algorithms<128, 8>(), _hi2c_(hi2c), _address_(address){}
};

#endif


#endif
