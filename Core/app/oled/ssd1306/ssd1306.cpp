#include "ssd1306.h"

void SSD1306_I2C::sendCmd(uint8_t cmd) {
    uint8_t SendBuffer[2];
    SendBuffer[0] = 0x00; // Control byte for command
    SendBuffer[1] = cmd;
    HAL_I2C_Master_Transmit(this->hi2c, this->address, SendBuffer, 2, HAL_MAX_DELAY);
}

void SSD1306_I2C::init(){
    memset(this->GRAM, 0, sizeof(this->GRAM));
    HAL_Delay(50);

    sendCmd(0xAE);

    sendCmd(0xD5);
    sendCmd(0x80);

    sendCmd(0xA8);
    sendCmd(0x3F);

    sendCmd(0xD3);
    sendCmd(0x00);

    sendCmd(0x40);

    sendCmd(0xA1);

    sendCmd(0xC8);

    sendCmd(0xDA);
    sendCmd(0x12);
    sendCmd(0x81);
    sendCmd(0xCF);

    sendCmd(0xD9);
    sendCmd(0xF1);

    sendCmd(0xDB);
    sendCmd(0x30);

    sendCmd(0xA4);

    sendCmd(0xA6);

    sendCmd(0x8D);
    sendCmd(0x14);

    sendCmd(0xAF);

    HAL_Delay(100);
}

void SSD1306_I2C::showFrame(){
    uint8_t SendBuffer[this->col() + 1];
    SendBuffer[0] = 0x40;
    for (uint8_t i = 0; i < this->page(); i++){
        for (uint8_t j = 0; j < this->col(); j++){
            SendBuffer[j + 1] = this->GRAM[i][j];
        }
        sendCmd(0xB0 + i);
        sendCmd(0x00);
        sendCmd(0x10);
        HAL_I2C_Master_Transmit(this->hi2c, this->address, SendBuffer, this->col() + 1, HAL_MAX_DELAY);
    }
}