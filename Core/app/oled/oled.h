#ifndef INC_OLED_H
#define INC_OLED_H

#include "font.h"
#include <stdint.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif
typedef struct{
    uint8_t x;
    uint8_t y;
}pointer;

#ifdef __cplusplus
}
class OLED{
public:
    OLED() = default;
    virtual ~OLED() = default;
    virtual void init() = 0;
    virtual void clear() = 0;
    virtual void showFrame() = 0;
    virtual void setPixel(uint8_t x, uint8_t y, uint8_t state) = 0;
    virtual void setPicture(uint8_t* picture, uint8_t width, uint8_t height, pointer ptr) = 0;
    virtual void setChar(char character, pointer ptr, const Font *font) = 0;
    virtual void setString(const char* string, pointer *ptr, const Font *font,
        uint8_t rspacing, uint8_t cspacing, bool backpointer = 1) = 0;
    virtual void setString(const char* string, pointer ptr, const Font *font,
        uint8_t rspacing, uint8_t cspacing) = 0;
    virtual uint16_t col() const = 0;
    virtual uint8_t page() const = 0;
protected:
    virtual void sendCmd(uint8_t cmd) = 0;
};

template<size_t COL, size_t PAGE>
class OLED_Algorithms : public OLED {
protected:
    uint8_t GRAM[PAGE][COL] = {};
public:
    OLED_Algorithms() : OLED(){}

    void clear() override;
    void setPixel(uint8_t x, uint8_t y, uint8_t state) override;
    void setPicture(uint8_t* picture, uint8_t width, uint8_t height, pointer ptr) override;
    void setChar(char character, pointer ptr, const Font *font) override;
    void setString(const char* string, pointer *ptr, const Font *font,
        uint8_t rspacing, uint8_t cspacing, bool backpointer = 1) override;
    void setString(const char* string, pointer ptr, const Font *font,
        uint8_t rspacing, uint8_t cspacing) override;
    
    uint16_t col() const override { return COL; }
    uint8_t page() const override { return PAGE; }
};

#ifndef OLED_IPP
#include "oled.ipp"
#endif

#endif /* __cplusplus */

#endif /* INC_OLED_H_ */