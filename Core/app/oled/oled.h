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
    /**
     * @brief Clear the GRAM buffer
     * @note This function only clears the GRAM buffer. You need to call showFrame() to update the display after calling this function.
     */
    void clear() override;
    /**
     * @brief Set a pixel in the GRAM buffer
     * @param x Range 0-127 (GRAM[][x]), begins at top left
     * @param y Range 0-63 (GRAM[y/8][x], bit y%8) begins at top left
     * @param state 1 to set pixel, 0 to clear pixel
     */
    void setPixel(uint8_t x, uint8_t y, uint8_t state) override;
    /**
     * @brief Display a monochrome bitmap on the OLED screen
     * @param picture Pointer to the bitmap data array
     * @param width Width of the bitmap in bytes (actual pixel width = width * 8)
     * @param height Height of the bitmap in pixels (number of rows)
     * @param ptr Position pointer for the top-left corner of the bitmap
     * @note The bitmap data is stored in row-major order where each byte represents 8 horizontal pixels
     */
    void setPicture(uint8_t* picture, uint8_t width, uint8_t height, pointer ptr) override;
    void setChar(char character, pointer ptr, const Font *font) override;
    /**
     * @brief Set a string on the OLED screen with pointer tracking
     * @param string String to display (ASCII characters + '\n' for newline)
     * @param ptr Pointer to position, will be updated during operation
     * @param font Font to use for rendering
     * @param rspacing Horizontal spacing between characters
     * @param cspacing Vertical spacing between lines
     * @param backpointer If true (default), returns pointer to original position after operation
     * @attention String must consist of ASCII numbers, alphabets, symbols, spaces; only '\n' is supported for newline
     */
    void setString(const char* string, pointer *ptr, const Font *font,
        uint8_t rspacing, uint8_t cspacing, bool backpointer = 1) override;
        /**
     * @brief Set a string on the OLED screen with pointer tracking
     * @param string String to display (ASCII characters + '\n' for newline)
     * @param ptr Pointer to position, will be updated during operation
     * @param font Font to use for rendering
     * @param rspacing Horizontal spacing between characters
     * @param cspacing Vertical spacing between lines
     * @attention String must consist of ASCII numbers, alphabets, symbols, spaces; only '\n' is supported for newline
     */
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