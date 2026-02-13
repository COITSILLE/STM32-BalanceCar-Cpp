#ifndef OLED_IPP
#define OLED_IPP

#include "oled.h"

//Basic functions

#ifdef __cplusplus

//Screen control
/**
 *O--------> X
 *|
 *|
 *| 
 *↓ Y
*/
template<size_t COL, size_t PAGE>
void OLED_Algorithms<COL, PAGE>::clear(){
    memset(this->GRAM, 0x00, sizeof(this->GRAM));
}

//basic edit GRAM functions
/**
 * @brief Set a pixel in the GRAM buffer
 * @param x Range 0-127(GRAM[][x]), begins at top left
 * @param y Range 0-63(GRAM[y/8][x], bit y%8) begins at top left
 * @param state 1 to set pixel, 0 to clear pixel
 */
template<size_t COL, size_t PAGE>
void OLED_Algorithms<COL, PAGE>::setPixel(uint8_t x, uint8_t y, uint8_t state){
    if (x < 0 || x >= COL || y < 0 || y >= PAGE * 8 || (state != 0 && state != 1)) {
        return;
    }
    this->GRAM[y / 8][x] |= (state << (y % 8));
}

/**
 * @brief Display a monochrome bitmap on the OLED screen
 * 
 * This function renders a bitmap image stored in program memory to the OLED display.
 * The bitmap data is stored in a byte array where each byte represents 8 horizontal pixels
 * (MSB left, LSB right), and the array is organized in row-major order.
 * 
 * @param picture Pointer to the bitmap data array
 * @param width Width of the bitmap in bytes (actual pixel width = width * 8)
 * @param height Height of the bitmap in pixels (number of rows)
 * @param ptrseq Index of the position pointer in the ptr array
 */
template<size_t COL, size_t PAGE>
void OLED_Algorithms<COL, PAGE>::setPicture(uint8_t* picture, uint8_t width, uint8_t height, pointer ptr){
    uint8_t scale = height * width;

    for (uint8_t i = 0; i < scale; i++) {
        uint8_t x_offset = (i % width) * 8;
        uint8_t y_offset = i / width;
        uint8_t byte = picture[i];
        
        for (uint8_t bit = 0; bit < 8; bit++) {
            uint8_t value = (byte >> bit & 0x01);
            this->setPixel(ptr.x + x_offset + bit, ptr.y + y_offset, value);
        }
    }
}

//complex edit GRAM functions
template<size_t COL, size_t PAGE>
void OLED_Algorithms<COL, PAGE>::setChar(char character, pointer ptr, const Font *font){
    uint8_t scale = font->height * font->width;

    this->setPicture(font->letter + ((uint8_t)character - 32) * scale, 
                    font->width, font->height, ptr);
}

/**
 * @brief Set a string on the OLED screen
 * @attention string must consist of ascii numbers, alphabets, symbols, spaces, while
              only "\n" is supported
 * @param backpointer 1(default) to return pointer to original position after operation, 0 to keep at the end of string
*/
template<size_t COL, size_t PAGE>
void OLED_Algorithms<COL, PAGE>::setString(const char* string, pointer *ptr, const Font *font, uint8_t rspacing, uint8_t cspacing, bool backpointer){
    uint8_t width_r = font->width * 8;

    int8_t rspc_offset = width_r - rspacing;
    int8_t cspc_offset = font->height - cspacing;

    uint8_t len = strlen(string);
    
    uint8_t x_0 = ptr->x;
    uint8_t y_0 = ptr->y;
    
    if ((len * (width_r - rspc_offset)) > (this->col() * (this->page() * 8 / (font->height - cspc_offset)))) {
        return;
    }

    for (uint8_t i = 0; i < len; i++){
        if (string[i] != '\n'){
            this->setChar(string[i], *ptr, font);
            ptr->x += width_r - rspc_offset;
        }
        
        if (ptr->x + width_r - rspc_offset > this->col() || string[i + 1] == '\n'){
            ptr->x = x_0;
            ptr->y += font->height - cspc_offset;
        }
    }

    if (backpointer){
        ptr->x = x_0;
        ptr->y = y_0;
    }
}

template<size_t COL, size_t PAGE>
void OLED_Algorithms<COL, PAGE>::setString(const char* string, pointer ptr, const Font *font, uint8_t rspacing, uint8_t cspacing){
    pointer temp = ptr;
    this->setString(string, &temp, font, rspacing, cspacing, 0);
}
//show
#endif

#endif /* OLED_IPP */
