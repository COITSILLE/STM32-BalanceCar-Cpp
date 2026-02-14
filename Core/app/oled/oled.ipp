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
template<size_t COL, size_t PAGE>
void OLED_Algorithms<COL, PAGE>::setPixel(uint8_t x, uint8_t y, uint8_t state){
    if (x < 0 || x >= COL || y < 0 || y >= PAGE * 8 || (state != 0 && state != 1)) {
        return;
    }
    this->GRAM[y / 8][x] |= (state << (y % 8));
}

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
#endif

#endif /* OLED_IPP */
