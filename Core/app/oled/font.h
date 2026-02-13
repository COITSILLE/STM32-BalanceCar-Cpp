#ifndef FONT_H
#define FONT_H

#include <stdint.h>

#define ascii16 &ASCII16
#define ascii12 &ASCII12
#define ascii8 &ASCII8

typedef struct{
    uint8_t *letter;
    uint8_t width;
    uint8_t height;
} Font;

extern const Font ASCII16;
extern const Font ASCII12;

#endif /*FONT_H*/