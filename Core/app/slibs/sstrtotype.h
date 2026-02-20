#ifndef SSTRTOTYPE_H
#define SSTRTOTYPE_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

char* s_strtok(char *str, const char delim);
uint8_t s_ctoi(const char num);
float s_atof(const char *str);
int32_t s_atoi(const char *str);
void s_reverse(char *str, uint16_t len);
uint32_t s_itoa(const int32_t num, char *str, const uint8_t base);
uint32_t s_ftoa(const float num, char *str, const uint8_t precision);
void s_joinf(char *str, const char delim, const float *args, uint16_t argc, uint8_t precision);

#ifdef __cplusplus
}
#endif

#endif