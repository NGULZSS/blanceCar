#ifndef __PRINT_H
#define __PRINT_H
#include "../Include/HardwareDriver/main.h"
#include <stdio.h>

extern UART_HandleTypeDef huart1;
#define Print_UART huart1

int fputc(int ch, FILE *f);
int fgetc(FILE *f);
#endif
