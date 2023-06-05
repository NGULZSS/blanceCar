/*这里是头文件*/
#ifndef __OLED_H_
#define __OLED_H_

#include "../Include/HardwareDriver/main.h"
#include "../Include/HardwareDriver/i2c.h"
extern I2C_HandleTypeDef hi2c2;
extern unsigned char bmp1[];

void oled_full(uint8_t data); //全屏填充
void oled_init(void); //初始化
void oled_show_char(uint8_t x,uint8_t y,uint8_t chr,uint8_t Char_Size);//单字节
void oled_show_string(uint8_t x, uint8_t y, char ch[], uint8_t TextSize); //输出字符串
void oled_show_chinese(uint8_t x,uint8_t y,uint8_t no); //输出汉字
void oled_show_bmp(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1, uint8_t bmp[]);//输出图像
void oled_clear(); //清屏
uint32_t oled_pow(uint8_t m,uint8_t n);
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size);
void OLED_ShowAllNum(uint8_t x,uint8_t y,int Anum,uint8_t len,uint8_t size);  //包含正负的整数
void OLED_ShowAllFloat(uint8_t x,uint8_t y,float Anum,uint8_t len,uint8_t size,uint8_t IFD); //显示小数  包含正负 IFD 是否有正负  1 有  0无
#endif  

