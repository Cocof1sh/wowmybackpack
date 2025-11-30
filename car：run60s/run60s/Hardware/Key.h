#ifndef __KEY_H
#define __KEY_H

void Key_Init(void);//配置按键引脚为输入模式
uint8_t Key_GetNum(void); // 按键读取函数声明：检测按键是否按下，返回按键值（0=未按，1=已按

#endif
