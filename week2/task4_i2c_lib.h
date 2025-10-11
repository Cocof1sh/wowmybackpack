#ifndef I2C_LIB_H
#define I2C_LIB_H // 定义宏

void Start(void);
void Stop(void);
void SendByte(unsigned char data);
unsigned char ReceiveAck(void);
void SendData(unsigned char data);//声明一些要用到的函数

#endif