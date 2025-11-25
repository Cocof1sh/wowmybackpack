//超前钻这个题很多不会的地方怕记不住，所以打注释了>。<
#include "RM.h"
#include "task4_i2c_lib.h"

void IIC_Delay_ms(uint16_t ms)
{
     HAL_Delay(ms);
}
void Start(void)
{
    SDA_High(); // SDA 线置高电平（释放数据线）
    SCL_High();  // SCL 线置高电平（时钟线拉高）
    SDA_Low();  // 在 SCL 为高时 SDA 从高变低，形成起始条件
    SCL_Low();  // 时钟线拉低，准备数据传输
}

void Stop(void)
{
    SDA_Low(); // SDA 线置低电平
    SCL_High();  // SCL 线置高电平
    SDA_High();  // 在 SCL 为高时 SDA 从低变高，形成停止条件
}

void SendByte(unsigned char data)
{
    unsigned char i; // 先定义循环计数器
    for(i = 0; i < 8; i++)  // 循环8次，发送8位数据
    {
        if(data & 0x80) // 检查数据的最高位（即第7位）
            SDA_High();  // 最高位为1的话，则SDA置高
        else
            SDA_Low();  // 类似上面如果最高位为0，SDA置低
        SCL_High();  // 时钟线置高，从机读取数据位 //产生时钟上升沿，从机在此时采样数据
        SCL_Low();   // 时钟线置低，准备下一位数据传输  //时钟下降沿
        data <<= 1;  // 数据左移1位，准备发送下一位
    }
}

unsigned char ReceiveAck(void)
{
    unsigned char ack; // 定义变量存储应答状态
    SDA_High();  // 主机释放SDA线（置高），让从机控制
    SCL_High(); // 时钟线置高，从机采样数据
    ack = SDA_Read(); // 读取SDA线状态，即从机的应答信号
    SCL_Low();  // 时钟线置低，准备下一位数据传输
    return ack;  // 返回应答状态（0=应答、拉低SDA线，1=非应答、SDA线保持高电平）
}

void SendData(unsigned char data)
{
    Start(); // 发送起始条件
    SendByte(data); // 发送数据
    ReceiveAck(); // 接收应答
    Stop(); // 发送停止条件
    Stop();
}
//结束啦！！