#ifndef __ROBOT_H  
#define __ROBOT_H  

void robot_Init(void);

//电机速度直接控制函数：手动设置4个电机通道的PWM速度（左1/左2/右1/右2，对应TIM4_CH1~CH4）
//left1_speed=左电机通道1速度，left2_speed=左电机通道2速度，right1_speed=右电机通道1速度，right2_speed=右电机通道2速度（速度范围0~99）
void robot_speed(uint8_t left1_speed,uint8_t left2_speed,uint8_t right1_speed,uint8_t right2_speed);

//小车前进函数：控制小车向前行驶
//speed=前进速度（0~99），time=行驶持续时间（ms，0表示一直行驶）
void makerobo_run(uint8_t speed,uint16_t time);

//小车刹车函数：控制小车停止行驶
//    time=刹车持续时间（ms，0表示一直刹车）
void makerobo_brake(uint16_t time);            

//  小车左转函数：普通左转（单轮驱动：左轮停、右轮转
//   speed=右转轮速度（0~99），time=左转持续时间（ms）
void makerobo_Left(int8_t speed,uint16_t time); 

// 小车左旋函数：原地左旋（左轮反转、右轮正转，原地转
//speed=电机速度（0~99），time=左旋持续时间（ms）
void makerobo_Spin_Left(int8_t speed,uint16_t time);  

//  小车右转函数：普通右转（单轮驱动：右轮停、左轮转
//    speed=左转轮速度（0~99），time=右转持续时间（ms）
void makerobo_Right(int8_t speed,uint16_t time);  

// 小车右旋函数：原地右旋（右轮反转、左轮正转，原地转圈
//    speed=电机速度（0~99），time=右旋持续时间（ms）
void makerobo_Spin_Right(int8_t speed,uint16_t time); 

//  小车后退函数：控制小车向后行驶（双轮反转）
//    speed=后退速度（0~99），time=后退持续时间（ms，0表示一直后退）
//没见过，都是直走哈哈哈
void makerobo_back(int8_t speed,uint16_t time);      

#endif 
