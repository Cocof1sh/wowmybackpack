#include "stm32f10x.h"                  // Device header
#include "PWM.h"                        
#include "Delay.h"                       

// 小车初始化函数
void robot_Init(void)
{
	PWM_Init(); 
}

// 电机速度控制，直接设置4个PWM通道速度
void robot_speed(uint8_t left1_speed,uint8_t left2_speed,uint8_t right1_speed,uint8_t right2_speed)
{	
	TIM_SetCompare1(TIM4,left1_speed); // 左电机通道1（TIM4_CH1）速度设置
	TIM_SetCompare2(TIM4,left2_speed); // 左电机通道2（TIM4_CH2）速度设置
	TIM_SetCompare3(TIM4,right1_speed); // 右电机通道1（TIM4_CH3）速度设置
	TIM_SetCompare4(TIM4,right2_speed); // 右电机通道2（TIM4_CH4）速度设置
}

// 小车前进函数
void makerobo_run(int8_t speed,uint16_t time)  
{
	if(speed > 100) // 速度上限限制（PWM最大占空比对应100）
	{
		speed = 100;
	}
	if(speed < 0) // 速度下限限制
	{
		speed = 0;
	}
	robot_speed(speed,0,speed,0); // 左1、右1通道调速（前进方向），左2、右2关
	Delay_ms(time); // 持续前进time毫秒
}

// 小车刹车函数
void makerobo_brake(uint16_t time) 
{
	robot_speed(0,0,0,0); // 所有电机通道速度设0，停止转
	Delay_ms(time); // 保持刹车time毫秒
}

// 小车普通左转函数（单轮驱动）
void makerobo_Left(int8_t speed,uint16_t time) 
{
	if(speed > 100) // 速度上限限制
	{
		speed = 100;
	}
	if(speed < 0) // 速度下限限制
	{
		speed = 0;
	}
	robot_speed(0,0,speed,0); // 左轮停，右轮调速（左转）
	Delay_ms(time); // 持续左转time毫秒
}

// 小车原地左旋函数
void makerobo_Spin_Left(int8_t speed,uint16_t time) 
{
	if(speed > 100) // 速度上限限制
	{
		speed = 100;
	}
	if(speed < 0) // 速度下限限制
	{
		speed = 0;
	}  
	robot_speed(0,speed,speed,0); // 左轮反转、右轮正转（原地左旋）
	Delay_ms(time); // 持续左旋time毫秒
}

// 小车普通右转函数（单轮驱动）
void makerobo_Right(int8_t speed,uint16_t time)
{
	if(speed > 100) // 速度上限限制
	{
		speed = 100;
	}
	if(speed < 0) // 速度下限限制
	{
		speed = 0;
	}
	robot_speed(speed,0,0,0); // 右轮停，左轮调速（右转）
	Delay_ms(time); // 持续右转time毫秒
}

// 小车原地右旋函数
void makerobo_Spin_Right(int8_t speed,uint16_t time) 
{
	if(speed > 100) // 速度上限限制
	{
		speed = 100;
	}
	if(speed < 0) // 速度下限限制
	{
		speed = 0;
	}  
	robot_speed(speed,0,0,speed); // 左轮正转、右轮反转（原地右旋）
	Delay_ms(time); // 持续右旋time毫秒
}

// 小车后退函数
void makerobo_back(int8_t speed,uint16_t time) 
{
	if(speed > 100) // 速度上限限制
	{
		speed = 100;
	}
	if(speed < 0) // 速度下限限制
	{
		speed = 0;
	}
	robot_speed(0,speed,0,speed); // 左2、右2通道调速（后退方向），左1、右1关
	Delay_ms(time); // 持续后退time毫秒
}
