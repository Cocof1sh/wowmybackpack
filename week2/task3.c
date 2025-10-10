#include <stdio.h>
typedef enum 
{
	GPIO_Speed_2MHz,
    GPIO_Speed_10MHz,
    GPIO_Speed_50MHz
}                   //枚举
GPIO_Speed_ENUM;//枚举类型的定义
typedef struct 
{
	GPIO_Speed_ENUM GPIO_Speed;//搞一个结构体的定义，前面是成员，后面是前面定义的枚举类型
} 
GPIO_InitTypeDef;

void GPIO_StructuredInit(GPIO_InitTypeDef *GPIO_InitStruct)
//最重要的函数，指针指向结构体
{
     GPIO_InitStruct->GPIO_Speed = GPIO_Speed_2MHz;//给默认值
}

int main()
{
	GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_StructuredInit(&GPIO_InitStruct);//调用函数，获取地址给函数
printf("GPIO Speed default value set to: %d\n", GPIO_InitStruct.GPIO_Speed);
    return 0;
} 

