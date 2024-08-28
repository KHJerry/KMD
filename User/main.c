/********************************** (C) COPYRIGHT *******************************
* File Name             : main.c
* Author                : HK
* Version               : V1.0.0
* Date                  : 2024/01/07
* Description           : HUE 智能车 气垫船 BLDC驱动
* Last Update Date      : 2024/03/28
*********************************************************************************/

#include "Interrupt.h"
#include "HardWare.h"

int main(void)
{

    //System Init
    SystemInit();
	SystemCoreClockUpdate();
    Interrupt_Configuration();
    Delay_Init();

    //USB Init
    USBFS_RCC_Init();
    USBFS_Device_Init(ENABLE);

    //GPIO Init
    MY_GPIO_Init();
    MY_TIM1_Init();
    MY_TIM3_Init();
    MY_ADC_Init();
    MY_CAN_Init();

    app_main();
    while(1);
}