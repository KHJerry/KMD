#include "ch32v30x.h"
#include "ch32v30x_adc.h"
#include "ch32v30x_tim.h"
#include "ch32v30x_gpio.h"
#include "ch32v30x_can.h"
#include "system_ch32v30x.h"
#include "ch32v30x_usbfs_device.h"
#include "ch32v30x_misc.h"
#include "ch32v30x_rcc.h"
#include "motor.h"

extern uint16_t ADC_Buff[2];


/**MCU‘s IO Configure**/
//Defines About LED And RGB
#define EN_GATE(X) X == 1 ? GPIO_SetBits(GPIOE,GPIO_Pin_2)  : GPIO_ResetBits(GPIOE,GPIO_Pin_2)
#define LED_R(X)   X == 0 ? GPIO_SetBits(GPIOE,GPIO_Pin_3)  : GPIO_ResetBits(GPIOE,GPIO_Pin_3)
#define LED_G(X)   X == 0 ? GPIO_SetBits(GPIOE,GPIO_Pin_4)  : GPIO_ResetBits(GPIOE,GPIO_Pin_4)
#define LED_B(X)   X == 0 ? GPIO_SetBits(GPIOE,GPIO_Pin_5)  : GPIO_ResetBits(GPIOE,GPIO_Pin_5)
#define M1_BK(X)   X == 0 ? GPIO_SetBits(GPIOB,GPIO_Pin_10) : GPIO_ResetBits(GPIOB,GPIO_Pin_10)
#define M2_BK(X)   X == 1 ? GPIO_SetBits(GPIOC,GPIO_Pin_4)  : GPIO_ResetBits(GPIOC,GPIO_Pin_4)
#define RGB_G(X)   TIM4->CH1CVR = X
#define RGB_R(X)   TIM4->CH2CVR = X
#define RGB_B(X)   TIM4->CH3CVR = X


void MY_GPIO_Init(void);

/**MCU‘s TIM Configure**/
void MY_TIM1_Init(void);    //MOTOR1   -- TIM1
void MY_TIM8_Init(void);    //MOTOR2   -- TIM8

void MY_TIM2_Init(void);    //ENCODER1 -- TIM2 support Motor1
void MY_TIM3_Init(void);    //ENCODER2 -- TIM3 support Motor2
void MY_TIM4_Init(void);    //RGB PWM Output

/**MCU‘s ADC Configure**/
void MY_ADC_Init(void);

void MY_CAN_Init(void);
uint8_t CAN_Send_Msg(uint32_t id,uint8_t *msg);