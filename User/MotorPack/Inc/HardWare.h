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
#define EN_GATE(X) X == 1 ? GPIO_SetBits(GPIOC,GPIO_Pin_9)  : GPIO_ResetBits(GPIOC,GPIO_Pin_9)
#define LED(X)     X == 1 ? GPIO_SetBits(GPIOA,GPIO_Pin_4)  : GPIO_ResetBits(GPIOA,GPIO_Pin_4)


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
void Uart_Init(void);
void USARTx_SendByte(USART_TypeDef* pUSARTx, uint8_t data);
uint8_t CAN_Send_Msg(uint32_t id,uint8_t *msg);