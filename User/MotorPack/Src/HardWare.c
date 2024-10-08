#include "HardWare.h"
#include "ch32v30x_gpio.h"
#include "ch32v30x_adc.h"
#include "ch32v30x_dma.h"
#include "ch32v30x_usart.h"
#include "ch32v30x_tim.h"

#define DEADTIME 1

void MY_GPIO_Init()
{
    /***********************
     * EN_GATE AND RGB GPIO
     * PE2 -- EN_GATE
     * PE3 -- Blue
     * PA4 -- White
     * PE5 -- Red
     **********************/
    GPIO_InitTypeDef GPIO_InitStructure;
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA , ENABLE);
    GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_4;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);//配置寄存器实体

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOC , ENABLE);
    GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_Init(GPIOC, &GPIO_InitStructure);//配置寄存器实体

    EN_GATE(0);
    LED(1);
}


void MY_TIM1_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;
    GPIO_InitTypeDef        GPIO_InitStructure;
    TIM_BDTRInitTypeDef     TIM_BDTRInitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB  |   RCC_APB2Periph_AFIO | RCC_APB2Periph_TIM1 , ENABLE);

    /***************
     * PA8  --  AL1
     * PA9  --  AH1
     * PA10 --  BL1
     * PE11 --  BH1
     * PE12 --  CL1
     * PE13 --  CH1
     **************/
    GPIO_StructInit(&GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_8 | GPIO_Pin_9  | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_13 | GPIO_Pin_14  | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    TIM_TimeBaseStructure.TIM_Prescaler         = 0;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_CenterAligned3;
    TIM_TimeBaseStructure.TIM_Period            = TPWM - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 1;

    //MOTOR1
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);    //配置寄存器实体
    TIM_OCStructInit(&TIM_OCInitStructure);                      //比较器初始化用结构体的初始化
    TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM2;                    //大于比较寄存器值为有效，否则为无效
    TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;             //正向通道比较输出使能
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;            //互补通道比较输出使能
    TIM_OCInitStructure.TIM_Pulse        = TPWM / 2;                           //dummy value
    TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;                //正向通道比较输出高电平有效
    TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_High;               //互补通道比较输出高电平有效 PU预驱
    TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;              //正向通道空闲状态为低电平
    TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;             //互补通道空闲状态为低电平 PU预驱
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);               //捕获比较通道1初始化
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);               //捕获比较通道2初始化
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);               //捕获比较通道3初始化
    TIM_OC4Init(TIM1, &TIM_OCInitStructure);               //捕获比较通道3初始化
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);       //使能捕获比较通道1的预装载
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);       //使能捕获比较通道2的预装载
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);       //使能捕获比较通道3的预装载
    TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);       //使能捕获比较通道4的预装载

    //刹车死区相关配置
    TIM_BDTRInitStructure.TIM_OSSRState         = TIM_OSSRState_Enable;        //OSSR使能
    TIM_BDTRInitStructure.TIM_OSSIState         = TIM_OSSIState_Enable;        //OSSI使能
    TIM_BDTRInitStructure.TIM_LOCKLevel         = TIM_LOCKLevel_1;             //寄存器配置锁定级别为1
    TIM_BDTRInitStructure.TIM_DeadTime          = DEADTIME;                    //设置死区时间
    TIM_BDTRInitStructure.TIM_Break             = TIM_Break_Disable;           //刹车使能
    TIM_BDTRInitStructure.TIM_BreakPolarity     = TIM_BreakPolarity_High;      //刹车信号极性配置为高有效
    TIM_BDTRInitStructure.TIM_AutomaticOutput   = TIM_AutomaticOutput_Enable;  //自动输出禁止

    //配置寄存器实体
    TIM_BDTRConfig(TIM1, &TIM_BDTRInitStructure);

    TIM_SelectOutputTrigger(TIM1, TIM_TRGOSource_Update);  //选择更新事件作为外部触发源
    TIM_SetCounter(TIM1,0);                                       //定时器计数器清零
    TIM_ClearFlag(TIM1, TIM_FLAG_Update);                       //清除更新事件中断标志
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);           //使能更新事件的中断
    TIM_ITConfig(TIM1,TIM1_CC_IRQn,ENABLE);

    TIM_Cmd(TIM1,ENABLE);                                       //使能TIM1
    TIM_CtrlPWMOutputs(TIM1,ENABLE);                            //PWM输出使能
    TIM1->CH4CVR = 1;
}

void MY_TIM3_Init(void)
{
    GPIO_InitTypeDef        GPIO_InitStructure;
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_ICInitTypeDef       TIM_ICInitStructure;

    //使能相应时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3 , ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB | RCC_APB2Periph_AFIO , ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING; //浮空输入
    GPIO_Init(GPIOB, &GPIO_InitStructure);


    GPIO_PinRemapConfig(GPIO_PartialRemap_TIM3,ENABLE);

    //定时器初始化配置
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period        = 4095;                          //计数器自动重装载值
    TIM_TimeBaseStructure.TIM_Prescaler     = 0;                             //预分频器值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;                  //时钟分频
    TIM_TimeBaseStructure.TIM_CounterMode   = TIM_CounterMode_Up;            //向上计数模式
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;                         //重复计数器值
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //初始化结构体

    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising); //使用编码器模式3

    //输入捕获配置
    TIM_ICInitStructure.TIM_Channel     = TIM_Channel_1|TIM_Channel_2;
    TIM_ICInitStructure.TIM_ICPolarity  = TIM_ICPolarity_Rising;      //输入捕获极性设置，可用于配置编码器正反相
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;             //输入捕获预分频器设置
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;   //输入捕获通道选择，编码器模式需选用此配置
    TIM_ICInitStructure.TIM_ICFilter    = 10;                         //输入捕获滤波器设置
    TIM_ICInit(TIM3, &TIM_ICInitStructure);

    TIM_ClearFlag(TIM3, TIM_FLAG_Update);               //清除TIM更新标志位
    TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);   //使能开启TIM中断

    //Reset counter
    TIM_SetCounter(TIM3,0);
    TIM_Cmd(TIM3, ENABLE);

}

void MY_TIM4_Init(void)
{
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM4 , ENABLE);
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOD | RCC_APB2Periph_AFIO, ENABLE);

    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Pin   =  GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14;
    GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    GPIO_PinRemapConfig(GPIO_Remap_TIM4,ENABLE);

    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_OCInitTypeDef       TIM_OCInitStructure;

    TIM_TimeBaseStructure.TIM_Prescaler         = 144 - 1;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period            = 1000 - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);    //配置寄存器实体



    TIM_OCStructInit(&TIM_OCInitStructure);                //比较器初始化用结构体的初始化
    TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM2;              //大于比较寄存器值为有效，否则为无效
    TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;       //正向通道比较输出使能
    TIM_OCInitStructure.TIM_Pulse        = 0;                            //dummy value
    TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;          //正向通道比较输出高电平有效
    TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Reset;        //正向通道空闲状态为低电平
    TIM_OC1Init(TIM4, &TIM_OCInitStructure);         //捕获比较通道1初始化
    TIM_OC2Init(TIM4, &TIM_OCInitStructure);         //捕获比较通道2初始化
    TIM_OC3Init(TIM4, &TIM_OCInitStructure);         //捕获比较通道3初始化
    TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Disable); //使能捕获比较通道1的预装载
    TIM_OC2PreloadConfig(TIM4, TIM_OCPreload_Disable); //使能捕获比较通道2的预装载
    TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Disable); //使能捕获比较通道3的预装载

    TIM_ARRPreloadConfig(TIM4,ENABLE);

    TIM_SelectOutputTrigger(TIM4, TIM_TRGOSource_Update);   //选择更新事件作为外部触发源
    TIM_SetCounter(TIM4,0);                                        //定时器计数器清零

    TIM_ClearFlag(TIM4, TIM_FLAG_Update);                        //清除更新事件中断标志
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);             //使能更新事件的中断

    TIM_Cmd(TIM4,ENABLE);
}


void MY_ADC_Init(void) {

    //初始化结构体
    ADC_InitTypeDef ADC_InitStructure;
    GPIO_InitTypeDef GPIO_InitStructure;

    //频率设置
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);//ADC时钟为PCLK2的六分频
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO | RCC_APB2Periph_ADC1 , ENABLE);

    //关闭内部温度ADC
    ADC_TempSensorVrefintCmd(DISABLE);

    /******************************
     * 母线电压采样(过压保护 && 低压保护)
     * 三相电流采样
     * PA0 -- VBUS -- Channel0
     * PA1 -- IA   -- Channel1
     * PA2 -- IB   -- Channel2
     * PA3 -- IC   -- Channel3
    ********************************/

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    ADC_DeInit(ADC1);

    ADC_InitStructure.ADC_Mode                  = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode          = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode    = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv      = ADC_ExternalTrigInjecConv_T1_CC4;
    ADC_InitStructure.ADC_DataAlign             = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel          = 4;
    ADC_InitStructure.ADC_OutputBuffer          = ADC_OutputBuffer_Disable;
    ADC_InitStructure.ADC_Pga                   = ADC_Pga_1;

    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_InjectedSequencerLengthConfig(ADC1, 4);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_0, 1, ADC_SampleTime_7Cycles5);  //VBUS
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_1, 2, ADC_SampleTime_7Cycles5);  //IA
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_2, 3, ADC_SampleTime_7Cycles5);  //IB
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_3, 4, ADC_SampleTime_7Cycles5);  //IC

    ADC_ITConfig( ADC1, ADC_IT_JEOC, ENABLE);
    ADC_DMACmd(   ADC1, DISABLE);
    ADC_Cmd(      ADC1, ENABLE);
    ADC_BufferCmd(ADC1,DISABLE);
}

void MY_CAN_Init(void)
{
    GPIO_InitTypeDef        GPIO_InitSturcture;
    CAN_InitTypeDef         CAN_InitSturcture;
    CAN_FilterInitTypeDef   CAN_FilterInitSturcture;

    RCC_APB2PeriphClockCmd( RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOB, ENABLE );
    RCC_APB1PeriphClockCmd( RCC_APB1Periph_CAN1, ENABLE );

    GPIO_PinRemapConfig(GPIO_Remap1_CAN1,ENABLE);



    GPIO_InitSturcture.GPIO_Pin     = GPIO_Pin_8;
    GPIO_InitSturcture.GPIO_Mode    = GPIO_Mode_IPU;
    GPIO_InitSturcture.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitSturcture);

    GPIO_InitSturcture.GPIO_Pin     = GPIO_Pin_9;
    GPIO_InitSturcture.GPIO_Mode    = GPIO_Mode_AF_PP;
    GPIO_InitSturcture.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_Init( GPIOB, &GPIO_InitSturcture);



    CAN_InitSturcture.CAN_TTCM      = DISABLE;
    CAN_InitSturcture.CAN_ABOM      = DISABLE;
    CAN_InitSturcture.CAN_AWUM      = DISABLE;
    CAN_InitSturcture.CAN_NART      = DISABLE;
    CAN_InitSturcture.CAN_RFLM      = DISABLE;
    CAN_InitSturcture.CAN_TXFP      = DISABLE;
    CAN_InitSturcture.CAN_Mode      = CAN_Mode_Normal;
    CAN_InitSturcture.CAN_SJW       = CAN_SJW_1tq;
    CAN_InitSturcture.CAN_BS1       = CAN_BS1_6tq;
    CAN_InitSturcture.CAN_BS2       = CAN_BS2_5tq;
    CAN_InitSturcture.CAN_Prescaler = 12;
    CAN_Init( CAN1, &CAN_InitSturcture );

    CAN_FilterInitSturcture.CAN_FilterNumber            = 0;
    CAN_FilterInitSturcture.CAN_FilterMode              = CAN_FilterMode_IdMask;
    CAN_FilterInitSturcture.CAN_FilterScale             = CAN_FilterScale_32bit;
    CAN_FilterInitSturcture.CAN_FilterIdHigh            = 0x0000;
    CAN_FilterInitSturcture.CAN_FilterIdLow             = 000000;
    CAN_FilterInitSturcture.CAN_FilterMaskIdHigh        = 0x0000;
    CAN_FilterInitSturcture.CAN_FilterMaskIdLow         = 0x0000;
    CAN_FilterInitSturcture.CAN_FilterFIFOAssignment    = CAN_Filter_FIFO0;
    CAN_FilterInitSturcture.CAN_FilterActivation        = ENABLE;
    CAN_FilterInit( &CAN_FilterInitSturcture);

    CAN_ITConfig( CAN1, CAN_IT_FMP0, ENABLE);
}

uint8_t CAN_Send_Msg(uint32_t id,uint8_t *msg)
{
    static CanTxMsg CanTxStructure;
    CanTxStructure.StdId = id;
    CanTxStructure.IDE   = 0;
    CanTxStructure.RTR   = 0;
    CanTxStructure.DLC   = 8;

    for(int i=0; i<8; i++ )
    {
        CanTxStructure.Data[i] = msg[i];
    }

    uint8_t mbox = CAN_Transmit( CAN1, &CanTxStructure);

    return 1;
}
void Uart_Init(void)
{
    GPIO_InitTypeDef   GPIO_InitStructure={0};
    USART_InitTypeDef  USART_InitStructure={0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_USART1,ENABLE);

    /* USART1 TX-->PA9   RX-->PA10 */
    GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_9;
    GPIO_InitStructure.GPIO_Speed   = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_AF_PP;              //设置PA9为复用推挽输出
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin     = GPIO_Pin_10;
    GPIO_InitStructure.GPIO_Mode    = GPIO_Mode_IN_FLOATING;        //设置PA10为浮空输入
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART_InitStructure.USART_BaudRate              = 9600;
    USART_InitStructure.USART_WordLength            = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits              = USART_StopBits_1;
    USART_InitStructure.USART_Parity                = USART_Parity_No;
    USART_InitStructure.USART_HardwareFlowControl   = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                  = USART_Mode_Tx | USART_Mode_Rx;

    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);
    USART_Cmd(USART1,ENABLE);
}

void USARTx_SendByte(USART_TypeDef* pUSARTx, uint8_t data)
{
    USART_SendData(pUSARTx, data);
    while(USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);
}

void USARTx_SendStr(USART_TypeDef* pUSARTx, char *str)
{
    uint8_t i = 0;
    do
    {
        USARTx_SendByte(pUSARTx, *(str+i));
        i++;
    }while(*(str+i) != '\0');
    while(USART_GetFlagStatus(pUSARTx, USART_FLAG_TC) == RESET);
}

