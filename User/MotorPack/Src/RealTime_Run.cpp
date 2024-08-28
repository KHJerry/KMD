/******************************************************************************************************
 *@file   : RealTime_Run.cpp
 *@brief  : 按照个人的理解就是,此文件用来分隔电机控制上层和底层,此文件就是底层的实现，实时性高，用来实时完成FOC的计算
 *@Author : HK
 *@note   : 结合个人理解和开源项目ODrive混合完成,感谢开源项目ODrive
 *@date   : 2023.10.02
 *@Update : 2024.05.13
 ******************************************************************************************************/

#ifdef __cplusplus

#include "HardWare.h"
#include "ch32v30x_usart.h"

#define PI               3.14159265358979f

#define Acc 1

float   vofa_float[9];
uint8_t vofa_data[40];

extern "C"{
#endif

void ADC1_2_IRQHandler(void)            __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM4_IRQHandler(void)              __attribute__((interrupt("WCH-Interrupt-fast")));
void USB_LP_CAN1_RX0_IRQHandler(void)   __attribute__((interrupt("WCH-Interrupt-fast")));
void USART1_IRQHandler(void)            __attribute__((interrupt("WCH-Interrupt-fast")));
#include "SMO.h"
#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

uint32_t timestamp_     = 0;

CanRxMsg CanRxStructure = {0,0,0,0,0,{0,0,0,0,0,0,0,0},0};

float pos1,pos2;
uint8_t p1[4],p2[4];
extern "C"{
#endif
u8 Start_Recv           = 0;
u8 USART_Rbuffer_Num    = 0;
u8 USART_Rbuffer[64];       //接收缓冲区数组
uint8_t mode            = 0;
uint8_t over            = 0;



    void OpenMV_ACK()
    {
//        uint8_t byte = '0';
//        USARTx_SendByte(USART1,byte);
    }

    void OpenMV_SendoneByte(uint8_t byte)
    {
        USARTx_SendByte(USART1,byte);
    }

    void xiaociandchongci(uint8_t mode)
    {
        uint8_t msg[] = {mode,0,0,0,0,0,0,0};
        CAN_Send_Msg(0x01,msg);
        Delay_Ms(100);
        OpenMV_ACK();
    }

    void USART1_IRQHandler(void)
    {
        if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)          //中断产生
        {
            USART_ClearITPendingBit(USART1,USART_IT_RXNE);             //清除中断标志
            uint8_t byte = USART_ReceiveData(USART1);                           //接收数据
            static uint8_t task_num = 0;
            static uint8_t index = 0;

            switch (task_num) {
                case 0:{
                    if(byte == 0XAA){
                        task_num++;
                    }else{
                        task_num=0;
                    }
                }break;

                case 1:{
                    if(byte == 0XAB){
                        task_num++;
                    }else{
                        task_num=0;
                    }
                }break;

                case 2:{
                    if(index < 10 ) {
                        USART_Rbuffer[index++] = byte;
                    }else{
                        index = 0;
                        task_num = 0;

                        p1[0] = USART_Rbuffer[0];
                        p1[1] = USART_Rbuffer[1];
                        p1[2] = USART_Rbuffer[2];
                        p1[3] = USART_Rbuffer[3];
                        p2[0] = USART_Rbuffer[4];
                        p2[1] = USART_Rbuffer[5];
                        p2[2] = USART_Rbuffer[6];
                        p2[3] = USART_Rbuffer[7];
                        mode  = USART_Rbuffer[8];

                        float *pp1 = (float*)p1;
                        float *pp2 = (float*)p2;

                        pos1 = *pp1-0.65f;    //- 0.923017f; // x轴
                        pos2 = *pp2+0.11f;;   // + 0.175866f;// + 0.144416f; // y轴

                        if(pos1 < 0) pos1 = 0;
                        else if(pos1 > W) pos1 = W;

                        if(pos2 <-H) pos2 = -H;
                        else if(pos2 > 0) pos2=0;

                        motor.pos_setpoint = (pos2);
                    }
                }break;
            }
        }
    }

    //C代码区域
    void ADC1_2_IRQHandler(void) {

        vofa_float[0]    = motor.vbus_measure;
        vofa_float[1]    = motor.currents.phA;
        vofa_float[2]    = motor.currents.phB;
        vofa_float[3]    = motor.currents.phC;
        vofa_float[4]    = motor.resistance;
        vofa_float[5]    = motor.inductance;
        vofa_float[6]    = motor.pos_circular_;//motor.phase_;
        vofa_float[7]    = 0;//motor.phase_observer_;
        vofa_float[8]    = CAN_GetLastErrorCode(CAN1);

        memcpy(vofa_data,(uint8_t*)vofa_float,sizeof(vofa_float));
        vofa_data[38] = 0X80;
        vofa_data[39] = 0X7F;
        USBFS_Endp_DataUp(DEF_UEP3,vofa_data,40,DEF_UEP_DMA_LOAD);

        if (ADC_GetFlagStatus(ADC1, ADC_FLAG_JEOC)) {
            if(TIM1->CH4CVR == 1)
            {
                TIM1->CH4CVR = TPWM - 2;

                motor.measure_currents(ADC1->IDATAR2,ADC1->IDATAR3,ADC1->IDATAR4);
                motor.Clark(motor.currents,motor.I_alpha_beta_);
                switch (motor.NowState) {
                    case Motor::STATE_NONE: {
                        motor.shut();
                    }break;
                    case Motor::STATE_ZERO:{
                        motor.go_zero();
                    }break;
                    case Motor::STATE_RESISTANCE_CALIBRATION: {
                        motor.measureResistance();
                    }break;
                    case Motor::STATE_INDUCTANCE_CALIBRATION: {
                        motor.measureInductance(timestamp_);
                    }break;
                    case Motor::STATE_ENCODER_CALIBRATION: {
                        motor.measureEncoder(timestamp_);
                    }break;
                    case Motor::STATE_CLOSELOOP: {
                        switch (motor.ctrlMode) {

                            case Motor::ControlMode::CTRL_POS: {
                                motor.positionLoop(timestamp_);
                            }break;

                            case Motor::ControlMode::CTRL_VEL: {
                                motor.velovityLoop(timestamp_);
                            }break;
                        }
                        motor.get_alpha_beta_output(timestamp_, &motor.mod_alpha_beta);
                        motor.SVM(motor.mod_alpha_beta.first, motor.mod_alpha_beta.second, &motor.CCRs_setpoint);
                    }break;
                }

//                Angle_SMOPare.Ialpha = motor.I_alpha_beta_.first;
//                Angle_SMOPare.Ibeta  = motor.I_alpha_beta_.second;
//                Angle_SMOPare.Valpha = -motor.mod_alpha_beta.first;
//                Angle_SMOPare.Vbeta  = -motor.mod_alpha_beta.second;
//
//                Angle_Cale(&Angle_SMOPare);

//                motor.phase_observer_ = atan2f(-Angle_SMOPare.Ealpha,Angle_SMOPare.Ebeta) + PI*(5.5f/4.f);
//                if(motor.phase_observer_ > PI) motor.phase_observer_-=2*PI;
//                float phase_observer_ = atan2f(-Angle_SMOPare.Ealpha,Angle_SMOPare.Ebeta) + PI*(5.0f/4.f);
//                if(phase_observer_ > PI) phase_observer_-=2*PI;
//                motor.phase_observer_ += current_meas_period * 5000 * (phase_observer_ - motor.phase_observer_last);
//                motor.phase_observer_last = motor.phase_observer_;

                motor.apply_pwm_timings();
            }
            else if(TIM1->CH4CVR == TPWM - 2)
            {
                TIM1->CH4CVR = 1;

                timestamp_++;
                motor.measurePhasePhaseVel();

                uint16_t vbus      = ADC1->IDATAR1;
                motor.ref_value[0] = ADC1->IDATAR2;
                motor.ref_value[1] = ADC1->IDATAR3;
                motor.ref_value[2] = ADC1->IDATAR4;

                float v = ((float(vbus) / 4096.f) * 3.3f * 11.f);
                motor.vbus_measure = v;
            }
            ADC_ClearFlag(ADC1,ADC_FLAG_JEOC);
        }
    }

    void TIM4_IRQHandler(void){

        if(TIM_GetFlagStatus(TIM4,TIM_FLAG_Update))
        {
            if(mode == 0 ){
                xiaociandchongci(0);
            }else if(mode == 1){
                xiaociandchongci(1);
            }

            TIM_ClearFlag(TIM4,TIM_FLAG_Update);
        }
    }

    void USB_LP_CAN1_RX0_IRQHandler(void)
    {

        if( CAN_GetITStatus( CAN1, CAN_IT_FMP0 ) != RESET )
        {
            CAN_Receive(CAN1,CAN_FIFO0,&CanRxStructure);

            if(CanRxStructure.StdId == 0X004)
            {
                float spd = CanRxStructure.Data[1];

                if(CanRxStructure.Data[0] == 0)
                {
                    spd*=-1;
                }

                motor.vel_setpoint_ = spd;
            }


        }
        CAN_ClearITPendingBit( CAN1, CAN_IT_FMP0 );
    }
#ifdef __cplusplus
}
#endif