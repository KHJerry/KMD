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

#define Acc 1

float   vofa_float[14];
uint8_t vofa_data[60];

extern "C"{
#endif

void ADC1_2_IRQHandler(void)            __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_UP_IRQHandler(void)           __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM8_UP_IRQHandler(void)           __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM4_IRQHandler(void)              __attribute__((interrupt("WCH-Interrupt-fast")));
void USB_LP_CAN1_RX0_IRQHandler(void)   __attribute__((interrupt("WCH-Interrupt-fast")));

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

uint32_t timestamp_      = 0;
uint32_t timestamp1_     = 0;
uint32_t timestamp2_     = 0;
uint32_t ms_delay_cnt    = 0;
uint32_t RGB_timestamp_  = 0;
uint16_t error_ibus_cnt  = 0;
uint16_t error_vbus_cnt  = 0;
uint32_t repeatTime      = 0;
float last_vel_set1      = 0;
float last_vel_set2      = 0;

CanRxMsg CanRxStructure = {0,0,0,0,0,{0,0,0,0,0,0,0,0},0};


extern "C"{
#endif
    //C代码区域
    void ADC1_2_IRQHandler(void) {

        if (ADC_GetFlagStatus(ADC1, ADC_FLAG_JEOC)) {
            motor1.ref_value[0] = 2058;
            motor1.ref_value[1] = 2058;

            uint16_t vbus = ADC1->IDATAR3;
            float v = ((float(vbus) / 4096.f) * 3.3f * 11.f);
            motor1.vbus_measure = motor2.vbus_measure = v;

            motor1.measure_currents(ADC1->IDATAR1,ADC1->IDATAR2,0);
            motor1.Clark(motor1.currents,motor1.I_alpha_beta_);
            motor1.ibus = motor1.Iq_measured_;

            switch (motor1.NowState) {

                case Motor::STATE_NONE: {
                    motor1.shut();
                }break;

                case Motor::STATE_ZERO:{
                    motor1.go_zero();
                }break;

                case Motor::STATE_RESISTANCE_CALIBRATION: {
                    motor1.measureResistance();
                }break;

                case Motor::STATE_INDUCTANCE_CALIBRATION: {
                    motor1.measureInductance(timestamp_);
                }break;

                case Motor::STATE_ENCODER_CALIBRATION: {
                    motor1.measureEncoder(timestamp_);
                }break;

                case Motor::STATE_CLOSELOOP: {
                    switch (motor1.ctrlMode) {
                        case Motor::ControlMode::CTRL_VEL: {
                            motor1.velovityLoop(timestamp_);
                        }break;

                        case Motor::ControlMode::CTRL_POS: {
                            motor1.positionLoop(timestamp_);
                        }break;
                    }

                    motor1.get_alpha_beta_output(timestamp_, &motor1.mod_alpha_beta);
                    motor1.SVM(motor1.mod_alpha_beta.first, motor1.mod_alpha_beta.second, &motor1.CCRs_setpoint);
                }break;
            }
            motor1.apply_pwm_timings();
            ADC_ClearFlag(ADC1,ADC_FLAG_JEOC);
        }

        if (ADC_GetFlagStatus(ADC2, ADC_FLAG_JEOC)) {

            motor2.ref_value[0] = 2058;
            motor2.ref_value[1] = 2058;

            motor2.measure_currents(ADC2->IDATAR1,ADC2->IDATAR2,0);
            motor2.Clark(motor2.currents,motor2.I_alpha_beta_);
            motor2.ibus = motor2.Iq_measured_;

            switch (motor2.NowState) {
                case Motor::STATE_NONE: {
                    motor2.shut();
                }break;

                case Motor::STATE_ZERO:{
                    motor2.go_zero();
                }break;

                case Motor::STATE_RESISTANCE_CALIBRATION: {
                    motor2.measureResistance();
                }break;

                case Motor::STATE_INDUCTANCE_CALIBRATION: {
                    motor2.measureInductance(timestamp_);
                }break;

                case Motor::STATE_ENCODER_CALIBRATION: {
                    motor2.measureEncoder(timestamp_);
                }break;

                case Motor::STATE_CLOSELOOP: {
                    switch (motor2.ctrlMode) {

                        case Motor::ControlMode::CTRL_VEL: {
                            motor2.velovityLoop(timestamp_);
                        }break;

                        case Motor::ControlMode::CTRL_POS: {
                            motor2.positionLoop(timestamp_);
                        }break;

                    }
                    motor2.get_alpha_beta_output(timestamp_, &motor2.mod_alpha_beta);
                    motor2.SVM(motor2.mod_alpha_beta.first, motor2.mod_alpha_beta.second, &motor2.CCRs_setpoint);
                }break;
            }
            motor2.apply_pwm_timings();
            ADC_ClearFlag(ADC2,ADC_FLAG_JEOC);
        }
    }

    //用来测速和计时
    void TIM1_UP_IRQHandler(void) {
        if (TIM_GetFlagStatus(TIM1, TIM_FLAG_Update)) {
            timestamp_++;
            motor1.measurePhasePhaseVel();
            motor2.measurePhasePhaseVel();
            TIM_ClearFlag(TIM1, TIM_FLAG_Update);
        }
    }

    void TIM4_IRQHandler(void){
        if(TIM_GetFlagStatus(TIM4,TIM_FLAG_Update))
        {
            if(motor2.set_vel > 0){
                if(motor2.set_vel < motor2.vel_setpoint_){
                    motor2.vel_setpoint_-=Acc;
                }else if(motor2.set_vel >= motor2.vel_setpoint_){
                    motor2.vel_setpoint_+= Acc;
                }
            }else if(motor2.set_vel < 0){
                if(motor2.set_vel > motor2.vel_setpoint_){
                    motor2.vel_setpoint_+=Acc;
                }else if(motor2.set_vel <= motor2.vel_setpoint_){
                    motor2.vel_setpoint_-=Acc;
                }
            }else{
                if(motor2.vel_setpoint_>0)      motor2.vel_setpoint_-=Acc;
                else if(motor2.vel_setpoint_<0) motor2.vel_setpoint_+=Acc;
                else                            motor2.vel_setpoint_=0;
            }

            if(motor1.set_vel > 0){
                if(motor1.set_vel < motor1.vel_setpoint_){
                    motor1.vel_setpoint_-=Acc;
                }else if(motor1.set_vel >= motor1.vel_setpoint_){
                    motor1.vel_setpoint_+=Acc;
                }
            }else if(motor1.set_vel < 0){
                if(motor1.set_vel > motor1.vel_setpoint_){
                    motor1.vel_setpoint_+=Acc;
                }else if(motor1.set_vel <= motor1.vel_setpoint_){
                    motor1.vel_setpoint_-=Acc;
                }
            }else{
                if(motor1.vel_setpoint_>0)      motor1.vel_setpoint_-=Acc;
                else if(motor1.vel_setpoint_<0) motor1.vel_setpoint_+=Acc;
                else                            motor1.vel_setpoint_=0;
            }

            vofa_float[0]    = motor1.vbus_measure;
            vofa_float[1]    = motor1.currents.phA;
            vofa_float[2]    = motor1.currents.phB;
            vofa_float[3]    = motor1.currents.phC;
            vofa_float[4]    = motor1.Idq_setpoint_.second;
            vofa_float[5]    = motor1.Idq.second;
            vofa_float[6]    = motor1.vel_estimate_;
            vofa_float[7]    = motor2.vbus_measure;
            vofa_float[8]    = motor2.currents.phA;
            vofa_float[9]    = motor2.currents.phB;
            vofa_float[10]   = motor2.currents.phC;
            vofa_float[11]   = motor2.Idq_setpoint_.second;
            vofa_float[12]   = motor2.Idq.second;
            vofa_float[13]   = motor2.vel_estimate_;

            memcpy(vofa_data,(uint8_t*)vofa_float,sizeof(vofa_float));
            vofa_data[58] = 0X80;
            vofa_data[59] = 0X7F;
            USBFS_Endp_DataUp(DEF_UEP3,vofa_data,60,DEF_UEP_DMA_LOAD);
            TIM_ClearFlag(TIM4,TIM_FLAG_Update);
        }
    }

    void USB_LP_CAN1_RX0_IRQHandler(void)
    {

        if( CAN_GetITStatus( CAN1, CAN_IT_FMP0 ) != RESET )
        {
            CAN_Receive(CAN1,CAN_FIFO0,&CanRxStructure);
            if(CanRxStructure.StdId == 0X001)
            {
                float spd1 = CanRxStructure.Data[1];
                float spd2 = CanRxStructure.Data[3];

                if(CanRxStructure.Data[0] == 0)
                {
                    spd1*=-1;
                }

                if(CanRxStructure.Data[2] == 1)
                {
                    spd2*=-1;
                }

                motor1.set_vel = spd1;
                motor2.set_vel = spd2;
            }

        }
        CAN_ClearITPendingBit( CAN1, CAN_IT_FMP0 );
    }
#ifdef __cplusplus
}
#endif