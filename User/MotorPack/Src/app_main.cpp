#include "motor.h"
#include "debug.h"
#include "ch32v30x_usart.h"

extern "C" {
#include "SMO.h"
};
#define T_speed 130

Motor motor;
Motor::RunState SelectState = Motor::STATE_RESISTANCE_CALIBRATION;
uint8_t CAN_Status;
uint8_t direction = 0;

void app_main()
{
    motor.Init(TIM1,TIM3);

    motor.vel_gain_                                 = 0.1f / 2.f;
    motor.vel_integrator_gain_                      = 50.f*motor.vel_gain_;
    motor.m_config_.current_control_bandwidth       = 1000;


    motor.K_current = 1.0f;

    motor.e_config_.bandwidth = 1000;
    motor.pll_kp_             =  motor.e_config_.bandwidth * 2.f;
    motor.pll_ki_             = 0.25f * ( motor.pll_kp_ *  motor.pll_kp_);
    motor.K_current           = 1.0f;

    motor.effective_current_limit = 13.5f;

    motor.vel_setpoint_ = 0;

    motor.pos_gain = 15;

    motor.start();

    while(motor.vbus_measure < 8.f) {
        Delay_Ms(1);
    }

    for(;;)
    {
        static uint8_t arr[8] = {0,0,0,0,0,0,0,0};

        switch (SelectState) {
            case Motor::STATE_NONE:{
                motor.NowState = Motor::STATE_NONE;
            }break;


            case Motor::STATE_RESISTANCE_CALIBRATION:{

                motor.NowState = Motor::STATE_RESISTANCE_CALIBRATION;
                Delay_Ms(1500);

                motor.getResistance();

                motor.resistance = 11.9000f;
                motor.inductance = 0.01003f;
                motor.e_config_.direction =1;

                motor.update_current_controller_gains();
                SelectState = Motor::STATE_CLOSELOOP;
            }break;

            case Motor::STATE_INDUCTANCE_CALIBRATION:{
                motor.NowState = Motor::STATE_INDUCTANCE_CALIBRATION;
                Delay_Ms(1500);
                motor.getInductance();

//                SMO_MotorPare.Rs = motor.resistance;
//                SMO_MotorPare.Ls = motor.inductance;
//                SMO_MotorPare.Ib = 25;
//                SMO_MotorPare.Vb = 16.8f*0.57735f;
//                SMO_MotorPare.Ts = 0.00008f;
//                SMO_MotorPare.POLES = 7;
//                SMO_MotorPare.Fsmopos = exp((-SMO_MotorPare.Rs/SMO_MotorPare.Ls)*(SMO_MotorPare.Ts));
//                SMO_MotorPare.Gsmopos = (SMO_MotorPare.Vb/SMO_MotorPare.Ib)*(1/SMO_MotorPare.Rs)*(1-SMO_MotorPare.Fsmopos);
//
//                Angle_SMOPare.Fsmopos = SMO_MotorPare.Fsmopos ;
//                Angle_SMOPare.Gsmopos = SMO_MotorPare.Gsmopos ;
//                Angle_SMOPare.Kslide  = 0.1f;    //
//                Angle_SMOPare.Kslf    = 0.01f;    //
//                Angle_SMOPare.E0      = 0.5f;    //  标幺值1的一半
//                Speed_estPare.speed_coeff=(float)(500*60/(SMO_MotorPare.POLES*4096.0f));  //  1.831054463982609


                motor.update_current_controller_gains();

                SelectState = Motor::STATE_ENCODER_CALIBRATION;
            }break;

            case Motor::STATE_ENCODER_CALIBRATION:{
                motor.NowState = Motor::STATE_ENCODER_CALIBRATION;

                if(!motor.e_config_.is_ready_){
                    motor.openloop_Vdq.first  = 0;motor.openloop_Vdq.second = 5.0f;
                    Delay_Ms(2000);
                }

                if(!motor.e_config_.is_ready_ && (( motor.phase_vel_ > 0 && motor.e_config_.direction == 1) || (motor.phase_vel_ < 0 && motor.e_config_.direction == -1))){
                    motor.e_config_.is_ready_ = true;
                }else if(!motor.e_config_.is_ready_){
                    motor.e_config_.direction = -motor.e_config_.direction;
                }

                motor.openloop_Vdq.first  = 0;motor.openloop_Vdq.second = 0;

                Delay_Ms(200);

                SelectState = Motor::STATE_CLOSELOOP;
            }break;

            case Motor::STATE_CLOSELOOP:{
                motor.ctrlMode = Motor::CTRL_POS;
                motor.NowState = Motor::STATE_CLOSELOOP;
            }
            motor.pos_setpoint += 0.5f;
            Delay_Ms(500);
        }
        Delay_Ms(1);
    }
}