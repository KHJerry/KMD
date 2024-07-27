#include "motor.h"
#include "debug.h"
#define T_speed 130
Motor motor1,motor2;
Motor::RunState SelectState = Motor::STATE_RESISTANCE_CALIBRATION;
uint8_t CAN_Status;

void app_main()
{
    motor1.Init(TIM1,TIM2);
    motor2.Init(TIM8,TIM3);

    motor1.vel_gain_                                 = 0.1f / 2.f;
    motor1.vel_integrator_gain_                      = 50.f*motor1.vel_gain_;
    motor1.m_config_.current_control_bandwidth       = 1500;

    motor2.vel_gain_                                 = 0.1f / 2.f;
    motor2.vel_integrator_gain_                      = 50.f*motor2.vel_gain_;
    motor2.m_config_.current_control_bandwidth       = 1500;

    motor1.K_current = 1.0f;
    motor2.K_current = 1.0f;

//    motor1.vel_gain_                                 = 0.06f/2.0f;
//    motor1.vel_integrator_gain_                      = 30.f*motor1.vel_gain_;
//    motor1.m_config_.current_control_bandwidth       = 1000;
//
//    motor2.vel_gain_                                 = 0.06f/2.0f;
//    motor2.vel_integrator_gain_                      = 30.f*motor2.vel_gain_;
//    motor2.m_config_.current_control_bandwidth       = 1000;

    motor1.e_config_.bandwidth = 1000;
    motor1.pll_kp_             =  motor1.e_config_.bandwidth * 2.f;
    motor1.pll_ki_             = 0.25f * ( motor1.pll_kp_ *  motor1.pll_kp_);
    motor1.K_current           = 1.0f;

    motor2.e_config_.bandwidth = 1000;
    motor2.pll_kp_             =  motor2.e_config_.bandwidth * 2.f;
    motor2.pll_ki_             = 0.25f * ( motor2.pll_kp_ *  motor2.pll_kp_);
    motor2.K_current           = 1.0f;

    motor1.effective_current_limit = 15.5f;
    motor2.effective_current_limit = 15.5f;

    motor1.vel_setpoint_ = 0;
    motor2.vel_setpoint_ = 0;

    motor1.s_set_vel = 210;
    motor2.s_set_vel = 210;

    motor1.start();
    motor2.start();

    float vel1 = 0,vel2=0;
    for(;;)
    {
        static uint8_t arr[8] = {0,0,0,0,0,0,0,0};

        switch (SelectState) {
            case Motor::STATE_NONE:{
                motor1.NowState = Motor::STATE_NONE;
                motor2.NowState = Motor::STATE_NONE;
            }break;


            case Motor::STATE_RESISTANCE_CALIBRATION:{

                motor1.NowState = Motor::STATE_RESISTANCE_CALIBRATION;
                motor2.NowState = Motor::STATE_RESISTANCE_CALIBRATION;
                Delay_Ms(1500);

                motor1.getResistance();
                motor2.getResistance();
                motor2.resistance = motor1.resistance;

                motor1.update_current_controller_gains();
                motor2.update_current_controller_gains();
                SelectState = Motor::STATE_INDUCTANCE_CALIBRATION;
            }break;

            case Motor::STATE_INDUCTANCE_CALIBRATION:{
                motor1.NowState = Motor::STATE_INDUCTANCE_CALIBRATION;
                motor2.NowState = Motor::STATE_INDUCTANCE_CALIBRATION;
                Delay_Ms(1500);
                motor1.getInductance();
                motor2.getInductance();
                motor2.inductance = motor1.inductance;

                motor1.update_current_controller_gains();
                motor2.update_current_controller_gains();

                SelectState = Motor::STATE_ENCODER_CALIBRATION;
            }break;

            case Motor::STATE_ENCODER_CALIBRATION:{
                motor1.NowState = Motor::STATE_ENCODER_CALIBRATION;
                motor2.NowState = Motor::STATE_ENCODER_CALIBRATION;

                if(!motor1.e_config_.is_ready_ && !motor2.e_config_.is_ready_){
                    motor1.openloop_Vdq.first  = 0;motor1.openloop_Vdq.second = 0.5;
                    motor2.openloop_Vdq.first  = 0;motor2.openloop_Vdq.second = 0.5;
                    Delay_Ms(2000);
                }

                if(!motor1.e_config_.is_ready_ && (( motor1.phase_vel_ > 0 && motor1.e_config_.direction == 1) || (motor1.phase_vel_ < 0 && motor1.e_config_.direction == -1))){
                    motor1.e_config_.is_ready_ = true;
                }else if(!motor1.e_config_.is_ready_){
                    motor1.e_config_.direction = -motor1.e_config_.direction;
                }

                if(!motor2.e_config_.is_ready_ && (( motor2.phase_vel_ > 0 && motor2.e_config_.direction == 1) || (motor2.phase_vel_ < 0 && motor2.e_config_.direction == -1))){
                    motor2.e_config_.is_ready_ = true;
                }else if(!motor2.e_config_.is_ready_){
                    motor2.e_config_.direction = -motor2.e_config_.direction;
                }


                motor1.openloop_Vdq.first  = 0;motor1.openloop_Vdq.second = 0;
                motor2.openloop_Vdq.first  = 0;motor2.openloop_Vdq.second = 0;

                Delay_Ms(200);

                SelectState = Motor::STATE_CLOSELOOP;
            }break;

            case Motor::STATE_CLOSELOOP:{
                motor1.ctrlMode = Motor::CTRL_VEL;
                motor2.ctrlMode = Motor::CTRL_VEL;

                motor1.NowState = Motor::STATE_CLOSELOOP;
                motor2.NowState = Motor::STATE_CLOSELOOP;

//                motor1.vel_setpoint_= -200.f;
//                motor2.vel_setpoint_= -200.f;
//                Delay_Ms(500);
//                motor1.vel_setpoint_= 1.f;
//                motor2.vel_setpoint_= 1.f;
//                Delay_Ms(500);
//
//                if(motor1.s_set_vel!=0 && motor2.s_set_vel !=0)
//                {
//                   motor2.set_vel+=0.05f;
//                   motor1.set_vel-=0.05f;
//                   if(motor2.set_vel>= motor2.s_set_vel)motor2.set_vel = motor2.s_set_vel;
//                   if(motor1.set_vel<=-motor1.s_set_vel)motor1.set_vel =-motor1.s_set_vel;
//                }else{
//                    motor2.set_vel = motor2.s_set_vel;
//                    motor1.set_vel =-motor1.s_set_vel;
//                }

                motor2.set_vel=250.0f;
                //motor2.torque_setpoint_src = 15.5f;
                Delay_Ms(1000);
                motor2.set_vel=0;
                //motor2.torque_setpoint_src = 0.0f;
                Delay_Ms(1000);
                motor1.set_vel=-250.00f;
                Delay_Ms(1000);
                motor1.set_vel=0;
                Delay_Ms(1000);

//                motor1.torque_setpoint_src =-1.05f;
//                motor2.torque_setpoint_src = 1.05f;
//                Delay_Ms(1000);
//                motor1.torque_setpoint_src=0.0f;
//                motor2.torque_setpoint_src=0.0f;
//                Delay_Ms(1000);


//                motor1.set_vel-=0.05f;
//                if(motor2.set_vel>= motor2.s_set_vel)motor2.set_vel = motor2.s_set_vel;
//                if(motor1.set_vel<=-motor1.s_set_vel)motor1.set_vel =-motor1.s_set_vel;

//                Delay_Ms(1000);
//                motor1.set_vel = -100;
//                Delay_Ms(1000);
//                motor1.set_vel = -200;
//                Delay_Ms(1000);
//                motor1.set_vel = -50;
//                Delay_Ms(1000);
//                motor1.set_vel = -200;
//                Delay_Ms(1000);
//                motor1.set_vel = 0;
//                Delay_Ms(1000);
//
//                motor1.set_vel = 210;
//                Delay_Ms(1000);
//                motor1.set_vel = 100;
//                Delay_Ms(1000);
//                motor1.set_vel = 210;
//                Delay_Ms(1000);
//                motor1.set_vel = 50;
//                Delay_Ms(1000);
//                motor1.set_vel = 210;
//                Delay_Ms(1000);
//                motor1.set_vel = 0;
//                Delay_Ms(1000);
            }
        }
        Delay_Ms(1);
    }
}