#ifndef __MOTOR_HPP
#define __MOTOR_HPP

#define TPWM                   144000000.0f / 25000.0f / 2.0f
#define one_by_sqrt3           0.57735026919F
#define two_by_sqrt3           1.15470053838F
#define sqrt3_by_2             0.86602540378F
#define current_meas_period    (0.00004f*2.f)

#define ABS(X) (X) < 0 ? -(X) : (X)

#ifdef __cplusplus
extern "C" {
#endif

#include "HardWare.h"

void app_main();
#ifdef __cplusplus
}
#endif


#ifdef __cplusplus
#include <limits>
#include <cmath>
class Motor {
public:
    enum MotorType{MOTOR_TYPE_HIGH_CURRENT = 0};
    enum EncoderMode{MODE_INCREMENTAL=0,MODE_ABS_AMS};
    enum RunState{STATE_NONE=0,STATE_ZERO,STATE_RESISTANCE_CALIBRATION,STATE_INDUCTANCE_CALIBRATION,STATE_ENCODER_CALIBRATION,STATE_CLOSELOOP};
    enum ControlMode{CTRL_VEL=0,CTRL_POS};
    enum RGBStatus{Blue,Green,Red,BlueGreen,BlueRed,GreenRed,BlueGreenRed,NONE};

    struct Anticogging_t {//23 Bytes
        uint32_t index                = 0;
        float   cogging_map[360];
        bool    pre_calibrated        = false;
        bool    calib_anticogging     = false;
        float   calib_pos_threshold   = 5.0f;
        float   calib_vel_threshold   = 1.0f;
        float   cogging_ratio         = 1.0f;
        bool    anticogging_enabled   = false;
    };
    struct MotorConfig_t {//88 Bytes
        bool pre_calibrated                 = false;
        int32_t pole_pairs                  = 14;
        float calibration_current           = 10.0f;
        float resistance_calib_max_voltage  = 2.0f;
        float phase_inductance              = 0.0f;
        float phase_resistance              = 0.0f;
        float torque_constant               = 0.04f;
        MotorType motor_type                = MOTOR_TYPE_HIGH_CURRENT;

        float current_lim                   = 10.0f;
        float current_lim_margin            = 8.0f;
        float torque_lim                    = 5.f;   //TODO:需要寻找扭矩的换算方法

        float current_control_bandwidth     = 1000.0f;
        float inverter_temp_limit_lower     = 100;
        float inverter_temp_limit_upper     = 120;

        float acim_gain_min_flux            = 10;
        float acim_autoflux_min_Id          = 10;
        bool acim_autoflux_enable           = false;
        float acim_autoflux_attack_gain     = 10.0f;
        float acim_autoflux_decay_gain      = 1.0f;

        bool R_wL_FF_enable                 = true;
        bool bEMF_FF_enable                 = false;

        float I_bus_hard_min                = -INFINITY;
        float I_bus_hard_max                =  INFINITY;
        float I_leak_max                    = 0.1f;

        float dc_calib_tau                  = 0.2f;
    };

    struct EncoderConfig_t {//46 Bytes
        EncoderMode mode                       = MODE_INCREMENTAL;
        float   calib_range                    = 0.02f;
        float   calib_scan_distance            = 16.0f * M_PI;
        float   calib_scan_omega               = 4.0f * M_PI;
        float   bandwidth                      = 1000.0f;
        int32_t phase_offset                   = 0;
        float   phase_offset_float             = 0.0f;
        int32_t cpr                            = (1000);
        float   index_offset                   = 0.0f;
        bool    use_index                      = false;
        bool    pre_calibrated                 = false;

        int32_t direction                      = 1;
        bool    use_index_offset               = true;
        bool    enable_phase_interpolation     = true;
        bool    find_idx_on_lockin_only        = false;
        bool    is_ready_                      = false;
    };
    struct float2D{float first,second;};
    struct Iph_ABC_t{float phA,phB,phC;};
    struct CCRs{float ccr1,ccr2,ccr3;};

    Motor(){};
    void Init(TIM_TypeDef *tD,TIM_TypeDef *tE);
    void start();
    void shut();
    void go_zero();
    void apply_pwm_timings();
    void measure_currents(uint16_t c1, uint16_t c2, uint16_t c3);
    void measureResistance(); void getResistance();
    void measureInductance(uint32_t timestamp); void getInductance();
    void update_current_controller_gains();
    void measureEncoder(uint32_t timestamp_);
    void measurePhasePhaseVel();
    void get_alpha_beta_output(uint32_t timestamp,float2D* mod_alpha_beta);
    void velovityLoop(uint32_t timestamp);
    void positionLoop(uint32_t timestamp);
    static void Set_RGB_Breath_Rate(uint8_t rate,uint8_t color);


    void SVM(float alpha, float beta,CCRs *ccrs) {

        if (beta >= 0.0f) {
            if (alpha >= 0.0f) {
                //quadrant I
                if (one_by_sqrt3 * beta > alpha)
                    Sextant = 2; //sextant v2-v3
                else
                    Sextant = 1; //sextant v1-v2
            } else {
                //quadrant II
                if (-one_by_sqrt3 * beta > alpha)
                    Sextant = 3; //sextant v3-v4
                else
                    Sextant = 2; //sextant v2-v3
            }
        } else {
            if (alpha >= 0.0f) {
                //quadrant IV
                if (-one_by_sqrt3 * beta > alpha)
                    Sextant = 5; //sextant v5-v6
                else
                    Sextant = 6; //sextant v6-v1
            } else {
                //quadrant III
                if (one_by_sqrt3 * beta > alpha)
                    Sextant = 4; //sextant v4-v5
                else
                    Sextant = 5; //sextant v5-v6
            }
        }

        switch (Sextant) {
            // sextant v1-v2
            case 1: {
                // Vector on-times
                float t1 = alpha - one_by_sqrt3 * beta;
                float t2 = two_by_sqrt3 * beta;

                // PWM timings
                tA = (1.0f - t1 - t2) * 0.5f;
                tB = tA + t1;
                tC = tB + t2;
            } break;

                // sextant v2-v3
            case 2: {
                // Vector on-times
                float t2 = alpha + one_by_sqrt3 * beta;
                float t3 = -alpha + one_by_sqrt3 * beta;

                // PWM timings
                tB = (1.0f - t2 - t3) * 0.5f;
                tA = tB + t3;
                tC = tA + t2;
            } break;

                // sextant v3-v4
            case 3: {
                // Vector on-times
                float t3 = two_by_sqrt3 * beta;
                float t4 = -alpha - one_by_sqrt3 * beta;

                // PWM timings
                tB = (1.0f - t3 - t4) * 0.5f;
                tC = tB + t3;
                tA = tC + t4;
            } break;

                // sextant v4-v5
            case 4: {
                // Vector on-times
                float t4 = -alpha + one_by_sqrt3 * beta;
                float t5 = -two_by_sqrt3 * beta;

                // PWM timings
                tC = (1.0f - t4 - t5) * 0.5f;
                tB = tC + t5;
                tA = tB + t4;
            } break;

                // sextant v5-v6
            case 5: {
                // Vector on-times
                float t5 = -alpha - one_by_sqrt3 * beta;
                float t6 = alpha - one_by_sqrt3 * beta;

                // PWM timings
                tC = (1.0f - t5 - t6) * 0.5f;
                tA = tC + t5;
                tB = tA + t6;
            } break;

                // sextant v6-v1
            case 6: {
                // Vector on-times
                float t6 = -two_by_sqrt3 * beta;
                float t1 = alpha + one_by_sqrt3 * beta;

                // PWM timings
                tA = (1.0f - t6 - t1) * 0.5f;
                tC = tA + t1;
                tB = tC + t6;
            } break;
        }
        ccrs->ccr1 = tA;
        ccrs->ccr2 = tB;
        ccrs->ccr3 = tC;
        return;
    }
    void Clark(Iph_ABC_t currents,float2D &Ialpha_beta){
        Ialpha_beta = {currents.phA,one_by_sqrt3*(currents.phB - currents.phC)};
    }
    int mod(const int dividend, const int divisor){
        int r = dividend % divisor;
        if (r < 0) r += divisor;
        return r;
    }
    int round_int(float x) {
#ifdef __arm__
        int res;
        asm("vcvtr.s32.f32   %[res], %[x]"
                : [res] "=X" (res)
        : [x] "w" (x) );
        return res;
#else
        return (int)nearbyint(x);
#endif
    }
    float wrap_pm(float x, float y) {
#ifdef FPU_FPV4
        float intval = (float)round_int(x / y);
#else
        float intval = nearbyintf(x / y);
#endif
        return x - intval * y;
    }
    float fmodf_pos(float x, float y) {
        float res = wrap_pm(x, y);
        if (res < 0) res += y;
        return res;
    }
    float wrap_pm_pi(float x) {
        return wrap_pm(x, 2 * M_PI);
    }
    float getCoggingRatio(){return 1.0f/360.f;}

//protected:
    TIM_TypeDef *timer_driver_;
    TIM_TypeDef *timer_encoder_;

    MotorConfig_t   m_config_;
    EncoderConfig_t e_config_;
    Anticogging_t   a_config_;

    Iph_ABC_t currents          = {0,0,0};
    CCRs CCRs_setpoint          = {0,0,0};
    float2D Vdq_setpoint_       = {0,1};
    float2D Idq_setpoint_       = {0,5.f};
    float2D I_alpha_beta_       = {0.0f,0.0f};
    float vbus_measure          = 24.0f;

    RunState NowState           = STATE_NONE;
    MotorType motorType         = MOTOR_TYPE_HIGH_CURRENT;
    EncoderMode encoderMode     = MODE_INCREMENTAL;

    //OpenLoop Parameters
    float   openloop_phase_     = 0.0f;
    float   openloop_phase_vel_ = M_PI;
    float2D openloop_Vdq        = {0.0f,0.5f};

    //Encoder Parameters
    float phase_                = 0.0f;
    float phase_vel_            = 0.0f;
    int32_t shadow_count_       = 0;
    int32_t count_in_cpr_       = 0;
    float pos_cpr_counts_       = 0.0f;
    float pos_estimate_counts_  = 0.0f;
    float vel_estimate_counts_  = 0.0f;
    float delta_pos_cpr_counts_ = 0.0f;
    float pll_kp_               = e_config_.bandwidth * 2.f;
    float pll_ki_               = 0.25f * (pll_kp_ * pll_kp_);
    float pos_estimate_         = 0.0f;
    float vel_estimate_         = 0.0f;
    float pos_circular_         = 0.0f;
    float interpolation_        = 0.0f;
    uint32_t encoder_last_time_stamp_ = 0;
    float p = 0;

    //MeasureResistance Parameters
    const float kI              = 1.0f;
    const float kIBetaFilt      = 80.0f;
    float max_voltage_          = 1.0f;
    float actual_current_       = 0.0f;
    float target_current_       = 10.f;
    float test_voltage_R        = 0.0f;
    float I_beta_               = 0.0f;
    float test_mod_             = NAN;
    float resistance            = 0.0f;
    float ibus = 0;

    //MeasueInductance Parameters
    float test_voltage_I           = 2.0f;
    bool attached_                 = false;
    float sign_                    = 0;
    float last_Ialpha_             = NAN;
    float deltaI_                  = 0.0f;
    uint32_t start_timestamp_      = 0;
    uint32_t last_input_timestamp_ = 0;
    float inductance               = 0.0f;

    //Current CloseLoop Parameters
    float effective_current_limit  = 70.0f;
    float2D pi_gains = {0,0};
    float I_measured_report_filter_k_ = 1.0f;
    float vbus_voltage_measured_;
    float2D Ialpha_beta_measured_;
    float Id_measured_=0;
    float Iq_measured_=0;
    float v_current_control_integral_d_ = 0.0f;
    float v_current_control_integral_q_ = 0.0f;
    float final_v_alpha_ = 0.0f;
    float final_v_beta_ = 0.0f;
    float power_ = 0.0f;
    uint32_t current_last_timestamp = 0;
    float2D Idq;
    float mod_to_V = (2.f/3.f) * vbus_measure;
    float V_to_mod = 1.f / mod_to_V;
    float mod_d,mod_q;
    float mod_alpha,mod_beta;
    float2D mod_alpha_beta;
    float tA, tB, tC;
    int Sextant;
    float K_current;
    float Ierr_d,Ierr_q;

    //Velocity Controller
    float vel_limit             = 20.0f;
    float vel_setpoint_         = 0.f;
    float vel_gain_             = 0.95f/2;
    float vel_integrator_gain_  = vel_gain_*5;
    float vel_integrator_torque_= 0.0f;
    float vel_integrator_limit_ = 10.0f;
    uint32_t velocity_last_time_stamp_ = 0;

    //Position Controller
    float pos_setpoint = 0.f;
    float pos_gain = 25.f;
    float pos_err;

    //Torque Controller
    float torque_limit_    = 3.0F;//Nm
    float torque_setpoint_ = 0.0f;
    float torque_setpoint_src = 0;
    float torque_constant_ = 0.03125F;//Nm/A
    float torque_output_ = 0;
    float set_vel;
    float s_set_vel;

    //RGB Parameter
    uint8_t RGB_cnt[3] = {0,0,0};
    uint8_t RGB_ccr[3] = {0,0,0};
    uint8_t RGB_status = NONE;

    //ControlMode
    ControlMode ctrlMode = ControlMode::CTRL_POS;

    float anticogging_valid_ = false;
    uint16_t ref_value[3];
};

extern Motor motor1,motor2;
extern Motor::RunState SelectState;
extern float vofa_float[];
extern uint8_t vofa_data[];
extern uint8_t CAN_Status;
//extern float vbus,ibus;
#endif



#endif
