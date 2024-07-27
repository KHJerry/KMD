#include <algorithm>
#include "motor.h"
#include "ch32v30x_adc.h"

void Motor::Init(TIM_TypeDef *tD, TIM_TypeDef *tE)
{
    this->timer_driver_ = tD;
    this->timer_encoder_= tE;

    this->m_config_.pre_calibrated                  = false;
    this->m_config_.pole_pairs                      = 7;
    this->m_config_.calibration_current             = 10.0f;
    this->m_config_.resistance_calib_max_voltage    = 2.0f;
    this->m_config_.phase_inductance                = 0.0f;
    this->m_config_.phase_resistance                = 0.0f;
    this->m_config_.torque_constant                 = 0.04f;
    this->m_config_.motor_type                      = MOTOR_TYPE_HIGH_CURRENT;
    this->m_config_.current_lim                     = 10.0f;
    this->m_config_.current_lim_margin              = 8.0f;
    this->m_config_.torque_lim                      = 5.f;   //TODO:需要寻找扭矩的换算方法
    this->m_config_.current_control_bandwidth       = 100.0f;
    this->m_config_.inverter_temp_limit_lower       = 100;
    this->m_config_.inverter_temp_limit_upper       = 120;
    this->m_config_.acim_gain_min_flux              = 10;
    this->m_config_.acim_autoflux_min_Id            = 10;
    this->m_config_.acim_autoflux_enable            = false;
    this->m_config_.acim_autoflux_attack_gain       = 10.0f;
    this->m_config_.acim_autoflux_decay_gain        = 1.0f;
    this->m_config_.R_wL_FF_enable                  = true;
    this->m_config_.bEMF_FF_enable                  = false;
    this->m_config_.I_bus_hard_min                  = -INFINITY;
    this->m_config_.I_bus_hard_max                  =  INFINITY;
    this->m_config_.I_leak_max                      = 0.1f;
    this->m_config_.dc_calib_tau                    = 0.2f;


    this->e_config_.mode                            = MODE_INCREMENTAL;
    this->e_config_.calib_range                     = 0.02f;
    this->e_config_.calib_scan_distance             = 16.0f * M_PI;
    this->e_config_.calib_scan_omega                = 4.0f * M_PI;
    this->e_config_.bandwidth                       = 1000.0f;
    this->e_config_.phase_offset                    = 0;
    this->e_config_.phase_offset_float              = 0.0f;
    this->e_config_.cpr                             = (1023);
    this->e_config_.index_offset                    = 0.0f;
    this->e_config_.use_index                       = false;
    this->e_config_.pre_calibrated                  = false;
    this->e_config_.direction                       = 1;
    this->e_config_.use_index_offset                = false;
    this->e_config_.enable_phase_interpolation      = true; //插值算法
    this->e_config_.find_idx_on_lockin_only         = false;
    this->e_config_.is_ready_                       = false;

    this->NowState                                  = STATE_NONE;
    this->motorType                                 = MOTOR_TYPE_HIGH_CURRENT;
    this->encoderMode                               = MODE_INCREMENTAL;

    this->openloop_phase_                           = 0.0f;
    this->openloop_phase_vel_                       = M_PI;
    this->openloop_Vdq                              = {0.0f,0.5f};

    this->phase_                                    = 0.0f;
    this->phase_vel_                                = 0.0f;
    this->shadow_count_                             = 0;
    this->count_in_cpr_                             = 0;
    this->pos_cpr_counts_                           = 0.0f;
    this->pos_estimate_counts_                      = 0.0f;
    this->vel_estimate_counts_                      = 0.0f;
    this->delta_pos_cpr_counts_                     = 0.0f;
    this->pll_kp_                                   = e_config_.bandwidth * 2.f;
    this->pll_ki_                                   = 0.25f * (pll_kp_ * pll_kp_);
    this->pos_estimate_                             = 0.0f;
    this->vel_estimate_                             = 0.0f;
    this->pos_circular_                             = 0.0f;
    this->interpolation_                            = 0.0f;
    this->encoder_last_time_stamp_                  = 0;

    this->max_voltage_                              = 1.0f;
    this->actual_current_                           = 0.0f;
    this->target_current_                           = 10.f;
    this->test_voltage_R                            = 1.0f;
    this->I_beta_                                   = 0.0f;
    this->test_mod_                                 = NAN;
    this->resistance                                = 0.0f;

    //MeasueInductance Parameters
    this->test_voltage_I                            = 2.0f;
    this->attached_                                 = false;
    this->sign_                                     = 0;
    this->last_Ialpha_                              = NAN;
    this->deltaI_                                   = 0.0f;
    this->start_timestamp_                          = 0;
    this->last_input_timestamp_                     = 0;
    this->inductance                                = 0.0f;

    //Current CloseLoop Parameters
    this->effective_current_limit                   = 20.5f;
    this->pi_gains                                  = {0,0};
    this->I_measured_report_filter_k_               = 1.0f;
    this->Id_measured_                              = 0;
    this->Iq_measured_                              = 0;
    this->v_current_control_integral_d_             = 0.0f;
    this->v_current_control_integral_q_             = 0.0f;
    this->final_v_alpha_                            = 0.0f;
    this->final_v_beta_                             = 0.0f;
    this->power_                                    = 0.0f;
    this->current_last_timestamp                    = 0;
    this->mod_to_V                                  = (2.f/3.f) * vbus_measure;
    this->V_to_mod                                  = 1.f / mod_to_V;


    //Velocity Controller
    this->vel_limit                                 = 255.0f;
    this->vel_setpoint_                             = 0.f;
    this->vel_gain_                                 = 1/12.f;
    this->vel_integrator_gain_                      = this->vel_gain_*5;
    this->vel_integrator_torque_                    = 0.0f;
    this->vel_integrator_limit_                     = 50.0f;

    //Position Controller
    this->pos_setpoint                              = 0.f;
    this->pos_gain                                  = 0.f;
    this->pos_err                                   = 0.f;

    //Torque Controller
    this->torque_limit_                             = 1.05F;     //Nm
    this->torque_setpoint_                          = 0.0f;
    this->torque_setpoint_src                       = 0;
    this->torque_constant_                          = 0.05F;    //Nm/A
    this->torque_output_                            = 0;
    this->ibus = 0;

    Motor::vbus_measure = 16.8;

    //ControlMode
    ControlMode ctrlMode = ControlMode::CTRL_VEL;
}

void Motor::start()
{
    if(timer_driver_ == TIM1)
    {
        ADC_ExternalTrigInjectedConvCmd(ADC1,ENABLE);           //使能外部事件启动注入转换
    }
    else if(timer_driver_ == TIM8)
    {
        ADC_ExternalTrigInjectedConvCmd(ADC2,ENABLE);           //使能外部事件启动注入转换
        EN_GATE(1);

    }
}

void Motor::apply_pwm_timings()
{
    CCRs_setpoint.ccr1 = CCRs_setpoint.ccr1 > 1 ? 1 : CCRs_setpoint.ccr1;CCRs_setpoint.ccr1 = CCRs_setpoint.ccr1 < 0 ? 0 : CCRs_setpoint.ccr1;
    CCRs_setpoint.ccr2 = CCRs_setpoint.ccr2 > 1 ? 1 : CCRs_setpoint.ccr2;CCRs_setpoint.ccr2 = CCRs_setpoint.ccr2 < 0 ? 0 : CCRs_setpoint.ccr2;
    CCRs_setpoint.ccr3 = CCRs_setpoint.ccr3 > 1 ? 1 : CCRs_setpoint.ccr3;CCRs_setpoint.ccr3 = CCRs_setpoint.ccr3 < 0 ? 0 : CCRs_setpoint.ccr3;

    this->timer_driver_->CH1CVR = (uint32_t)(CCRs_setpoint.ccr1*TPWM);
    this->timer_driver_->CH2CVR = (uint32_t)(CCRs_setpoint.ccr2*TPWM);
    this->timer_driver_->CH3CVR = (uint32_t)(CCRs_setpoint.ccr3*TPWM);
}

void Motor::measure_currents(uint16_t c1, uint16_t c2, uint16_t c3)
{

    uint16_t ia_half_word   = c1;
    uint16_t ib_half_word   = c2;
    uint16_t ic_half_word   = c3;

    mod_to_V = (2.f/3.f) * vbus_measure;
    V_to_mod = 1.f / mod_to_V;


    currents.phA = -this->K_current*((float(ia_half_word   - ref_value[0])   / 4096.f) * 3.3f / 20.f / 0.001f);
    currents.phB = -this->K_current*((float(ib_half_word   - ref_value[1])   / 4096.f) * 3.3f / 20.f / 0.001f);
//    currents.phC = this->K_current*((float(ic_half_word   - ref_value[2])   / 4096.f) * 3.3f / 20.f / 0.001f);
    currents.phC = -currents.phA - currents.phB;
//    currents.phC = -currents.phA-currents.phB;

//    currents.phA = ia_half_word;
//    currents.phB = ib_half_word;
//    currents.phC = ic_half_word;
}

void Motor::measureResistance()
{
    actual_current_ = I_alpha_beta_.first;
    //test_voltage_R += kI * current_meas_period * (target_current_ - actual_current_);
    test_voltage_R = 0.5;
    //I_beta_ += (kIBetaFilt *current_meas_period) * (I_alpha_beta_.second - I_beta_);

    //if(test_voltage_R > max_voltage_) test_voltage_R = max_voltage_;
    float vfactor = 1.f / ((2.f/3.f) * vbus_measure);
    test_mod_ = test_voltage_R *vfactor;

    SVM(test_mod_,0,&CCRs_setpoint);

}

void Motor::getResistance()
{
    resistance = ABS(test_voltage_R / (actual_current_));

    this->timer_encoder_->CNT = 0;
    this->pos_cpr_counts_=0;
    this->pos_circular_ = 0;
    this->pos_estimate_counts_=0;
    this->pos_estimate_ = 0;
    this->phase_ = 0;
    this->phase_vel_ = 0;
}

void Motor::measureInductance(uint32_t timestamp)
{
    float Ialpha = I_alpha_beta_.first;

    if(attached_){
        float sign = test_voltage_I >=0.f ? 1.0f : -1.0f;
        deltaI_ += -sign * (Ialpha - last_Ialpha_);
    }else{
        start_timestamp_ = timestamp;
        attached_ = true;
    }
    last_Ialpha_ = Ialpha;
    last_input_timestamp_ = timestamp;

    test_voltage_I *= -1.f;
    float vfactor = 1.f / ((2.f/3.f)*vbus_measure);

    SVM(test_voltage_I*vfactor,0.0f,&CCRs_setpoint);

}

void Motor::getInductance()
{
    float dt = (float)(last_input_timestamp_ - start_timestamp_) * current_meas_period;
    inductance = ABS(test_voltage_I / (deltaI_ / dt));
}

void Motor::shut()
{
    CCRs_setpoint.ccr1 = 1;
    CCRs_setpoint.ccr2 = 1;
    CCRs_setpoint.ccr3 = 1;
}
void Motor::measurePhasePhaseVel()
{
    int32_t delta_enc = 0;

    switch (encoderMode) {
        case EncoderMode::MODE_INCREMENTAL:{
            delta_enc = (int16_t)this->timer_encoder_->CNT - (int16_t)shadow_count_;
        }break;

        case EncoderMode::MODE_ABS_AMS:{

        }break;
    }

    shadow_count_ += delta_enc;
    count_in_cpr_ += delta_enc;
    count_in_cpr_  = mod(count_in_cpr_,e_config_.cpr);

    float pos_cpr_counts_last_ = pos_cpr_counts_;

    pos_estimate_counts_ += current_meas_period * vel_estimate_counts_;
    pos_cpr_counts_      += current_meas_period * vel_estimate_counts_;

    float delta_pos_counts      = (float)(shadow_count_ - (int32_t)std::floor(pos_estimate_counts_));
    float delta_pos_cpr_counts  = (float)(count_in_cpr_ - (int32_t)std::floor(pos_cpr_counts_));
    delta_pos_cpr_counts        = wrap_pm(delta_pos_cpr_counts,(float)(e_config_.cpr));
    delta_pos_cpr_counts_+= 0.1f * (delta_pos_cpr_counts - delta_pos_cpr_counts_);

    pos_estimate_counts_ += current_meas_period * pll_kp_ * delta_pos_counts;
    pos_cpr_counts_      += current_meas_period * pll_kp_ * delta_pos_cpr_counts;
    pos_cpr_counts_      =  fmodf_pos(pos_cpr_counts_,float(e_config_.cpr));
    vel_estimate_counts_ += current_meas_period*pll_ki_*delta_pos_cpr_counts;

    bool snap_to_zero_vel = false;
    float abs_vel_estimate_counts_;
    abs_vel_estimate_counts_ = ABS(vel_estimate_counts_);

    if(abs_vel_estimate_counts_ < 0.5f*current_meas_period*pll_ki_){
        vel_estimate_counts_ = 0.0f;
        snap_to_zero_vel = true;
    }

    pos_estimate_ = pos_estimate_counts_ / (float)e_config_.cpr;
    float vel_estimate = vel_estimate_counts_ / (float)e_config_.cpr;
    vel_estimate_ = vel_estimate;

    float pos_circular = pos_circular_;
    pos_circular+= wrap_pm((pos_cpr_counts_ - pos_cpr_counts_last_)/(float(e_config_.cpr)),1.0f);
    pos_circular_=pos_circular;

    int32_t corrected_enc = count_in_cpr_ ;
    if (snap_to_zero_vel || !e_config_.enable_phase_interpolation) {
        interpolation_ = 0.5f;
    } else if (delta_enc > 0) {
        interpolation_ = 0.0f;
    } else if (delta_enc < 0) {
        interpolation_ = 1.0f;
    } else {
        interpolation_ += current_meas_period * vel_estimate_counts_;
        if (interpolation_ > 1.0f) interpolation_ = 1.0f;
        if (interpolation_ < 0.0f) interpolation_ = 0.0f;
    }
    float interpolated_enc = corrected_enc + interpolation_;

    float elec_rad_per_enc = m_config_.pole_pairs * 2 * M_PI * (1.0f / (float)(e_config_.cpr));
    float ph = elec_rad_per_enc * (interpolated_enc);


    phase_ = wrap_pm_pi(ph) * e_config_.direction;
    float vel = (2*M_PI) * vel_estimate_* m_config_.pole_pairs;
    phase_vel_ = vel;//(1 - A) * vel + (A)*phase_vel_;
}

void Motor::measureEncoder(uint32_t timestamp_)
{
    uint8_t dt = timestamp_ - encoder_last_time_stamp_;
    encoder_last_time_stamp_ = timestamp_;


    mod_d = V_to_mod * openloop_Vdq.first;
    mod_q = V_to_mod * openloop_Vdq.second;

    openloop_phase_vel_ = 5*M_PI;


    if(!e_config_.is_ready_){
        mod_alpha = cosf(openloop_phase_)*mod_d - sinf(openloop_phase_) *mod_q;
        mod_beta  = cosf(openloop_phase_)*mod_q + sinf(openloop_phase_) *mod_d;

        if(e_config_.direction ==  1) p += openloop_phase_vel_ * dt * current_meas_period;
        if(e_config_.direction == -1) p += openloop_phase_vel_ * dt * current_meas_period;

        openloop_phase_ = wrap_pm_pi(p);

    }else{
        mod_alpha = cosf(phase_)*mod_d - sinf(phase_) *mod_q;
        mod_beta  = cosf(phase_)*mod_q + sinf(phase_) *mod_d;
    }

    SVM(mod_alpha,mod_beta,&CCRs_setpoint);
}


void Motor::get_alpha_beta_output(uint32_t timestamp, Motor::float2D *mod_alpha_beta)
{
    float maybe_torque = torque_setpoint_src;

    float torque = e_config_.direction * maybe_torque;


    Idq_setpoint_.first  = 0;
    Idq_setpoint_.second = torque / torque_constant_;


    float iq_lim_sqr = effective_current_limit*effective_current_limit - Idq_setpoint_.first*Idq_setpoint_.first;
    float Iq_lim = (iq_lim_sqr <= 0.0f) ? 0.0f : sqrt(iq_lim_sqr);

    if(Idq_setpoint_.second > Iq_lim)       Idq_setpoint_.second = Iq_lim;
    else if(Idq_setpoint_.second <-Iq_lim)  Idq_setpoint_.second =-Iq_lim;


    float vd = 0.0f;
    float vq = 0.0f;


    if (m_config_.R_wL_FF_enable) {

        vd -= phase_vel_ * inductance * Idq_setpoint_.second;
        vq += phase_vel_ * inductance * Idq_setpoint_.first;
        vd += resistance * Idq_setpoint_.first;
        vq += resistance * Idq_setpoint_.second;
    }

    if (m_config_.bEMF_FF_enable) {
        vq += phase_vel_ * (2.0f/3.0f) * (0.05 / 7);
    }

    Vdq_setpoint_ = {vd, vq};

    //Park transform
    float Ialpha = I_alpha_beta_.first;
    float Ibeta  = I_alpha_beta_.second;

    float I_phase = phase_ + phase_vel_ * (float)(timestamp - current_last_timestamp) * current_meas_period;
    float c_I = cosf(I_phase);
    float s_I = sinf(I_phase);

    Idq.first  = (c_I*Ialpha + s_I*Ibeta);
    Idq.second = (c_I*Ibeta  - s_I*Ialpha);

    Id_measured_ += I_measured_report_filter_k_*(Idq.first  - Id_measured_);
    Iq_measured_ += I_measured_report_filter_k_*(Idq.second - Iq_measured_);


    //get Idq's errors
    Ierr_d = Idq_setpoint_.first  - Idq.first;
    Ierr_q = Idq_setpoint_.second - Idq.second;

    mod_d = V_to_mod * (Vdq_setpoint_.first  + v_current_control_integral_d_ + Ierr_d * pi_gains.first);
    mod_q = V_to_mod * (Vdq_setpoint_.second + v_current_control_integral_q_ + Ierr_q * pi_gains.first);

    //    mod_d = 0.089286f * (Vdq_setpoint_.first  + v_current_control_integral_d_ + Ierr_d * pi_gains.first);
    //    mod_q = 0.089286f * (Vdq_setpoint_.second + v_current_control_integral_q_ + Ierr_q * pi_gains.first);

    float result = sqrtf(mod_d * mod_d + mod_q * mod_q);
    float mod_scalefactor = 0.80f * sqrt3_by_2 * 1.0f / result;

    if (mod_scalefactor < 1.0f) {
        mod_d *= mod_scalefactor;
        mod_q *= mod_scalefactor;
        v_current_control_integral_d_ *= 0.99f;
        v_current_control_integral_q_ *= 0.99f;
    } else {
        v_current_control_integral_d_ += Ierr_d * (pi_gains.second * current_meas_period);
        v_current_control_integral_q_ += Ierr_q * (pi_gains.second * current_meas_period);
    }

    float pwm_phase = phase_ + phase_vel_ * ((float)(int32_t)(timestamp - current_last_timestamp) * current_meas_period);
    float c_p = cosf(pwm_phase);
    float s_p = sinf(pwm_phase);
    mod_alpha_beta->first  = c_p * mod_d - s_p * mod_q;
    mod_alpha_beta->second = c_p * mod_q + s_p * mod_d;
    current_last_timestamp = timestamp;
}

void Motor::update_current_controller_gains()
{
    float p_gain = m_config_.current_control_bandwidth * inductance;
    float plant_pole = resistance / inductance;
    pi_gains = {p_gain,plant_pole*p_gain};
}

void Motor::velovityLoop(uint32_t timestamp)
{
    //限制速度
    vel_setpoint_ = vel_setpoint_ > vel_limit ? vel_limit : vel_setpoint_;
    vel_setpoint_ = vel_setpoint_ <-vel_limit ?-vel_limit : vel_setpoint_;

    //限制力矩
    torque_setpoint_ = torque_setpoint_ > torque_limit_ ? torque_limit_ : torque_setpoint_;
    torque_setpoint_ = torque_setpoint_ <-torque_limit_ ?-torque_limit_ : torque_setpoint_;


    float torque = torque_setpoint_;
    float gain_scheduling_multiplier=1.0f;
    float v_err = 0;

    if(anticogging_valid_ && a_config_.anticogging_enabled){
        float anticogging_pos = pos_estimate_ / getCoggingRatio();
        torque += a_config_.cogging_map[mod((int)anticogging_pos, 360)];
    }

    v_err = vel_setpoint_ - vel_estimate_;
    torque = (vel_gain_ * gain_scheduling_multiplier)*v_err;
    torque += vel_integrator_torque_;


    //限制力矩
    bool limit = false;
    if(this->timer_driver_ == TIM1){
        torque = torque > 0 ? 0 : torque;
        torque = torque < -torque_limit_ ? -torque_limit_ : torque;
    }else{
        torque = torque > torque_limit_ ? torque_limit_ : torque;
        torque = torque <0 ? 0 : torque;
    }
    if(torque == torque_limit_ || torque == -torque_limit_) limit = true;

    if(limit) vel_integrator_torque_*=0.99f;
    else      vel_integrator_torque_+=((vel_integrator_gain_*gain_scheduling_multiplier)*current_meas_period*(float)(timestamp - velocity_last_time_stamp_))*v_err;

    vel_integrator_torque_ = vel_integrator_torque_ > vel_integrator_limit_ ? vel_integrator_limit_ : vel_integrator_torque_;
    vel_integrator_torque_ = vel_integrator_torque_ <-vel_integrator_limit_ ?-vel_integrator_limit_ : vel_integrator_torque_;


    torque_setpoint_src = torque;
    velocity_last_time_stamp_ = timestamp;
}

void Motor::positionLoop(uint32_t timestamp){
    static uint32_t last_time_stamp_ = 0;

    if(a_config_.calib_anticogging){
        if (std::abs(pos_err) <= a_config_.calib_pos_threshold / (float)e_config_.cpr &&std::abs(vel_estimate_) < a_config_.calib_vel_threshold / (float)e_config_.cpr) {
            if(a_config_.index < 360)a_config_.cogging_map[a_config_.index++] = vel_integrator_torque_;
        }

        if (a_config_.index < 360) {
            pos_setpoint = a_config_.index * getCoggingRatio();
            vel_setpoint_ = 0.0f;
            torque_setpoint_ = 0.0f;
        } else {
            a_config_.index = 0;
            pos_setpoint = 0.0f;  // Send the motor home
            vel_setpoint_ = 0.0f;
            torque_setpoint_ = 0.0f;
            anticogging_valid_ = true;
            a_config_.calib_anticogging = false;
        }
    }

    //限制速度
    vel_setpoint_ = vel_setpoint_ > vel_limit ? vel_limit : vel_setpoint_;
    vel_setpoint_ = vel_setpoint_ <-vel_limit ?-vel_limit : vel_setpoint_;

    //限制力矩
    torque_setpoint_ = torque_setpoint_ > torque_limit_ ? torque_limit_ : torque_setpoint_;
    torque_setpoint_ = torque_setpoint_ <-torque_limit_ ?-torque_limit_ : torque_setpoint_;

    float gain_scheduling_multiplier = 1.0f;
    float vel_des = vel_setpoint_;
    pos_err = pos_setpoint - pos_circular_;
    vel_des += pos_gain * pos_err;

    vel_des = vel_des > vel_limit ? vel_limit : vel_des;
    vel_des = vel_des <-vel_limit ?-vel_limit : vel_des;

    float torque = torque_setpoint_;

    if(anticogging_valid_ && a_config_.anticogging_enabled){
        ctrlMode = ControlMode::CTRL_VEL;
        ctrlMode = ControlMode::CTRL_VEL;
        ctrlMode = ControlMode::CTRL_VEL;
        float anticogging_pos = pos_circular_ / getCoggingRatio();
        torque += a_config_.cogging_map[mod((int)anticogging_pos, 360)];
    }

    float v_err   =  vel_des - vel_estimate_;
    torque += (vel_gain_ * gain_scheduling_multiplier) * v_err;

    torque += vel_integrator_torque_;

    bool limit = false;
    torque = torque > torque_limit_ ? torque_limit_ : torque;
    torque = torque <-torque_limit_ ?-torque_limit_ : torque;
    if(torque == torque_limit_ || torque == -torque_limit_) limit = true;

    if(limit) vel_integrator_torque_*=0.99f;
    else      vel_integrator_torque_+=((vel_integrator_gain_*gain_scheduling_multiplier)*current_meas_period*(timestamp - last_time_stamp_))*v_err;

    vel_integrator_torque_ = vel_integrator_torque_ > vel_integrator_limit_ ? vel_integrator_limit_ : vel_integrator_torque_;
    vel_integrator_torque_ = vel_integrator_torque_ <-vel_integrator_limit_ ?-vel_integrator_limit_ : vel_integrator_torque_;

    torque_setpoint_src = torque;

    last_time_stamp_ = timestamp;
}
void Motor::go_zero() {

    this->timer_encoder_->CNT   = 0;
    this->pos_cpr_counts_       = 0;
    this->pos_circular_         = 0;
    this->pos_estimate_counts_  = 0;
    this->pos_estimate_         = 0;
    this->phase_                = 0;

    float vd=0.05,vq=0;
    float alpha,beta;
    float c_p = cosf(0);
    float s_p = sinf(0);
    alpha  = c_p * vd - s_p * vq;
    beta   = c_p * vq + s_p * vd;
    SVM(alpha,beta,&CCRs_setpoint);
}


void Motor::Set_RGB_Breath_Rate(uint8_t rate,uint8_t color)
{
#define CNT 1000
    static uint16_t cnt[3]     = {0,0,0};
    static uint8_t  reverse[3] = {0,0,0};

    switch (color) {
        case 1:{
            if(!reverse[0]) cnt[0]+=rate;
            else            cnt[0]-=rate;

            if(cnt[0] >= CNT){
                cnt[0]     = CNT;
                reverse[0] = 1;
            }else if(cnt[0] <= 0){
                cnt[0]     = 0;
                reverse[0] = 0;
            }
        }break;

        case 2:{
            if(!reverse[1]) cnt[1]+=rate;
            else            cnt[1]-=rate;

            if(cnt[1] >= CNT){
                cnt[1]     = CNT;
                reverse[1] = 1;
            }else if(cnt[1] <= 0){
                cnt[1]     = 0;
                reverse[1] = 0;
            }
        }break;

        case 3:{
            if(!reverse[2]) cnt[2]+=rate;
            else            cnt[2]-=rate;

            if(cnt[2] >= CNT){
                cnt[2]     = CNT;
                reverse[2] = 1;
            }else if(cnt[2] <= 0){
                cnt[2]     = 0;
                reverse[2] = 0;
            }
        }break;
        default:{

        }break;
    }
    RGB_R(cnt[0]);
    RGB_G(cnt[1]);
    RGB_B(cnt[2]);
}

