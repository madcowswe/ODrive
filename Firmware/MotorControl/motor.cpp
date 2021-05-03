
#include <algorithm>

#include "drv8301.h"
#include "odrive_main.h"


Motor::Motor(const MotorHardwareConfig_t& hw_config,
             const GateDriverHardwareConfig_t& gate_driver_config,
             Config_t& config) :
        hw_config_(hw_config),
        gate_driver_config_(gate_driver_config),
        config_(config),
        gate_driver_({
            .spiHandle = gate_driver_config_.spi,
            .EngpioHandle = gate_driver_config_.enable_port,
            .EngpioNumber = gate_driver_config_.enable_pin,
            .nCSgpioHandle = gate_driver_config_.nCS_port,
            .nCSgpioNumber = gate_driver_config_.nCS_pin,
        }) {
    update_current_controller_gains();
}

// @brief Arms the PWM outputs that belong to this motor.
//
// Note that this does not yet activate the PWM outputs, it just unlocks them.
//
// While the motor is armed, the control loop must set new modulation timings
// between any two interrupts (that is, enqueue_modulation_timings must be executed).
// If the control loop fails to do so, the next interrupt handler floats the
// phases. Once this happens, missed_control_deadline is set to true and
// the motor can be considered disarmed.
//
// @returns: True on success, false otherwise
bool Motor::arm() {

    // Reset controller states, integrators, setpoints, etc.
    axis_->controller_.reset();
    reset_current_control();

    // Wait until the interrupt handler triggers twice. This gives
    // the control loop the correct time quota to set up modulation timings.
    if (!axis_->wait_for_current_meas())
        return axis_->error_ |= Axis::ERROR_CURRENT_MEASUREMENT_TIMEOUT, false;
    next_timings_valid_ = false;
    safety_critical_arm_motor_pwm(*this);
    return true;
}

void Motor::reset_current_control() {
    current_control_.v_current_control_integral_d = 0.0f;
    current_control_.v_current_control_integral_q = 0.0f;
    current_control_.acim_rotor_flux = 0.0f;
    current_control_.Ibus = 0.0f;
}

// @brief Tune the current controller based on phase resistance and inductance
// This should be invoked whenever one of these values changes.
// TODO: allow update on user-request or update automatically via hooks
void Motor::update_current_controller_gains() {
    // Calculate current control gains
    current_control_.p_gain = config_.current_control_bandwidth * config_.phase_inductance;
    float plant_pole = config_.phase_resistance / config_.phase_inductance;
    current_control_.i_gain = plant_pole * current_control_.p_gain;
}

// @brief Set up the gate drivers
void Motor::DRV8301_setup() {
    // for reference:
    // 20V/V on 500uOhm gives a range of +/- 150A
    // 40V/V on 500uOhm gives a range of +/- 75A
    // 20V/V on 666uOhm gives a range of +/- 110A
    // 40V/V on 666uOhm gives a range of +/- 55A

    // Solve for exact gain, then snap down to have equal or larger range as requested
    // or largest possible range otherwise
    constexpr float kMargin = 0.90f;
    constexpr float kTripMargin = 1.0f; // Trip level is at edge of linear range of amplifer
    constexpr float max_output_swing = 1.35f; // [V] out of amplifier
    float max_unity_gain_current = kMargin * max_output_swing * hw_config_.shunt_conductance; // [A]
    float requested_gain = max_unity_gain_current / config_.requested_current_range; // [V/V]

    // Decoding array for snapping gain
    std::array<std::pair<float, DRV8301_ShuntAmpGain_e>, 4> gain_choices = { 
        std::make_pair(10.0f, DRV8301_ShuntAmpGain_10VpV),
        std::make_pair(20.0f, DRV8301_ShuntAmpGain_20VpV),
        std::make_pair(40.0f, DRV8301_ShuntAmpGain_40VpV),
        std::make_pair(80.0f, DRV8301_ShuntAmpGain_80VpV)
    };

    // We use lower_bound in reverse because it snaps up by default, we want to snap down.
    auto gain_snap_down = std::lower_bound(gain_choices.crbegin(), gain_choices.crend(), requested_gain, 
    [](std::pair<float, DRV8301_ShuntAmpGain_e> pair, float val){
        return pair.first > val;
    });

    // If we snap to outside the array, clip to smallest val
    if(gain_snap_down == gain_choices.crend())
       --gain_snap_down;

    // Values for current controller
    phase_current_rev_gain_ = 1.0f / gain_snap_down->first;
    // Clip all current control to actual usable range
    current_control_.max_allowed_current = max_unity_gain_current * phase_current_rev_gain_;
    // Set trip level
    current_control_.overcurrent_trip_level = (kTripMargin / kMargin) * current_control_.max_allowed_current;

    // We now have the gain settings we want to use, lets set up DRV chip
    DRV_SPI_8301_Vars_t* local_regs = &gate_driver_regs_;
    DRV8301_enable(&gate_driver_);
    DRV8301_setupSpi(&gate_driver_, local_regs);

    local_regs->Ctrl_Reg_1.OC_MODE = DRV8301_OcMode_LatchShutDown;
    // Overcurrent set to approximately 150A at 100degC. This may need tweaking.
    local_regs->Ctrl_Reg_1.OC_ADJ_SET = DRV8301_VdsLevel_0p730_V;
    local_regs->Ctrl_Reg_2.GAIN = gain_snap_down->second;

    local_regs->SndCmd = true;
    DRV8301_writeData(&gate_driver_, local_regs);
    local_regs->RcvCmd = true;
    DRV8301_readData(&gate_driver_, local_regs);
}

// @brief Checks if the gate driver is in operational state.
// @returns: true if the gate driver is OK (no fault), false otherwise
bool Motor::check_DRV_fault() {
    //TODO: make this pin configurable per motor ch
    GPIO_PinState nFAULT_state = HAL_GPIO_ReadPin(gate_driver_config_.nFAULT_port, gate_driver_config_.nFAULT_pin);
    if (nFAULT_state == GPIO_PIN_RESET) {
        // Update DRV Fault Code
        gate_driver_exported_.drv_fault = (GateDriverIntf::DrvFault)DRV8301_getFaultType(&gate_driver_);
        // Update/Cache all SPI device registers
        // DRV_SPI_8301_Vars_t* local_regs = &gate_driver_regs_;
        // local_regs->RcvCmd = true;
        // DRV8301_readData(&gate_driver_, local_regs);
        return false;
    };
    return true;
}

void Motor::set_error(Motor::Error error){
    error_ |= error;
    axis_->error_ |= Axis::ERROR_MOTOR_FAILED;
    safety_critical_disarm_motor_pwm(*this);
    update_brake_current();
}

bool Motor::do_checks() {
    if (!check_DRV_fault()) {
        set_error(ERROR_DRV_FAULT);
        return false;
    }

    return true;
}

float Motor::effective_current_lim() {
    // Configured limit
    float current_lim = config_.current_lim;
    // Hardware limit
    if (axis_->motor_.config_.motor_type == Motor::MOTOR_TYPE_GIMBAL) {
        current_lim = std::min(current_lim, 0.98f*one_by_sqrt3*vbus_voltage); //gimbal motor is voltage control
    } else {
        current_lim = std::min(current_lim, axis_->motor_.current_control_.max_allowed_current);
    }

    // Apply axis current limiters
    for (const CurrentLimiter* const limiter : axis_->current_limiters_) {
        current_lim = std::min(current_lim, limiter->get_current_limit(config_.current_lim));
    }

    effective_current_lim_ = current_lim;

    return effective_current_lim_;
}

//return the maximum available torque for the motor.
//Note - for ACIM motors, available torque is allowed to be 0.
float Motor::max_available_torque() {
    if (config_.motor_type == Motor::MOTOR_TYPE_ACIM) {
        float max_torque = effective_current_lim() * config_.torque_constant * current_control_.acim_rotor_flux;
        max_torque = std::clamp(max_torque, 0.0f, config_.torque_lim);
        return max_torque;
    }
    else {
        float max_torque = effective_current_lim() * config_.torque_constant;
        max_torque = std::clamp(max_torque, 0.0f, config_.torque_lim);
        return max_torque;
    }
}

void Motor::log_timing(TimingLog_t log_idx) {
    static const uint16_t clocks_per_cnt = (uint16_t)((float)TIM_1_8_CLOCK_HZ / (float)TIM_APB1_CLOCK_HZ);
    uint16_t timing = clocks_per_cnt * htim13.Instance->CNT; // TODO: Use a hw_config

    if (log_idx < TIMING_LOG_NUM_SLOTS) {
        timing_log_[log_idx] = timing;
    }
}

float Motor::phase_current_from_adcval(uint32_t ADCValue) {
    int adcval_bal = (int)ADCValue - (1 << 11);
    float amp_out_volt = (3.3f / (float)(1 << 12)) * (float)adcval_bal;
    float shunt_volt = amp_out_volt * phase_current_rev_gain_;
    float current = shunt_volt * hw_config_.shunt_conductance;
    return current;
}

//--------------------------------
// Measurement and calibration
//--------------------------------

// TODO check Ibeta balance to verify good motor connection
bool Motor::measure_phase_resistance(float test_current, float max_voltage) {
    static const float kI = 10.0f;                                 // [(V/s)/A]
    static const int num_test_cycles = (int)(3.0f / CURRENT_MEAS_PERIOD); // Test runs for 3s
    float test_voltage = 0.0f;
    
    size_t i = 0;
    axis_->run_control_loop([&](){
        float Ialpha = -(current_meas_.phB + current_meas_.phC);
        test_voltage += (kI * current_meas_period) * (test_current - Ialpha);
        if (test_voltage > max_voltage || test_voltage < -max_voltage)
            return set_error(ERROR_PHASE_RESISTANCE_OUT_OF_RANGE), false;

        // Test voltage along phase A
        if (!enqueue_voltage_timings(test_voltage, 0.0f))
            return false; // error set inside enqueue_voltage_timings
        log_timing(TIMING_LOG_MEAS_R);

        return ++i < num_test_cycles;
    });
    if (axis_->error_ != Axis::ERROR_NONE)
        return false;

    //// De-energize motor
    //if (!enqueue_voltage_timings(motor, 0.0f, 0.0f))
    //    return false; // error set inside enqueue_voltage_timings

    float R = test_voltage / test_current;
    config_.phase_resistance = R;
    return true; // if we ran to completion that means success
}

bool Motor::measure_phase_inductance(float voltage_low, float voltage_high) {
    float test_voltages[2] = {voltage_low, voltage_high};
    float Ialphas[2] = {0.0f};
    static const int num_cycles = 5000;

    size_t t = 0;
    axis_->run_control_loop([&](){
        int i = t & 1;
        Ialphas[i] += -current_meas_.phB - current_meas_.phC;

        // Test voltage along phase A
        if (!enqueue_voltage_timings(test_voltages[i], 0.0f))
            return false; // error set inside enqueue_voltage_timings
        log_timing(TIMING_LOG_MEAS_L);

        return ++t < (num_cycles << 1);
    });
    if (axis_->error_ != Axis::ERROR_NONE)
        return false;

    //// De-energize motor
    //if (!enqueue_voltage_timings(motor, 0.0f, 0.0f))
    //    return false; // error set inside enqueue_voltage_timings

    float v_L = 0.5f * (voltage_high - voltage_low);
    // Note: A more correct formula would also take into account that there is a finite timestep.
    // However, the discretisation in the current control loop inverts the same discrepancy
    float dI_by_dt = (Ialphas[1] - Ialphas[0]) / (current_meas_period * (float)num_cycles);
    float L = v_L / dI_by_dt;

    config_.phase_inductance = L;
    // TODO arbitrary values set for now
    if (L < 2e-6f || L > 4000e-6f)
        return set_error(ERROR_PHASE_INDUCTANCE_OUT_OF_RANGE), false;
    return true;
}


bool Motor::run_calibration() {
    float R_calib_max_voltage = config_.resistance_calib_max_voltage;
    if (config_.motor_type == MOTOR_TYPE_HIGH_CURRENT
        || config_.motor_type == MOTOR_TYPE_ACIM) {
        if (!measure_phase_resistance(config_.calibration_current, R_calib_max_voltage))
            return false;
        if (!measure_phase_inductance(-R_calib_max_voltage, R_calib_max_voltage))
            return false;
    } else if (config_.motor_type == MOTOR_TYPE_GIMBAL) {
        // no calibration needed
    } else if (config_.motor_type == MOTOR_TYPE_BRUSHED_CURRENT) {
        // no calibration needed
    } else if (config_.motor_type == MOTOR_TYPE_BRUSHED_VOLTAGE) {
        // no calibration needed
    } else {
        return false;
    }

    update_current_controller_gains();
    
    is_calibrated_ = true;
    return true;
}

bool Motor::enqueue_modulation_timings(float mod_alpha, float mod_beta) {
    float tA, tB, tC;
    if (SVM(mod_alpha, mod_beta, &tA, &tB, &tC) != 0)
        return set_error(ERROR_MODULATION_MAGNITUDE), false;
    next_timings_[0] = (uint16_t)(tA * (float)TIM_1_8_PERIOD_CLOCKS);
    next_timings_[1] = (uint16_t)(tB * (float)TIM_1_8_PERIOD_CLOCKS);
    next_timings_[2] = (uint16_t)(tC * (float)TIM_1_8_PERIOD_CLOCKS);
    next_timings_valid_ = true;
    return true;
}

bool Motor::enqueue_voltage_timings(float v_alpha, float v_beta) {
    float vfactor = 1.0f / ((2.0f / 3.0f) * vbus_voltage);
    float mod_alpha = vfactor * v_alpha;
    float mod_beta = vfactor * v_beta;
    if (!enqueue_modulation_timings(mod_alpha, mod_beta))
        return false;
    log_timing(TIMING_LOG_FOC_VOLTAGE);
    return true;
}

bool Motor::enqueue_brushed_voltage_timings(float voltage_setpoint) {

    // Clamp voltage_setpoint to just shy of vbus voltage.
    if((-vbus_voltage * BRUSHED_VOLTAGE_MAX_RATIO) > voltage_setpoint)
      voltage_setpoint = -vbus_voltage * BRUSHED_VOLTAGE_MAX_RATIO;
    if((vbus_voltage * BRUSHED_VOLTAGE_MAX_RATIO) < voltage_setpoint)
      voltage_setpoint = vbus_voltage * BRUSHED_VOLTAGE_MAX_RATIO;

    float drive_ratio = voltage_setpoint/vbus_voltage;

    float tA, tB, tC;

    tA = 0.5f;
    tB = (drive_ratio + 1.0f)/2.0f;
    tC = 1.0f - tB;

    next_timings_[0] = (uint16_t)(tA * (float)TIM_1_8_PERIOD_CLOCKS);
    next_timings_[1] = (uint16_t)(tB * (float)TIM_1_8_PERIOD_CLOCKS);
    next_timings_[2] = (uint16_t)(tC * (float)TIM_1_8_PERIOD_CLOCKS);
    next_timings_valid_ = true;
    return true;
}

bool Motor::enqueue_brushed_current_timings(float current_setpoint) {

    // Syntactic sugar
    CurrentControl_t& ictrl = current_control_;

    if (fabsf(current_meas_.phB) > ictrl.overcurrent_trip_level
     || fabsf(current_meas_.phC) > ictrl.overcurrent_trip_level) {
        set_error(ERROR_CURRENT_SENSE_SATURATION);
    }


    // Current is just average od phB and phC where they have opposing signs.
    float Iq = 0.5f * (current_meas_.phC - current_meas_.phB);

    // Used to read measured current from host.
    ictrl.Iq_measured = Iq;

    // This relates to overvoltage conditions when reversing the motor.
    // Obviously this hard coded value is a hack. We could find a better way to
    // detect overvoltage, or add a max_voltage term to the ictrl object.
    // TODO(tlalexander): This is a hack. See notes above.
    if(vbus_voltage > 50)
    {
      ictrl.Ibus = (vbus_voltage-50.0f) * -5.0f;
    } else
    {
      ictrl.Ibus = 0.0f;
    }

    float Ierr_q = current_setpoint - Iq;

    //Integral control calculation.
    ictrl.v_current_control_integral_q += Ierr_q * (ictrl.i_gain * current_meas_period);
    float Vq = ictrl.v_current_control_integral_q + Ierr_q * ictrl.p_gain;


    // TODO(tlalexander): final_v_beta has been hijacked to use as voltage ramp
    // rate control. This is used to smooth high frequency voltage oscillation
    // with high current gain values. Perhaps a dedicated term could be added
    // to the protocol for this.
    float V_ramp_limit;
    if(ictrl.final_v_beta > 0.0001)
    {
      V_ramp_limit = ictrl.final_v_beta;
    }else
    {
      V_ramp_limit = 1.0;
    }
    // Clamping voltage ramp rate.
    if(Vq - ictrl.final_v_alpha > V_ramp_limit)
    {
      Vq = ictrl.final_v_alpha + V_ramp_limit;
    }

    if(Vq - ictrl.final_v_alpha < -V_ramp_limit)
    {
      Vq = ictrl.final_v_alpha - V_ramp_limit;
    }

    // Clamp commanded voltage to maximums.
    if((-vbus_voltage * BRUSHED_VOLTAGE_MAX_RATIO) > Vq)
    {
      Vq = -vbus_voltage * BRUSHED_VOLTAGE_MAX_RATIO;
    }

    if((vbus_voltage * BRUSHED_VOLTAGE_MAX_RATIO) < Vq)
    {
      Vq = vbus_voltage * BRUSHED_VOLTAGE_MAX_RATIO;
    }

    // Set final_v_alpha to final voltage used so it can be read by the host.
    ictrl.final_v_alpha = Vq;

    return enqueue_brushed_voltage_timings(Vq);
}
// We should probably make FOC Current call FOC Voltage to avoid duplication.
bool Motor::FOC_voltage(float v_d, float v_q, float pwm_phase) {
    float c = our_arm_cos_f32(pwm_phase);
    float s = our_arm_sin_f32(pwm_phase);
    float v_alpha = c*v_d - s*v_q;
    float v_beta = c*v_q + s*v_d;
    return enqueue_voltage_timings(v_alpha, v_beta);
}

bool Motor::FOC_current(float Id_des, float Iq_des, float I_phase, float pwm_phase) {
    // Syntactic sugar
    CurrentControl_t& ictrl = current_control_;

    // For Reporting
    ictrl.Iq_setpoint = Iq_des;

    // Check for current sense saturation
    if (std::abs(current_meas_.phB) > ictrl.overcurrent_trip_level || std::abs(current_meas_.phC) > ictrl.overcurrent_trip_level) {
        set_error(ERROR_CURRENT_SENSE_SATURATION);
        return false;
    }

    // Clarke transform
    float Ialpha = -current_meas_.phB - current_meas_.phC;
    float Ibeta = one_by_sqrt3 * (current_meas_.phB - current_meas_.phC);

    // Park transform
    float c_I = our_arm_cos_f32(I_phase);
    float s_I = our_arm_sin_f32(I_phase);
    float Id = c_I * Ialpha + s_I * Ibeta;
    float Iq = c_I * Ibeta - s_I * Ialpha;
    ictrl.Iq_measured += ictrl.I_measured_report_filter_k * (Iq - ictrl.Iq_measured);
    ictrl.Id_measured += ictrl.I_measured_report_filter_k * (Id - ictrl.Id_measured);

    // Check for violation of current limit
    float I_trip = effective_current_lim() + config_.current_lim_margin;
    if (SQ(Id) + SQ(Iq) > SQ(I_trip)) {
        set_error(ERROR_CURRENT_LIMIT_VIOLATION);
        return false;
    }

    // Current error
    float Ierr_d = Id_des - Id;
    float Ierr_q = Iq_des - Iq;

    // TODO look into feed forward terms (esp omega, since PI pole maps to RL tau)
    // Apply PI control
    float Vd = ictrl.v_current_control_integral_d + Ierr_d * ictrl.p_gain;
    float Vq = ictrl.v_current_control_integral_q + Ierr_q * ictrl.p_gain;

    float mod_to_V = (2.0f / 3.0f) * vbus_voltage;
    float V_to_mod = 1.0f / mod_to_V;
    float mod_d = V_to_mod * Vd;
    float mod_q = V_to_mod * Vq;

    // Vector modulation saturation, lock integrator if saturated
    // TODO make maximum modulation configurable
    float mod_scalefactor = 0.80f * sqrt3_by_2 * 1.0f / sqrtf(mod_d * mod_d + mod_q * mod_q);
    if (mod_scalefactor < 1.0f) {
        mod_d *= mod_scalefactor;
        mod_q *= mod_scalefactor;
        // TODO make decayfactor configurable
        ictrl.v_current_control_integral_d *= 0.99f;
        ictrl.v_current_control_integral_q *= 0.99f;
    } else {
        ictrl.v_current_control_integral_d += Ierr_d * (ictrl.i_gain * current_meas_period);
        ictrl.v_current_control_integral_q += Ierr_q * (ictrl.i_gain * current_meas_period);
    }

    // Compute estimated bus current
    ictrl.Ibus = mod_d * Id + mod_q * Iq;

    // Inverse park transform
    float c_p = our_arm_cos_f32(pwm_phase);
    float s_p = our_arm_sin_f32(pwm_phase);
    float mod_alpha = c_p * mod_d - s_p * mod_q;
    float mod_beta = c_p * mod_q + s_p * mod_d;

    // Report final applied voltage in stationary frame (for sensorles estimator)
    ictrl.final_v_alpha = mod_to_V * mod_alpha;
    ictrl.final_v_beta = mod_to_V * mod_beta;

    // Apply SVM
    if (!enqueue_modulation_timings(mod_alpha, mod_beta))
        return false; // error set inside enqueue_modulation_timings
    log_timing(TIMING_LOG_FOC_CURRENT);

    if (axis_->axis_num_ == 0) {

        // Edit these to suit your capture needs
        float trigger_data = ictrl.v_current_control_integral_d;
        float trigger_threshold = 0.5f;
        float sample_data = Ialpha;

        static bool ready = false;
        static bool capturing = false;
        if (trigger_data < trigger_threshold) {
            ready = true;
        }
        if (ready && trigger_data >= trigger_threshold) {
            capturing = true;
            ready = false;
        }
        if (capturing) {
            oscilloscope[oscilloscope_pos] = sample_data;
            if (++oscilloscope_pos >= OSCILLOSCOPE_SIZE) {
                oscilloscope_pos = 0;
                capturing = false;
            }
        }
    }

    return true;
}

// torque_setpoint [Nm]
// phase [rad electrical]
// phase_vel [rad/s electrical]
bool Motor::update(float torque_setpoint, float phase, float phase_vel) {
    float current_setpoint = 0.0f;
    phase *= config_.direction;
    phase_vel *= config_.direction;

    if (config_.motor_type == MOTOR_TYPE_ACIM) {
        current_setpoint = torque_setpoint / (config_.torque_constant * fmax(current_control_.acim_rotor_flux, config_.acim_gain_min_flux));
    }
    else {
        current_setpoint = torque_setpoint / config_.torque_constant;
    }
    current_setpoint *= config_.direction;

    // TODO: 2-norm vs independent clamping (current could be sqrt(2) bigger)
    float ilim = effective_current_lim();
    float id = std::clamp(current_control_.Id_setpoint, -ilim, ilim);
    float iq = std::clamp(current_setpoint, -ilim, ilim);

    if (config_.motor_type == MOTOR_TYPE_ACIM) {
        // Note that the effect of the current commands on the real currents is actually 1.5 PWM cycles later
        // However the rotor time constant is (usually) so slow that it doesn't matter
        // So we elect to write it as if the effect is immediate, to have cleaner code

        if (config_.acim_autoflux_enable) {
            float abs_iq = fabsf(iq);
            float gain = abs_iq > id ? config_.acim_autoflux_attack_gain : config_.acim_autoflux_decay_gain;
            id += gain * (abs_iq - id) * current_meas_period;
            id = std::clamp(id, config_.acim_autoflux_min_Id, ilim);
            current_control_.Id_setpoint = id;
        }

        // acim_rotor_flux is normalized to units of [A] tracking Id; rotor inductance is unspecified
        float dflux_by_dt = config_.acim_slip_velocity * (id - current_control_.acim_rotor_flux);
        current_control_.acim_rotor_flux += dflux_by_dt * current_meas_period;
        float slip_velocity = config_.acim_slip_velocity * (iq / current_control_.acim_rotor_flux);
        // Check for issues with small denominator. Polarity of check to catch NaN too
        bool acceptable_vel = fabsf(slip_velocity) <= 0.1f * (float)current_meas_hz;
        if (!acceptable_vel)
            slip_velocity = 0.0f;
        phase_vel += slip_velocity;
        // reporting only:
        current_control_.async_phase_vel = slip_velocity;

        current_control_.async_phase_offset += slip_velocity * current_meas_period;
        current_control_.async_phase_offset = wrap_pm_pi(current_control_.async_phase_offset);
        phase += current_control_.async_phase_offset;
        phase = wrap_pm_pi(phase);
    }

    float pwm_phase = phase + 1.5f * current_meas_period * phase_vel;

    // Execute current command
    switch(config_.motor_type){
        case MOTOR_TYPE_HIGH_CURRENT: return FOC_current(id, iq, phase, pwm_phase); break;
        case MOTOR_TYPE_ACIM: return FOC_current(id, iq, phase, pwm_phase); break;
        case MOTOR_TYPE_GIMBAL: return FOC_voltage(id, iq, pwm_phase); break;
        //In brushed motor mode, current is reinterptreted as voltage.
	case MOTOR_TYPE_BRUSHED_CURRENT: return enqueue_brushed_current_timings(current_setpoint); break;
        case MOTOR_TYPE_BRUSHED_VOLTAGE: return enqueue_brushed_voltage_timings(current_setpoint); break;
        default: set_error(ERROR_NOT_IMPLEMENTED_MOTOR_TYPE); return false; break;
    }
    return true;
}
