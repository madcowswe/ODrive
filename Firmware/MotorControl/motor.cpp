
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
    static const float kMargin = 0.90f;
    static const float kTripMargin = 1.0f; // Trip level is at edge of linear range of amplifer
    static const float max_output_swing = 1.35f; // [V] out of amplifier
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
        drv_fault_ = DRV8301_getFaultType(&gate_driver_);
        // Update/Cache all SPI device registers
        // DRV_SPI_8301_Vars_t* local_regs = &gate_driver_regs_;
        // local_regs->RcvCmd = true;
        // DRV8301_readData(&gate_driver_, local_regs);
        return false;
    };
    return true;
}

void Motor::set_error(Motor::Error_t error){
    error_ |= error;
    axis_->error_ |= Axis::ERROR_MOTOR_FAILED;
    safety_critical_disarm_motor_pwm(*this);
    update_brake_current();
}

float Motor::get_inverter_temp() {
    float adc = adc_measurements_[hw_config_.inverter_thermistor_adc_ch];
    float normalized_voltage = adc / adc_full_scale;
    return horner_fma(normalized_voltage, thermistor_poly_coeffs, thermistor_num_coeffs);
}

bool Motor::update_thermal_limits() {
    float fet_temp = get_inverter_temp();
    float temp_margin = config_.inverter_temp_limit_upper - fet_temp;
    float derating_range = config_.inverter_temp_limit_upper - config_.inverter_temp_limit_lower;
    thermal_current_lim_ = config_.current_lim * (temp_margin / derating_range);
    if (!(thermal_current_lim_ >= 0.0f)) { //Funny polarity to also catch NaN
        thermal_current_lim_ = 0.0f;
    }
    if (fet_temp > config_.inverter_temp_limit_upper + 5) {
        set_error(ERROR_INVERTER_OVER_TEMP);
        return false;
    }
    return true;
}

bool Motor::do_checks() {
    if (!check_DRV_fault()) {
        set_error(ERROR_DRV_FAULT);
        return false;
    }
    if (!update_thermal_limits()) {
        //error already set in function
        return false;
    }
    return true;
}

float Motor::effective_current_lim() {
    // Configured limit
    float current_lim = config_.current_lim;
    // Hardware limit
    if (axis_->motor_.config_.motor_type == Motor::MOTOR_TYPE_GIMBAL) {
        current_lim = std::min(current_lim, 0.98f*one_by_sqrt3*vbus_voltage);
    } else {
        current_lim = std::min(current_lim, axis_->motor_.current_control_.max_allowed_current);
    }
    // Thermal limit
    current_lim = std::min(current_lim, thermal_current_lim_);

    return current_lim;
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
    static const int num_test_cycles = static_cast<int>(3.0f / CURRENT_MEAS_PERIOD); // Test runs for 3s
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
    if (config_.motor_type == MOTOR_TYPE_HIGH_CURRENT) {
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
// We should probably make FOC Current call FOC Voltage to avoid duplication.
bool Motor::FOC_voltage(float v_d, float v_q, float pwm_phase) {
    float c = our_arm_cos_f32(pwm_phase);
    float s = our_arm_sin_f32(pwm_phase);
    float v_alpha = c*v_d - s*v_q;
    float v_beta  = c*v_q + s*v_d;
    return enqueue_voltage_timings(v_alpha, v_beta);
}

bool Motor::FOC_current(float Id_des, float Iq_des, float I_phase, float pwm_phase) {
    // Syntactic sugar
    CurrentControl_t& ictrl = current_control_;

    // For Reporting
    ictrl.Iq_setpoint = Iq_des;

    // Check for current sense saturation
    if (fabsf(current_meas_.phB) > ictrl.overcurrent_trip_level
     || fabsf(current_meas_.phC) > ictrl.overcurrent_trip_level) {
        set_error(ERROR_CURRENT_SENSE_SATURATION);
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
    float mod_beta  = c_p * mod_q + s_p * mod_d;

    // Report final applied voltage in stationary frame (for sensorles estimator)
    ictrl.final_v_alpha = mod_to_V * mod_alpha;
    ictrl.final_v_beta = mod_to_V * mod_beta;

    // Apply SVM
    if (!enqueue_modulation_timings(mod_alpha, mod_beta))
        return false; // error set inside enqueue_modulation_timings
    log_timing(TIMING_LOG_FOC_CURRENT);

    return true;
}


bool Motor::update(float current_setpoint, float phase, float phase_vel) {
    current_setpoint *= config_.direction;
    phase *= config_.direction;
    phase_vel *= config_.direction;

    float pwm_phase = phase + 1.5f * current_meas_period * phase_vel;

    // Execute current command
    // TODO: move this into the mot
    if (config_.motor_type == MOTOR_TYPE_HIGH_CURRENT) {
        if(!FOC_current(0.0f, current_setpoint, phase, pwm_phase)){
            return false;
        }
    } else if (config_.motor_type == MOTOR_TYPE_GIMBAL) {
        //In gimbal motor mode, current is reinterptreted as voltage.
        if(!FOC_voltage(0.0f, current_setpoint, pwm_phase))
            return false;
    } else if (config_.motor_type == MOTOR_TYPE_BRUSHED_CURRENT) {
        //TBD
    } else if (config_.motor_type == MOTOR_TYPE_BRUSHED_VOLTAGE) {
        //In brushed motor mode, current is reinterptreted as voltage.
        enqueue_brushed_voltage_timings(current_setpoint);
    } else {
        set_error(ERROR_NOT_IMPLEMENTED_MOTOR_TYPE);
        return false;
    }
    return true;
}
