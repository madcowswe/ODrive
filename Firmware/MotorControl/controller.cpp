
#include "odrive_main.h"
#include "arm_const_structs.h"


Controller::Controller(Config_t& config) :
    config_(config)
{}

void Controller::reset() {
    pos_setpoint_ = 0.0f;
    vel_setpoint_ = 0.0f;
    vel_integrator_current_ = 0.0f;
    current_setpoint_ = 0.0f;
}

void Controller::set_error(Error_t error) {
    error_ |= error;
    axis_->error_ |= Axis::ERROR_CONTROLLER_FAILED;
}

//--------------------------------
// Command Handling
//--------------------------------

void Controller::set_pos_setpoint(float pos_setpoint, float vel_feed_forward, float current_feed_forward) {
    pos_setpoint_ = pos_setpoint;
    vel_setpoint_ = vel_feed_forward;
    current_setpoint_ = current_feed_forward;
    config_.control_mode = CTRL_MODE_POSITION_CONTROL;
#ifdef DEBUG_PRINT
    printf("POSITION_CONTROL %6.0f %3.3f %3.3f\n", pos_setpoint, vel_setpoint_, current_setpoint_);
#endif
}

void Controller::set_vel_setpoint(float vel_setpoint, float current_feed_forward) {
    vel_setpoint_ = vel_setpoint;
    current_setpoint_ = current_feed_forward;
    config_.control_mode = CTRL_MODE_VELOCITY_CONTROL;
#ifdef DEBUG_PRINT
    printf("VELOCITY_CONTROL %3.3f %3.3f\n", vel_setpoint_, motor->current_setpoint_);
#endif
}

void Controller::set_current_setpoint(float current_setpoint) {
    current_setpoint_ = current_setpoint;
    config_.control_mode = CTRL_MODE_CURRENT_CONTROL;
#ifdef DEBUG_PRINT
    printf("CURRENT_CONTROL %3.3f\n", current_setpoint_);
#endif
}

void Controller::move_to_pos(float goal_point) {
    axis_->trap_.planTrapezoidal(goal_point, pos_setpoint_, vel_setpoint_,
                                 axis_->trap_.config_.vel_limit,
                                 axis_->trap_.config_.accel_limit,
                                 axis_->trap_.config_.decel_limit);
    traj_start_loop_count_ = axis_->loop_counter_;
    config_.control_mode = CTRL_MODE_TRAJECTORY_CONTROL;
}

void Controller::start_anticogging_calibration() {
    // Ensure the cogging map was correctly allocated earlier and that the motor is capable of calibrating
    if (anticogging_.cogging_map != NULL && axis_->error_ == Axis::ERROR_NONE) {
        anticogging_.calib_anticogging = true;
    }
}

/*
 * This anti-cogging implementation iterates through each encoder position,
 * waits for zero velocity & position error,
 * then samples the current required to maintain that position.
 * 
 * This holding current is added as a feedforward term in the control loop.
 */
bool Controller::anticogging_calibration(float pos_estimate, float vel_estimate) {
    if (anticogging_.calib_anticogging && anticogging_.cogging_map != NULL) {
        float pos_err = anticogging_.index - pos_estimate;
        if (fabsf(pos_err) <= anticogging_.calib_pos_threshold &&
            fabsf(vel_estimate) < anticogging_.calib_vel_threshold) {
            anticogging_.cogging_map[anticogging_.index++] = vel_integrator_current_;
        }
        if (anticogging_.index < axis_->encoder_.config_.cpr) { // TODO: remove the dependency on encoder CPR
            set_pos_setpoint(anticogging_.index, 0.0f, 0.0f);
            return false;
        } else {
            anticogging_.index = 0;
            set_pos_setpoint(0.0f, 0.0f, 0.0f);  // Send the motor home
            extract_harmonics();
            anticogging_.use_anticogging = true;  // We're good to go, enable anti-cogging
            anticogging_.calib_anticogging = false;
            return true;
        }
    }
    return false;
}
/*
 * This function takes the cogging map in the 'time' domain, performs an FFT,
 * finds the frequencies with the highest magnitudes (ignoring zero) and saves them to
 * the config structure.
 */
void Controller::extract_harmonics(){
    int32_t fft_size = axis_->encoder_.config_.cpr;

    // Determine what sized fft to use:
    // TODO: Handle non power series of two FFTs
    // TODO: We could do this with RFFTs
    const arm_cfft_instance_f32 *S;
    switch (fft_size) {
      case 512:  S = &arm_cfft_sR_f32_len512;  break;
      case 1024: S = &arm_cfft_sR_f32_len1024; break;
      case 2048: S = &arm_cfft_sR_f32_len2048; break;
      case 4096: S = &arm_cfft_sR_f32_len4096; break;
    };

    float *fft_array = (float*)malloc(2 * fft_size * sizeof(float));
    float *mag_array = (float*)malloc((fft_size /2) * sizeof(float));

    for(int i = 0; i < fft_size; i++){
        fft_array[i*2] = anticogging_.cogging_map[i];
        fft_array[(i*2)+1] = 0;
    }

    arm_cfft_f32(S, fft_array, 0, 1);
    // real ffts are symmetric, we dont need half this array...
    arm_cmplx_mag_f32(fft_array, mag_array, fft_size/2);

    // ignore the DC component
    mag_array[0] = 0;

    // we need to find the NUM_HARMONICS highest magnitudes
    uint32_t indices[NUM_HARMONICS];
    find_n_highest_indices(mag_array, fft_size/2, indices, NUM_HARMONICS);
    
    for (int i = 0; i < NUM_HARMONICS; i++){
        uint32_t index = indices[i];
        config_.harmonics[i].index = index;
        config_.harmonics[i].real = fft_array[index *2];
        config_.harmonics[i].imaginary = fft_array[(index*2) +1];
    }

    free(fft_array);
    free(mag_array);
}
/*
 * Find the indices of the n highest values in an array
 */
void Controller::find_n_highest_indices(float* arr, uint32_t arr_size, uint32_t* indices, uint32_t n){
    float *min_sorted_maxes = (float*)malloc(n * sizeof(float));

    for(uint32_t i = 0; i < n-1; i++){
        min_sorted_maxes[i] = 0;
        indices[i] = 0;
    }
    min_sorted_maxes[n-1] = arr[0];
    indices[n-1] = 0;

    for(uint32_t i = 1; i < arr_size; i++){
        float val = arr[i];
        if(val > min_sorted_maxes[0]){
            for(uint32_t j = 1; j <= n; j++){
                if((j < n) && (val > min_sorted_maxes[j])){
                    min_sorted_maxes[j-1] = min_sorted_maxes[j];
                    indices[j-1] = indices[j];
                }
                else{
                    min_sorted_maxes[j-1] = val;
                    indices[j-1] = i;
                    break;
                }
            }
        }
    }
    free(min_sorted_maxes);
}
float Controller::write_harmonics(int32_t index, int32_t harmonic, int32_t imag, float value){
    if (!std::isnan(value)){
        config_.harmonics[index].index = harmonic;
        imag ? config_.harmonics[index].imaginary = value : config_.harmonics[index].real = value;
    }
    return imag ? config_.harmonics[index].imaginary: config_.harmonics[index].real;
}

float Controller::write_anticogging_map(int32_t index, float value){
    if (anticogging_.cogging_map != NULL && !std::isnan(value))
        anticogging_.cogging_map[index] = value;
    return anticogging_.cogging_map[index];
}

void Controller::init_anticogging_map(){
    if (anticogging_.cogging_map == NULL)
        return;
    int32_t fft_size = axis_->encoder_.config_.cpr;

    // Determine what sized fft to use:
    // TODO: Handle non power series of two FFTs
    const static arm_cfft_instance_f32 *S;
    switch (fft_size) {
      case 512: S = &arm_cfft_sR_f32_len512; break;
      case 1024: S = &arm_cfft_sR_f32_len1024; break;
      case 2048: S = &arm_cfft_sR_f32_len2048; break;
      case 4096: S = &arm_cfft_sR_f32_len4096; break;
    };

    float *fft_array = (float*)malloc(2 * fft_size * sizeof(float));
    for(int i = 0; i < fft_size*2; i++){
        fft_array[i] = 0;
    }

    for(int i = 0; i < NUM_HARMONICS; i++){
        uint32_t index = config_.harmonics[i].index;
        if(index < (uint32_t)axis_->encoder_.config_.cpr){
            fft_array[index * 2] = config_.harmonics[i].real;
            fft_array[(index * 2)+1] = config_.harmonics[i].imaginary;
        }
    }

    arm_cfft_f32(S, fft_array, 1, 1);
    for(int i = 0; i < fft_size; i++){
        //-- seems to be half the value of python ifft
        anticogging_.cogging_map[i] = fft_array[i*2] * 2;
    }
    free(fft_array);
    anticogging_.use_anticogging = true;
}

bool Controller::update(float pos_estimate, float vel_estimate, float* current_setpoint_output) {
    // Only runs if anticogging_.calib_anticogging is true; non-blocking
    anticogging_calibration(pos_estimate, vel_estimate);
    float anticogging_pos = pos_estimate;

    // Trajectory control
    if (config_.control_mode == CTRL_MODE_TRAJECTORY_CONTROL) {
        // Note: uint32_t loop count delta is OK across overflow
        // Beware of negative deltas, as they will not be well behaved due to uint!
        float t = (axis_->loop_counter_ - traj_start_loop_count_) * current_meas_period;
        if (t > axis_->trap_.Tf_) {
            // Drop into position control mode when done to avoid problems on loop counter delta overflow
            config_.control_mode = CTRL_MODE_POSITION_CONTROL;
            // pos_setpoint already set by trajectory
            vel_setpoint_ = 0.0f;
            current_setpoint_ = 0.0f;
        } else {
            TrapezoidalTrajectory::Step_t traj_step = axis_->trap_.eval(t);
            pos_setpoint_ = traj_step.Y;
            vel_setpoint_ = traj_step.Yd;
            current_setpoint_ = traj_step.Ydd * axis_->trap_.config_.A_per_css;
        }
        anticogging_pos = pos_setpoint_; // FF the position setpoint instead of the pos_estimate
    }

    // Ramp rate limited velocity setpoint
    if (config_.control_mode == CTRL_MODE_VELOCITY_CONTROL && vel_ramp_enable_) {
        float max_step_size = current_meas_period * config_.vel_ramp_rate;
        float full_step = vel_ramp_target_ - vel_setpoint_;
        float step;
        if (fabsf(full_step) > max_step_size) {
            step = std::copysignf(max_step_size, full_step);
        } else {
            step = full_step;
        }
        vel_setpoint_ += step;
    }

    // Position control
    // TODO Decide if we want to use encoder or pll position here
    float vel_des = vel_setpoint_;
    if (config_.control_mode >= CTRL_MODE_POSITION_CONTROL) {
        float pos_err;
        if (config_.setpoints_in_cpr) {
            // TODO this breaks the semantics that estimates come in on the arguments.
            // It's probably better to call a get_estimate that will arbitrate (enc vs sensorless) instead.
            float cpr = (float)(axis_->encoder_.config_.cpr);
            // Keep pos setpoint from drifting
            pos_setpoint_ = fmodf_pos(pos_setpoint_, cpr);
            // Circular delta
            pos_err = pos_setpoint_ - axis_->encoder_.pos_cpr_;
            pos_err = wrap_pm(pos_err, 0.5f * cpr);
        } else {
            pos_err = pos_setpoint_ - pos_estimate;
        }
        vel_des += config_.pos_gain * pos_err;
    }

    // Velocity limiting
    float vel_lim = config_.vel_limit;
    if (vel_des > vel_lim) vel_des = vel_lim;
    if (vel_des < -vel_lim) vel_des = -vel_lim;

    // Check for overspeed fault (done in this module (controller) for cohesion with vel_lim)
    if (config_.vel_limit_tolerance > 0.0f) { // 0.0f to disable
        if (fabsf(vel_estimate) > config_.vel_limit_tolerance * vel_lim) {
            set_error(ERROR_OVERSPEED);
            return false;
        }
    }

    // Velocity control
    float Iq = current_setpoint_;

    // Anti-cogging is enabled after calibration
    // We get the current position and apply a current feed-forward
    // ensuring that we handle negative encoder positions properly (-1 == motor->encoder.encoder_cpr - 1)
    if (anticogging_.use_anticogging) {
        Iq += anticogging_.cogging_map[mod(static_cast<int>(anticogging_pos), axis_->encoder_.config_.cpr)];
    }

    float v_err = vel_des - vel_estimate;
    if (config_.control_mode >= CTRL_MODE_VELOCITY_CONTROL) {
        Iq += config_.vel_gain * v_err;
    }

    // Velocity integral action before limiting
    Iq += vel_integrator_current_;

    // Current limiting
    float Ilim = std::min(axis_->motor_.config_.current_lim, axis_->motor_.current_control_.max_allowed_current);
    bool limited = false;
    if (Iq > Ilim) {
        limited = true;
        Iq = Ilim;
    }
    if (Iq < -Ilim) {
        limited = true;
        Iq = -Ilim;
    }

    // Velocity integrator (behaviour dependent on limiting)
    if (config_.control_mode < CTRL_MODE_VELOCITY_CONTROL) {
        // reset integral if not in use
        vel_integrator_current_ = 0.0f;
    } else {
        if (limited) {
            // TODO make decayfactor configurable
            vel_integrator_current_ *= 0.99f;
        } else {
            vel_integrator_current_ += (config_.vel_integrator_gain * current_meas_period) * v_err;
        }
    }

    if (current_setpoint_output) *current_setpoint_output = Iq;
    return true;
}
