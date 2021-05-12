#ifndef __AXIS_HPP
#define __AXIS_HPP

#ifndef __ODRIVE_MAIN_H
#error "This file should not be included directly. Include odrive_main.h instead."
#endif

#include <array>

class Axis : public ODriveIntf::AxisIntf {
public:
    struct LockinConfig_t {
        float current = 10.0f;           // [A]
        float ramp_time = 0.4f;          // [s]
        float ramp_distance = 1 * M_PI;  // [rad]
        float accel = 20.0f;     // [rad/s^2]
        float vel = 40.0f; // [rad/s]
        float finish_distance = 100.0f;  // [rad]
        bool finish_on_vel = false;
        bool finish_on_distance = false;
        bool finish_on_enc_idx = false;
    };

    static LockinConfig_t default_calibration();
    static LockinConfig_t default_sensorless();
    static LockinConfig_t default_lockin();

    struct Config_t {
        bool startup_motor_calibration = false;   //<! run motor calibration at startup, skip otherwise
        bool startup_encoder_index_search = false; //<! run encoder index search after startup, skip otherwise
                                                // this only has an effect if encoder.config.use_index is also true
        bool startup_encoder_offset_calibration = false; //<! run encoder offset calibration after startup, skip otherwise
        bool startup_closed_loop_control = false; //<! enable closed loop control after calibration/startup
        bool startup_sensorless_control = false; //<! enable sensorless control after calibration/startup
        bool startup_homing = false; //<! enable homing after calibration/startup

        bool enable_step_dir = false; //<! enable step/dir input after calibration
                                    //   For M0 this has no effect if enable_uart is true
        bool step_dir_always_on = false; //<! Keep step/dir enabled while the motor is disabled.
                                         //<! This is ignored if enable_step_dir is false.
                                         //<! This setting only takes effect on a state transition
                                         //<! into idle or out of closed loop control.

        float turns_per_step = 1.0f / 1024.0f;

        float watchdog_timeout = 0.0f; // [s]
        bool enable_watchdog = false;

        // Defaults loaded from hw_config in load_configuration in main.cpp
        uint16_t step_gpio_pin = 0;
        uint16_t dir_gpio_pin = 0;

        LockinConfig_t calibration_lockin = default_calibration();
        LockinConfig_t sensorless_ramp = default_sensorless();
        LockinConfig_t general_lockin;
        uint32_t can_node_id = 0; // Both axes will have the same id to start
        bool can_node_id_extended = false;
        uint32_t can_heartbeat_rate_ms = 100;

        // custom setters
        Axis* parent = nullptr;
        void set_step_gpio_pin(uint16_t value) { step_gpio_pin = value; parent->decode_step_dir_pins(); }
        void set_dir_gpio_pin(uint16_t value) { dir_gpio_pin = value; parent->decode_step_dir_pins(); }
    };

    //ERG - Parameters to be used by run_motor_characterize_input()
    struct InputConfig_t {
        InputType input_type = INPUT_TYPE_STEP; //see: InputType_t
        float test_delay = 2.0f;            // [s]
        float test_duration = 5.0f;	        // [s]
        float impulse_voltage = 2.0f;       // [V]
        uint32_t impulse_peakDuration = 1;  // [#] loopCount cycles, which run at 8kHz
        float step_voltage = 0.25f;		    // [V]
        float chirp_amplitude = 0.25f;	    // [V]
        float chirp_midline = 0.0f;		    // [V]
        float chirp_freqLow = 1.0f;		    // [Hz]
        float chirp_freqHigh = 1000.0f;	    // [Hz]
        uint32_t noise_max = 2;             // [#] percentage of voltage limit
    };

    void record_motor_characterize_data(float timestep, float voltage_setpoint); //ERG
    bool run_motor_characterize_input(); //ERG

    struct Homing_t {
        bool is_homed = false;
    };

    enum thread_signals {
        M_SIGNAL_PH_CURRENT_MEAS = 1u << 0
    };

    Axis(int axis_num,
            const AxisHardwareConfig_t& hw_config,
            Config_t& config,
            InputConfig_t& input_config, //ERG
            Encoder& encoder,
            SensorlessEstimator& sensorless_estimator,
            Controller& controller,
            OnboardThermistorCurrentLimiter& fet_thermistor,
            OffboardThermistorCurrentLimiter& motor_thermistor,
            Motor& motor,
            TrapezoidalTrajectory& trap,
            Endstop& min_endstop,
            Endstop& max_endstop);

    void setup();
    void start_thread();
    void signal_current_meas();
    bool wait_for_current_meas();

    void step_cb();
    void set_step_dir_active(bool enable);
    void decode_step_dir_pins();

    static void load_default_step_dir_pin_config(
        const AxisHardwareConfig_t& hw_config, Config_t* config);
    static void load_default_can_id(const int& id, Config_t& config);

    bool check_DRV_fault();
    bool check_PSU_brownout();
    bool do_checks();
    bool do_updates();

    void watchdog_feed();
    bool watchdog_check();

    void clear_errors() {
        motor_.error_ = Motor::ERROR_NONE;
        controller_.error_ = Controller::ERROR_NONE;
        sensorless_estimator_.error_ = SensorlessEstimator::ERROR_NONE;
        encoder_.error_ = Encoder::ERROR_NONE;
        encoder_.spi_error_rate_ = 0.0f;

        error_ = ERROR_NONE;
    }

    // True if there are no errors
    bool inline check_for_errors() {
        return error_ == ERROR_NONE;
    }

    // @brief Runs the specified update handler at the frequency of the current measurements.
    //
    // The loop runs until one of the following conditions:
    //  - update_handler returns false
    //  - the current measurement times out
    //  - the health checks fail (brownout, driver fault line)
    //  - update_handler doesn't update the modulation timings in time
    //    This criterion is ignored if current_state is AXIS_STATE_IDLE
    //
    // If update_handler is going to update the motor timings, you must call motor.arm()
    // shortly before this function.
    //
    // If the function returns, it is guaranteed that error is non-zero, except if the cause
    // for the exit was a negative return value of update_handler or an external
    // state change request (requested_state != AXIS_STATE_DONT_CARE).
    // Under all exit conditions the motor is disarmed and the brake current set to zero.
    // Furthermore, if the update_handler does not set the phase voltages in time, they will
    // go to zero.
    //
    // @tparam T Must be a callable type that takes no arguments and returns a bool
    template<typename T>
    void run_control_loop(const T& update_handler) {
        while (requested_state_ == AXIS_STATE_UNDEFINED) {
            // look for errors at axis level and also all subcomponents
            bool checks_ok = do_checks();
            // Update all estimators
            // Note: updates run even if checks fail
            bool updates_ok = do_updates(); 

            // make sure the watchdog is being fed. 
            bool watchdog_ok = watchdog_check();
            
            if (!checks_ok || !updates_ok || !watchdog_ok) {
                // It's not useful to quit idle since that is the safe action
                // Also leaving idle would rearm the motors
                if (current_state_ != AXIS_STATE_IDLE)
                    break;
            }

            // Run main loop function, defer quitting for after wait
            // TODO: change arming logic to arm after waiting
            bool main_continue = update_handler();

            // Check we meet deadlines after queueing
            ++loop_counter_;

            // Wait until the current measurement interrupt fires
            if (!wait_for_current_meas()) {
                // maybe the interrupt handler is dead, let's be
                // safe and float the phases
                safety_critical_disarm_motor_pwm(motor_);
                update_brake_current();
                error_ |= ERROR_CURRENT_MEASUREMENT_TIMEOUT;
                break;
            }

            if (!main_continue)
                break;
        }
    }

    bool run_lockin_spin(const LockinConfig_t &lockin_config);
    bool run_sensorless_control_loop();
    bool run_closed_loop_control_loop();
    bool run_homing();
    bool run_idle_loop();

    constexpr uint32_t get_watchdog_reset() {
        return static_cast<uint32_t>(std::clamp<float>(config_.watchdog_timeout, 0, UINT32_MAX / (current_meas_hz + 1)) * current_meas_hz);
    }

    void run_state_machine_loop();

    int axis_num_;
    const AxisHardwareConfig_t& hw_config_;
    Config_t& config_;
    InputConfig_t& input_config_; //ERG 

    Encoder& encoder_;
    SensorlessEstimator& sensorless_estimator_;
    Controller& controller_;
    OnboardThermistorCurrentLimiter& fet_thermistor_;
    OffboardThermistorCurrentLimiter& motor_thermistor_;
    Motor& motor_;
    TrapezoidalTrajectory& trap_traj_;
    Endstop& min_endstop_;
    Endstop& max_endstop_;

    // List of current_limiters and thermistors to
    // provide easy iteration.
    std::array<CurrentLimiter*, 2> current_limiters_;
    std::array<ThermistorCurrentLimiter*, 2> thermistors_;

    osThreadId thread_id_;
    const uint32_t stack_size_ = 2048; // Bytes
    volatile bool thread_id_valid_ = false;

    // variables exposed on protocol
    Error error_ = ERROR_NONE;
    bool step_dir_active_ = false; // auto enabled after calibration, based on config.enable_step_dir

    // updated from config in constructor, and on protocol hook
    GPIO_TypeDef* step_port_;
    uint16_t step_pin_;
    GPIO_TypeDef* dir_port_;
    uint16_t dir_pin_;

    AxisState requested_state_ = AXIS_STATE_STARTUP_SEQUENCE;
    std::array<AxisState, 10> task_chain_ = { AXIS_STATE_UNDEFINED };
    AxisState& current_state_ = task_chain_.front();
    uint32_t loop_counter_ = 0;
    LockinState lockin_state_ = LOCKIN_STATE_INACTIVE;
    Homing_t homing_;
    uint32_t last_heartbeat_ = 0;

    // watchdog
    uint32_t watchdog_current_value_= 0;
};


#endif /* __AXIS_HPP */
