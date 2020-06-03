#ifndef __ODRIVE_MAIN_H
#define __ODRIVE_MAIN_H

// Note on central include scheme by Samuel:
// there are circular dependencies between some of the header files,
// e.g. the Motor header needs a forward declaration of Axis and vice versa
// so I figured I'd make one main header that takes care of
// the forward declarations and right ordering
// btw this pattern is not so uncommon, for instance IIRC the stdlib uses it too

#ifdef __cplusplus
#include <fibre/protocol.hpp>
extern "C" {
#endif

// STM specific includes
#include <stm32f4xx_hal.h>  // Sets up the correct chip specifc defines required by arm_math
#include <can.h>
#include <i2c.h>
#define ARM_MATH_CM4 // TODO: might change in future board versions
#include <arm_math.h>

// OS includes
#include <cmsis_os.h>

// Hardware configuration
#if HW_VERSION_MAJOR == 3
#include "board_config_v3.h"
#else
#error "unknown board version"
#endif

//default timeout waiting for phase measurement signals
#define PH_CURRENT_MEAS_TIMEOUT 2 // [ms]

// Period in [s]
static const float current_meas_period = CURRENT_MEAS_PERIOD;

// Frequency in [Hz]
static const int current_meas_hz = CURRENT_MEAS_HZ;

// extern const float elec_rad_per_enc;
extern uint32_t _reboot_cookie;
extern bool user_config_loaded_;

extern uint64_t serial_number;
extern char serial_number_str[13];

typedef struct {
    bool fully_booted;
    uint32_t uptime; // [ms]
    uint32_t min_heap_space; // FreeRTOS heap [Bytes]
    uint32_t min_stack_space_axis0; // minimum remaining space since startup [Bytes]
    uint32_t min_stack_space_axis1;
    uint32_t min_stack_space_comms;
    uint32_t min_stack_space_usb;
    uint32_t min_stack_space_uart;
    uint32_t min_stack_space_usb_irq;
    uint32_t min_stack_space_startup;
    uint32_t min_stack_space_can;

    uint32_t stack_usage_axis0;
    uint32_t stack_usage_axis1;
    uint32_t stack_usage_comms;
    uint32_t stack_usage_usb;
    uint32_t stack_usage_uart;
    uint32_t stack_usage_usb_irq;
    uint32_t stack_usage_startup;
    uint32_t stack_usage_can;
} SystemStats_t;
extern SystemStats_t system_stats_;

#ifdef __cplusplus
}

struct PWMMapping_t {
    endpoint_ref_t endpoint;
    float min = 0;
    float max = 0;
};

// @brief general user configurable board configuration
struct BoardConfig_t {
    bool enable_uart = true;
    bool enable_i2c_instead_of_can = false;
    bool enable_ascii_protocol_on_usb = true;
    float max_regen_current = 0.0f;
#if HW_VERSION_MAJOR == 3 && HW_VERSION_MINOR >= 5 && HW_VERSION_VOLTAGE >= 48
    float brake_resistance = 2.0f;     // [ohm]
#else
    float brake_resistance = 0.47f;     // [ohm]
#endif
    float dc_bus_undervoltage_trip_level = 8.0f;                        //<! [V] minimum voltage below which the motor stops operating
    float dc_bus_overvoltage_trip_level = 1.07f * HW_VERSION_VOLTAGE;   //<! [V] maximum voltage above which the motor stops operating.
                                                                        //<! This protects against cases in which the power supply fails to dissipate
                                                                        //<! the brake power if the brake resistor is disabled.
                                                                        //<! The default is 26V for the 24V board version and 52V for the 48V board version.

    /**
     * If enabled, if the measured DC voltage exceeds `dc_bus_overvoltage_ramp_start`,
     * the ODrive will sink more power than usual into the the brake resistor
     * in an attempt to bring the voltage down again.
     * 
     * The brake duty cycle is increased by the following amount:
     *  vbus_voltage == dc_bus_overvoltage_ramp_start  =>  brake_duty_cycle += 0%
     *  vbus_voltage == dc_bus_overvoltage_ramp_end  =>  brake_duty_cycle += 100%
     * 
     * Remarks:
     *  - This feature is active even when all motors are disarmed.
     *  - This feature is disabled if `brake_resistance` is non-positive.
     */
    bool enable_dc_bus_overvoltage_ramp = false;
    float dc_bus_overvoltage_ramp_start = 1.07f * HW_VERSION_VOLTAGE; //!< See `enable_dc_bus_overvoltage_ramp`.
                                                                      //!< Do not set this lower than your usual vbus_voltage,
                                                                      //!< unless you like fried brake resistors.
    float dc_bus_overvoltage_ramp_end = 1.07f * HW_VERSION_VOLTAGE; //!< See `enable_dc_bus_overvoltage_ramp`.
                                                                    //!< Must be larger than `dc_bus_overvoltage_ramp_start`,
                                                                    //!< otherwise the ramp feature is disabled.

    float dc_max_positive_current = INFINITY; // Max current [A] the power supply can source
    float dc_max_negative_current = -0.000001f; // Max current [A] the power supply can sink. You most likely want a non-positive value here. Set to -INFINITY to disable.
    PWMMapping_t pwm_mappings[GPIO_COUNT];
    PWMMapping_t analog_mappings[GPIO_COUNT];

    /**
     * Defines the baudrate used on the UART interface.
     * Some baudrates will have a small timing error due to hardware limitations.
     * 
     * Here's an (incomplete) list of baudrates for ODrive v3.x:
     * 
     *   Configured  | Actual        | Error [%]
     *  -------------|---------------|-----------
     *   1.2 KBps    | 1.2 KBps      | 0
     *   2.4 KBps    | 2.4 KBps      | 0
     *   9.6 KBps    | 9.6 KBps      | 0
     *   19.2 KBps   | 19.195 KBps   | 0.02
     *   38.4 KBps   | 38.391 KBps   | 0.02
     *   57.6 KBps   | 57.613 KBps   | 0.02
     *   115.2 KBps  | 115.068 KBps  | 0.11
     *   230.4 KBps  | 230.769 KBps  | 0.16
     *   460.8 KBps  | 461.538 KBps  | 0.16
     *   921.6 KBps  | 913.043 KBps  | 0.93
     *   1.792 MBps  | 1.826 MBps    | 1.9
     *   1.8432 MBps | 1.826 MBps    | 0.93
     * 
     * For more information refer to Section 30.3.4 and Table 142 (the column with f_PCLK = 42 MHz) in the STM datasheet:
     * https://www.st.com/content/ccc/resource/technical/document/reference_manual/3d/6d/5a/66/b4/99/40/d4/DM00031020.pdf/files/DM00031020.pdf/jcr:content/translations/en.DM00031020.pdf
     */
    uint32_t uart_baudrate = 115200;
};
extern BoardConfig_t board_config;
extern bool user_config_loaded_;

// Forward Declarations
class Axis;
class Motor;
class ODriveCAN;

constexpr size_t AXIS_COUNT = 2;
extern std::array<Axis*, AXIS_COUNT> axes;
extern ODriveCAN *odCAN;

// if you use the oscilloscope feature you can bump up this value
#define OSCILLOSCOPE_SIZE 4096
extern float oscilloscope[OSCILLOSCOPE_SIZE];
extern size_t oscilloscope_pos;

// TODO: move
// this is technically not thread-safe but practically it might be
#define DEFINE_ENUM_FLAG_OPERATORS(ENUMTYPE) \
inline ENUMTYPE operator | (ENUMTYPE a, ENUMTYPE b) { return static_cast<ENUMTYPE>(static_cast<std::underlying_type_t<ENUMTYPE>>(a) | static_cast<std::underlying_type_t<ENUMTYPE>>(b)); } \
inline ENUMTYPE operator & (ENUMTYPE a, ENUMTYPE b) { return static_cast<ENUMTYPE>(static_cast<std::underlying_type_t<ENUMTYPE>>(a) & static_cast<std::underlying_type_t<ENUMTYPE>>(b)); } \
inline ENUMTYPE operator ^ (ENUMTYPE a, ENUMTYPE b) { return static_cast<ENUMTYPE>(static_cast<std::underlying_type_t<ENUMTYPE>>(a) ^ static_cast<std::underlying_type_t<ENUMTYPE>>(b)); } \
inline ENUMTYPE &operator |= (ENUMTYPE &a, ENUMTYPE b) { return reinterpret_cast<ENUMTYPE&>(reinterpret_cast<std::underlying_type_t<ENUMTYPE>&>(a) |= static_cast<std::underlying_type_t<ENUMTYPE>>(b)); } \
inline ENUMTYPE &operator &= (ENUMTYPE &a, ENUMTYPE b) { return reinterpret_cast<ENUMTYPE&>(reinterpret_cast<std::underlying_type_t<ENUMTYPE>&>(a) &= static_cast<std::underlying_type_t<ENUMTYPE>>(b)); } \
inline ENUMTYPE &operator ^= (ENUMTYPE &a, ENUMTYPE b) { return reinterpret_cast<ENUMTYPE&>(reinterpret_cast<std::underlying_type_t<ENUMTYPE>&>(a) ^= static_cast<std::underlying_type_t<ENUMTYPE>>(b)); } \
inline ENUMTYPE operator ~ (ENUMTYPE a) { return static_cast<ENUMTYPE>(~static_cast<std::underlying_type_t<ENUMTYPE>>(a)); }


// ODrive specific includes
#include <utils.hpp>
#include <gpio_utils.hpp>
#include <low_level.h>
#include <motor.hpp>
#include <encoder.hpp>
#include <sensorless_estimator.hpp>
#include <controller.hpp>
#include <trapTraj.hpp>
#include <endstop.hpp>
#include <axis.hpp>
#include <communication/communication.h>

#endif // __cplusplus


// general system functions defined in main.cpp
void save_configuration(void);
void erase_configuration(void);
void enter_dfu_mode(void);

#endif /* __ODRIVE_MAIN_H */
