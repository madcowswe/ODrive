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

//TODO clean this up
static const float current_meas_period = CURRENT_MEAS_PERIOD;
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
} SystemStats_t;
extern SystemStats_t system_stats_;

#ifdef __cplusplus
}

struct PWMMapping_t {
    endpoint_ref_t endpoint = { 0 };
    float min = 0;
    float max = 0;
};

// @brief general user configurable board configuration
struct BoardConfig_t {
    bool enable_uart = true;
    bool enable_i2c_instead_of_can = false;
    bool enable_ascii_protocol_on_usb = true;
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
    PWMMapping_t pwm_mappings[GPIO_COUNT];
    PWMMapping_t analog_mappings[GPIO_COUNT];
};
extern BoardConfig_t board_config;
extern bool user_config_loaded_;

class Axis;
class Motor;

constexpr size_t AXIS_COUNT = 2;
extern Axis *axes[AXIS_COUNT];

// if you use the oscilloscope feature you can bump up this value
#define OSCILLOSCOPE_SIZE 128
extern float oscilloscope[OSCILLOSCOPE_SIZE];
extern size_t oscilloscope_pos;

// TODO: See why this is needed. I saw DRV errors above this sometimes.
#define BRUSHED_VOLTAGE_MAX_RATIO 0.997f

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
#include <utils.h>
#include <low_level.h>
#include <encoder.hpp>
#include <sensorless_estimator.hpp>
#include <controller.hpp>
#include <motor.hpp>
#include <trapTraj.hpp>
#include <axis.hpp>
#include <communication/communication.h>

#endif // __cplusplus


// general system functions defined in main.cpp
void save_configuration(void);
void erase_configuration(void);
void enter_dfu_mode(void);

#endif /* __ODRIVE_MAIN_H */
