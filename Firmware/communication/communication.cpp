
/* Includes ------------------------------------------------------------------*/

#include "communication.h"

#include "interface_usb.h"
#include "interface_uart.h"
#include "interface_can.hpp"
#include "interface_i2c.h"

#include "odrive_main.h"
#include "freertos_vars.h"
#include "utils.hpp"
#include "gpio_utils.hpp"

#include <cmsis_os.h>
#include <memory>
//#include <usbd_cdc_if.h>
//#include <usb_device.h>
//#include <usart.h>
#include <gpio.h>

#include <type_traits>

/* Private defines -----------------------------------------------------------*/
/* Private macros ------------------------------------------------------------*/
/* Private typedef -----------------------------------------------------------*/
/* Global constant data ------------------------------------------------------*/
/* Global variables ----------------------------------------------------------*/

uint64_t serial_number;
char serial_number_str[13]; // 12 digits + null termination

/* Private constant data -----------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

osThreadId comm_thread;
const uint32_t stack_size_comm_thread = 4096; // Bytes
volatile bool endpoint_list_valid = false;

/* Private function prototypes -----------------------------------------------*/
/* Function implementations --------------------------------------------------*/

void init_communication(void) {
    printf("hi!\r\n");

    // Start command handling thread
    osThreadDef(task_cmd_parse, communication_task, osPriorityNormal, 0, stack_size_comm_thread / sizeof(StackType_t));
    comm_thread = osThreadCreate(osThread(task_cmd_parse), NULL);

    while (!endpoint_list_valid)
        osDelay(1);
}

float oscilloscope[OSCILLOSCOPE_SIZE] = {0};
size_t oscilloscope_pos = 0;

// ERG - modeling characterization data structure after oscilloscope
float motor_characterize_data[4][MOTORCHARACTERIZEDATA_SIZE] = {0}; //the 4x128 version
uint8_t motorCharacterizeData_size = MOTORCHARACTERIZEDATA_SIZE; //for use in communication protocol
uint32_t motor_characterize_data_pos = 0;

// Thread to handle deffered processing of USB interrupt, and
// read commands out of the UART DMA circular buffer
void communication_task(void * ctx) {
    (void) ctx; // unused parameter

    // Allow main init to continue
    endpoint_list_valid = true;
    
    start_uart_server();
    start_usb_server();
    if (odrv.config_.enable_i2c_instead_of_can) {
        start_i2c_server();
    } else {
        odCAN->start_can_server();
    }

    for (;;) {
        osDelay(1000); // nothing to do
    }
}

extern "C" {
int _write(int file, const char* data, int len);
}

// @brief This is what printf calls internally
int _write(int file, const char* data, int len) {
#ifdef USB_PROTOCOL_STDOUT
    usb_stream_output_ptr->process_bytes((const uint8_t *)data, len, nullptr);
#endif
#ifdef UART_PROTOCOL_STDOUT
    uart4_stream_output_ptr->process_bytes((const uint8_t *)data, len, nullptr);
#endif
    return len;
}


#include "../autogen/function_stubs.hpp"

ODrive& ep_root = odrv;
#include "../autogen/endpoints.hpp"
