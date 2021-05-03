#!/usr/bin/env python3
"""
Example usage of the ODrive python library to monitor and control ODrive devices
"""

from __future__ import print_function

import odrive
from odrive.enums import *
from odrive.utils import dump_errors
import time
import math
import sys
import fibre


def idle_wait():
    while odrv0.axis1.current_state != AXIS_STATE_IDLE:
        time.sleep(0.1)
    print(dump_errors(odrv0))

# Find a connected ODrive (this will block until you connect one)
print("finding an odrive...")
odrv0 = odrive.find_any()

print(dump_errors(odrv0,True))

# Find an ODrive that is connected on the serial port /dev/ttyUSB0
#odrv0 = odrive.find_any("serial:/dev/ttyUSB0")


odrv0.axis1.motor.config.current_control_bandwidth = 20
odrv0.axis1.encoder.config.bandwidth = 100
odrv0.axis1.motor.current_control.p_gain = 0.3
odrv0.axis1.motor.current_control.i_gain = 0
odrv0.axis1.motor.current_control.final_v_beta = 0.1 # Voltage Ramp Rate


print(dump_errors(odrv0,True))
print("try current control")
odrv0.axis1.requested_state = AXIS_STATE_BRUSHED_CURRENT_CONTROL

print(dump_errors(odrv0,True))
odrv0.axis1.clear_errors()

odrv0.axis1.requested_state = AXIS_STATE_BRUSHED_VOLTAGE_CONTROL
print(dump_errors(odrv0,True))


odrv0.axis1.motor.current_control.p_gain = 50.0
odrv0.axis1.motor.current_control.i_gain = 0.0


odrv0.axis1.motor.current_control.final_v_beta = 0.1 # Voltage Ramp Rate



odrv0.axis1.controller.current_setpoint = 3

