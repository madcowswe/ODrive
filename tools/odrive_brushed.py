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


odrv0.axis1.requested_state = 11
odrv0.axis1.motor.current_control.p_gain = 50.0
odrv0.axis1.motor.current_control.i_gain = 0.0


odrv0.axis1.motor.current_control.final_v_beta = 0.1 # Voltage Ramp Rate


max_vel = 0
reverse = False
reverse_start_time = 0
reverse_duration_allowed_sec = 4
ctrl = odrv0.axis1.motor.current_control
start_time = time.time()
reverse_position = 0

print(odrv0.axis1.encoder.pos_estimate)
print(odrv0.axis1.encoder.pos_estimate)

odrv0.axis1.controller.current_setpoint = 3

# This is a current-based hard block homing routine.
while True:
    print("Iq_measured {}, bus voltage {}, vel estimate {}, max_vel {}.".format(ctrl.Iq_measured, odrv0.vbus_voltage, odrv0.axis1.encoder.vel_estimate, max_vel))
    if reverse:
        if odrv0.axis1.encoder.pos_estimate - reverse_position > 400 or time.time() - reverse_start_time > reverse_duration_allowed_sec:
            reverse = False
            odrv0.axis1.controller.current_setpoint *= -1
            max_vel = 0
            start_time = time.time()
            print(odrv0.axis1.encoder.pos_estimate)
            print(odrv0.axis1.encoder.pos_estimate)
    else:
        if abs(odrv0.axis1.encoder.vel_estimate) > max_vel:
            max_vel = abs(odrv0.axis1.encoder.vel_estimate)
        if max_vel > 100 and abs(odrv0.axis1.encoder.vel_estimate) < 10:
            print("Contact")
            break
        if max_vel < 400 and time.time()-start_time > 2.0:
            odrv0.axis1.controller.current_setpoint *= -1
            reverse = True
            reverse_start_time = time.time()
            reverse_position = odrv0.axis1.encoder.pos_estimate
    #if max_vel >
    time.sleep(0.1)


print(dump_errors(odrv0,True))
print(odrv0.axis1.encoder.pos_estimate)
print(odrv0.axis1.encoder.pos_estimate)
print(odrv0.axis1.encoder.pos_estimate)
odrv0.axis1.requested_state = AXIS_STATE_ENCODER_INDEX_SEARCH
idle_wait()
print("Index Search complete")
print(dump_errors(odrv0,True))

print(odrv0.axis1.encoder.pos_estimate)
print(odrv0.axis1.encoder.pos_estimate)



print("Bus voltage is " + str(odrv0.vbus_voltage) + "V")


timer = time.time()

time.sleep(2)

if True:
    odrv0.axis1.encoder.config.bandwidth = 100
    odrv0.axis1.controller.config.vel_ramp_rate = 100
    odrv0.axis1.controller.config.vel_gain = -0.03
    odrv0.axis1.controller.config.vel_integrator_gain = 0
    odrv0.axis1.controller.vel_integrator_current = 0
    odrv0.axis1.requested_state = AXIS_STATE_IDLE
    odrv0.axis1.controller.config.control_mode = CTRL_MODE_POSITION_CONTROL
    odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    odrv0.axis1.controller.config.pos_gain = 5
    odrv0.axis1.controller.vel_ramp_enable = True
    odrv0.axis1.controller.config.vel_ramp_rate = 1

    odrv0.axis1.motor.config.current_lim = 10.0
    odrv0.axis1.controller.config.vel_limit_tolerance = 2.5
    odrv0.axis1.controller.config.vel_limit = 2000
    odrv0.axis0.trap_traj.config.vel_limit = 2000
    odrv0.axis0.trap_traj.config.accel_limit = 200
    odrv0.axis0.trap_traj.config.decel_limit = 200
    odrv0.axis0.trap_traj.config.A_per_css = 0

    base_position = 1130/2
    offset = 400
    odrv0.axis1.controller.pos_setpoint = base_position
    min = base_position - offset
    count = 0
    steps = 6
    #sys.exit()
    while True:
        print("For setpoint {}, estimate is {}, current: {}, bus_voltage {}".format(odrv0.axis1.controller.pos_setpoint, odrv0.axis1.encoder.pos_estimate, ctrl.Iq_measured, odrv0.vbus_voltage))
        #odrv0.axis1.encoder.pos_estimate
        #print(dump_errors(odrv0))
        time.sleep(0.1)
        if time.time() - timer > 0.5:
            timer = time.time()
            count += 1
            #offset *= -1
            #odrv0.axis1.controller.move_to_pos(base_position + offset)
            if count > steps:
                count = 0
            odrv0.axis1.controller.move_to_pos(min + (2*offset)*count/steps)
            if count == 0:
                timer += 1.0
                #time.sleep(1)
        if odrv0.axis1.error:
            print(dump_errors(odrv0,True))
            print("Gate Driver: {}".format(odrv0.axis1.motor.gate_driver))
            print("For setpoint {}, estimate is {}, current: {}, bus_voltage {}".format(odrv0.axis1.controller.pos_setpoint, odrv0.axis1.encoder.pos_estimate, ctrl.Iq_measured, odrv0.vbus_voltage))
            sys.exit()
