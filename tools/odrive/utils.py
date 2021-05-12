from __future__ import print_function

import sys
import time
import threading
import platform
import subprocess
import os
import numpy as np
import matplotlib.pyplot as plt
from fibre.utils import Event
import odrive.enums
from odrive.enums import *

try:
    if platform.system() == 'Windows':
        import win32console
        import colorama
        colorama.init()
except ImportError:
    print("Could not init terminal features.")
    print("Refer to install instructions at http://docs.odriverobotics.com/#downloading-and-installing-tools")
    sys.stdout.flush()
    pass

if sys.version_info < (3, 0):
    input = raw_input

_VT100Colors = {
    'green': '\x1b[92;1m',
    'cyan': '\x1b[96;1m',
    'yellow': '\x1b[93;1m',
    'red': '\x1b[91;1m',
    'default': '\x1b[0m'
}

def calculate_thermistor_coeffs(degree, Rload, R_25, Beta, Tmin, Tmax, plot = False):
    T_25 = 25 + 273.15 #Kelvin
    temps = np.linspace(Tmin, Tmax, 1000)
    tempsK = temps + 273.15

    # https://en.wikipedia.org/wiki/Thermistor#B_or_%CE%B2_parameter_equation
    r_inf = R_25 * np.exp(-Beta/T_25)
    R_temps = r_inf * np.exp(Beta/tempsK)
    V = Rload / (Rload + R_temps)

    fit = np.polyfit(V, temps, degree)
    p1 = np.poly1d(fit)
    fit_temps = p1(V)

    if plot:
        print(fit)
        plt.plot(V, temps, label='actual')
        plt.plot(V, fit_temps, label='fit')
        plt.xlabel('normalized voltage')
        plt.ylabel('Temp [C]')
        plt.legend(loc=0)
        plt.show()

    return p1

class OperationAbortedException(Exception):
    pass

def set_motor_thermistor_coeffs(axis, Rload, R_25, Beta, Tmin, TMax):
    coeffs = calculate_thermistor_coeffs(3, Rload, R_25, Beta, Tmin, TMax)
    axis.motor_thermistor.config.poly_coefficient_0 = float(coeffs[3])
    axis.motor_thermistor.config.poly_coefficient_1 = float(coeffs[2])
    axis.motor_thermistor.config.poly_coefficient_2 = float(coeffs[1])
    axis.motor_thermistor.config.poly_coefficient_3 = float(coeffs[0])

def dump_errors(odrv, clear=False):
    axes = [(name, axis) for name, axis in odrv._remote_attributes.items() if 'axis' in name]
    axes.sort()
    for name, axis in axes:
        print(name)

        # Flatten axis and submodules
        # (name, remote_obj, errorcode)
        module_decode_map = [
            ('axis', axis, {k: v for k, v in odrive.enums.__dict__ .items() if k.startswith("AXIS_ERROR_")}),
            ('motor', axis.motor, {k: v for k, v in odrive.enums.__dict__ .items() if k.startswith("MOTOR_ERROR_")}),
            ('fet_thermistor', axis.fet_thermistor, {k: v for k, v in odrive.enums.__dict__ .items() if k.startswith("THERMISTOR_CURRENT_LIMITER_ERROR")}),
            ('motor_thermistor', axis.motor_thermistor, {k: v for k, v in odrive.enums.__dict__ .items() if k.startswith("THERMISTOR_CURRENT_LIMITER_ERROR")}),
            ('encoder', axis.encoder, {k: v for k, v in odrive.enums.__dict__ .items() if k.startswith("ENCODER_ERROR_")}),
            ('controller', axis.controller, {k: v for k, v in odrive.enums.__dict__ .items() if k.startswith("CONTROLLER_ERROR_")}),
        ]

        # Module error decode
        for name, remote_obj, errorcodes in module_decode_map:
            prefix = ' '*2 + name + ": "
            if (remote_obj.error != 0):
                foundError = False
                print(prefix + _VT100Colors['red'] + "Error(s):" + _VT100Colors['default'])
                errorcodes_tup = [(name, val) for name, val in errorcodes.items() if 'ERROR_' in name]
                for codename, codeval in errorcodes_tup:
                    if remote_obj.error & codeval != 0:
                        foundError = True
                        print("    " + codename)
                if not foundError:
                    print("    " + 'UNKNOWN ERROR!')
                if clear:
                    remote_obj.error = 0
            else:
                print(prefix + _VT100Colors['green'] + "no error" + _VT100Colors['default'])

def oscilloscope_dump(odrv, num_vals, filename='oscilloscope.csv'):
    with open(filename, 'w') as f:
        for x in range(num_vals):
            f.write(str(odrv.get_oscilloscope_val(x)))
            f.write('\n')

data_rate = 100
plot_rate = 10
num_samples = 1000
def start_liveplotter(get_var_callback):
    """
    Starts a liveplotter.
    The variable that is plotted is retrieved from get_var_callback.
    This function returns immediately and the liveplotter quits when
    the user closes it.
    """

    import matplotlib.pyplot as plt

    cancellation_token = Event()

    global vals
    vals = []
    def fetch_data():
        global vals
        while not cancellation_token.is_set():
            try:
                data = get_var_callback()
            except Exception as ex:
                print(str(ex))
                time.sleep(1)
                continue
            vals.append(data)
            if len(vals) > num_samples:
                vals = vals[-num_samples:]
            time.sleep(1/data_rate)

    # TODO: use animation for better UI performance, see:
    # https://matplotlib.org/examples/animation/simple_anim.html
    def plot_data():
        global vals

        plt.ion()

        # Make sure the script terminates when the user closes the plotter
        def did_close(evt):
            cancellation_token.set()
        fig = plt.figure()
        fig.canvas.mpl_connect('close_event', did_close)

        while not cancellation_token.is_set():
            plt.clf()
            plt.plot(vals)
            plt.legend(list(range(len(vals))))
            fig.canvas.draw()
            fig.canvas.start_event_loop(1/plot_rate)

    fetch_t = threading.Thread(target=fetch_data)
    fetch_t.daemon = True
    fetch_t.start()
    
    plot_t = threading.Thread(target=plot_data)
    plot_t.daemon = True
    plot_t.start()

    return cancellation_token;
    #plot_data()


class BulkCapture:
    '''
    Asynchronously captures a bulk set of data when instance is created.

    get_var_callback: a function that returns the data you want to collect (see the example below)
    data_rate: Rate in hz
    length: Length of time to capture in seconds

    Example Usage:
        capture = BulkCapture(lambda :[odrv0.axis0.encoder.pos_estimate, odrv0.axis0.controller.pos_setpoint])
        # Do stuff while capturing (like sending position commands)
        capture.event.wait() # When you're done doing stuff, wait for the capture to be completed.
        print(capture.data) # Do stuff with the data
        capture.plot_data() # Helper method to plot the data
    '''

    def __init__(self,
                 get_var_callback,
                 data_rate=500.0,
                 duration=2.0):
        from threading import Event, Thread
        import numpy as np

        self.get_var_callback = get_var_callback
        self.event = Event()
        def loop():
            vals = []
            start_time = time.monotonic()
            period = 1.0 / data_rate
            while time.monotonic() - start_time < duration:
                try:
                    data = get_var_callback()
                except Exception as ex:
                    print(str(ex))
                    print("Waiting 1 second before next data point")
                    time.sleep(1)
                    continue
                relative_time = time.monotonic() - start_time
                vals.append([relative_time] + data)
                time.sleep(period - (relative_time % period)) # this ensures consistently timed samples
            self.data = np.array(vals) # A lock is not really necessary due to the event
            print("Capture complete")
            achieved_data_rate = len(self.data) / self.data[-1, 0]
            if achieved_data_rate < (data_rate * 0.9):
                print("Achieved average data rate: {}Hz".format(achieved_data_rate))
                print("If this rate is significantly lower than what you specified, consider lowering it below the achieved value for more consistent sampling.")
            self.event.set() # tell the main thread that the bulk capture is complete
        Thread(target=loop, daemon=True).start()
    
    def plot(self):
        import matplotlib.pyplot as plt
        import inspect
        from textwrap import wrap
        plt.plot(self.data[:,0], self.data[:,1:])
        plt.xlabel("Time (seconds)")
        title = (str(inspect.getsource(self.get_var_callback))
                .strip("['\\n']")
                .split(" = ")[1])
        plt.title("\n".join(wrap(title, 60)))
        plt.legend(range(self.data.shape[1]-1))
        plt.show()


def step_and_plot(  axis,
                    step_size=100.0,
                    settle_time=0.5,
                    data_rate=500.0,
                    ctrl_mode=CONTROL_MODE_POSITION_CONTROL):
    
    if ctrl_mode is CONTROL_MODE_POSITION_CONTROL:
        get_var_callback = lambda :[axis.encoder.pos_estimate, axis.controller.pos_setpoint]
        initial_setpoint = axis.encoder.pos_estimate
        def set_setpoint(setpoint):
            axis.controller.pos_setpoint = setpoint
    elif ctrl_mode is CONTROL_MODE_VELOCITY_CONTROL:
        get_var_callback = lambda :[axis.encoder.vel_estimate, axis.controller.vel_setpoint]
        initial_setpoint = 0
        def set_setpoint(setpoint):
            axis.controller.vel_setpoint = setpoint
    else:
        print("Invalid control mode")
        return
    
    initial_settle_time = 0.5
    initial_control_mode = axis.controller.config.control_mode # Set it back afterwards
    print(initial_control_mode)
    axis.controller.config.control_mode = ctrl_mode
    axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
    
    capture = BulkCapture(get_var_callback,
                          data_rate=data_rate,
                          duration=initial_settle_time + settle_time)

    set_setpoint(initial_setpoint)
    time.sleep(initial_settle_time)
    set_setpoint(initial_setpoint + step_size) # relative/incremental movement

    capture.event.wait() # wait for Bulk Capture to be complete

    axis.requested_state = AXIS_STATE_IDLE
    axis.controller.config.control_mode = initial_control_mode
    capture.plot()

#ERG
def run_motor_characterize_input(odrv, axs, dir):
    """
    Runs configured test input for motor characterization; records time, voltage command, position, and velocity to a *.CSV in the provided directory.
    Note: must be set to gimbal motor mode and current control. Make sure current_limit is set appropriately,
    and be aware that this 'current limit' actually signifies the voltage limit when in gimbal motor mode.
    """

    from odrive.enums import AXIS_STATE_MOTOR_CHARACTERIZE_INPUT
    from datetime import datetime

    start_time = datetime.now()
    time_string = start_time.strftime("%m%d%Y_%H%M%S")
    file_name = dir + '\\motorData' + time_string + '.csv'
    if not os.path.isdir(dir):
        print("Error: invalid directory")
        return

    timeout = 30 # [s]
    buffer_size = odrv.get_motor_characterize_data_size()
    vals = []

    with open(file_name, "a+") as file:
        file.write('%Motor characterization data\n')
        file.write("%Each row's values were recorded on the same timestep\n")
        file.write("%Timestep increments at 8kHz\n\n")
        file.write('%Operator:\n')
        file.write('%Motor:\n')
        file.write('%ODrive axis: axis' + str(axs) + '\n')
        file.write('%Date:,' + start_time.strftime("%d/%m/%Y") + '\n')
        file.write('%Start time:,' + start_time.strftime("%H:%M:%S") + '\n\n')
        file.write('%timestep (8Hz),voltage,position,velocity\n')
        file.write('%[#],[V],[turns],[turns/s]\n')
        file.flush()

        print("Input starting...")
        if axs == 0 and odrv.axis0.motor.is_calibrated:
            odrv.axis0.requested_state = AXIS_STATE_MOTOR_CHARACTERIZE_INPUT
        elif axs == 1 and odrv.axis1.motor.is_calibrated:
            odrv.axis1.requested_state = AXIS_STATE_MOTOR_CHARACTERIZE_INPUT
        else:
            print("Error: invalid axis. Please choose either 0 or 1, and make sure motor is calibrated.")
            return

        finished = False
        finish_counter = 0
        while not finished:
            try:            
                idx = odrv.get_motor_characterize_data_idx()
                #ERG TODO - figure out why fetching data with idx==0 shifts all the data over a column, remove idx>0 condition
                #If desired, reducing data collected (e.g. to only timestep and velocity) could get lower latency
                if idx < buffer_size and idx > 0:
                    data = [odrv.get_motor_characterize_data_timestep(idx),
                            odrv.get_motor_characterize_data_voltage(idx),
                            odrv.get_motor_characterize_data_position(idx),
                            odrv.get_motor_characterize_data_velocity(idx)]
                else:
                    data = [float("NaN"), float("NaN"), float("NaN"), float("NaN")]
                    finish_counter += 1
            except Exception as ex:
                print(str(ex))
                time.sleep(1)
                continue
            
            #Record latest data (do not write yet, to optimize for speed)
            vals.append(data)
            
            #Check for end conditions (either time recorded is zero, or timeout)
            if data[0] < 1:
                finish_counter += 1
            else:
                finish_counter = 0

            if finish_counter > 10:
                finished = True

            elapsed = (datetime.now() - start_time).seconds
            if elapsed >= timeout:
                print("Timeout: took more than " + str(timeout) + " seconds")
                finished = True

            #When finished, write all recorded data
            if finished:
                print("Input finished. Recording data...")
                for line in vals:
                    str_data = map(str,line)
                    file.write(",".join(str_data) + ';\n')
                    file.flush()
                print("Data saved at: " + file_name)
    return

def print_drv_regs(name, motor):
    """
    Dumps the current gate driver regisers for the specified motor
    """
    fault = motor.gate_driver.drv_fault
    status_reg_1 = motor.gate_driver.status_reg_1
    status_reg_2 = motor.gate_driver.status_reg_2
    ctrl_reg_1 = motor.gate_driver.ctrl_reg_1
    ctrl_reg_2 = motor.gate_driver.ctrl_reg_2
    print(name + ": " + str(fault))
    print("DRV Fault Code: " + str(fault))
    print("Status Reg 1: " + str(status_reg_1) + " (" + format(status_reg_1, '#010b') + ")")
    print("Status Reg 2: " + str(status_reg_2) + " (" + format(status_reg_2, '#010b') + ")")
    print("Control Reg 1: " + str(ctrl_reg_1) + " (" + format(ctrl_reg_1, '#013b') + ")")
    print("Control Reg 2: " + str(ctrl_reg_2) + " (" + format(ctrl_reg_2, '#09b') + ")")

def show_oscilloscope(odrv):
    size = 18000
    values = []
    for i in range(size):
        values.append(odrv.get_oscilloscope_val(i))

    import matplotlib.pyplot as plt
    plt.plot(values)
    plt.show()

def rate_test(device):
    """
    Tests how many integers per second can be transmitted
    """

    # import matplotlib.pyplot as plt
    # plt.ion()

    print("reading 10000 values...")
    numFrames = 10000
    vals = []
    for _ in range(numFrames):
        vals.append(device.axis0.loop_counter)

    loopsPerFrame = (vals[-1] - vals[0])/numFrames
    loopsPerSec = (168000000/(6*3500))
    FramePerSec = loopsPerSec/loopsPerFrame
    print("Frames per second: " + str(FramePerSec))

    # plt.plot(vals)
    # plt.show(block=True)

def usb_burn_in_test(get_var_callback, cancellation_token):
    """
    Starts background threads that read a values form the USB device in a spin-loop
    """

    def fetch_data():
        global vals
        i = 0
        while not cancellation_token.is_set():
            try:
                get_var_callback()
                i += 1
            except Exception as ex:
                print(str(ex))
                time.sleep(1)
                i = 0
                continue
            if i % 1000 == 0:
                print("read {} values".format(i))
    threading.Thread(target=fetch_data, daemon=True).start()

def yes_no_prompt(question, default=None):
    if default is None:
        question += " [y/n] "
    elif default == True:
        question += " [Y/n] "
    elif default == False:
        question += " [y/N] "

    while True:
        print(question, end='')

        choice = input().lower()
        if choice in {'yes', 'y'}:
            return True
        elif choice in {'no', 'n'}:
            return False
        elif choice == '' and default is not None:
            return default
