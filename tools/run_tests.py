#!/bin/env python3
#
# This script tests various functions of the ODrive firmware and
# the ODrive Python library.
#
# Usage:
# 1. adapt test-rig.yaml for your test rig.
# 2. ./run_tests.py

import yaml
import os
import sys
import threading
import traceback
import argparse
from odrive.tests import *
from odrive.utils import Logger, for_all_parallel, Event

script_path=os.path.dirname(os.path.realpath(__file__))

parser = argparse.ArgumentParser(description='ODrive automated test tool\n')
parser.add_argument("--skip-boring-tests", action="store_true",
                    help="Skip the boring tests and go right to the high power tests")
parser.add_argument("--ignore", metavar='DEVICE', action='store', nargs='+',
                    help="Ignore one or more ODrives or axes")
parser.add_argument("--test-rig-yaml", type=argparse.FileType('r'),
                    help="test rig YAML file")
parser.set_defaults(test_rig_yaml=script_path + '/test-rig.yaml')
parser.set_defaults(ignore=[])
args = parser.parse_args()

all_tests = []
if not args.skip_boring_tests:
    all_tests.append(TestFlashAndErase())
    all_tests.append(TestSetup())
    all_tests.append(TestMotorCalibration())
    #    # TODO: test encoder index search
    all_tests.append(TestEncoderOffsetCalibration())
    #    # TODO: hold down one motor while the other one does an index search (should fail)
    all_tests.append(TestClosedLoopControl())
    all_tests.append(TestStoreAndReboot())
    all_tests.append(TestEncoderOffsetCalibration()) # need to find offset _or_ index after reboot
    all_tests.append(TestClosedLoopControl())
else:
    all_tests.append(TestDiscoverAndGotoIdle())
    all_tests.append(TestEncoderOffsetCalibration(pass_if_ready=True))

#all_tests.append(TestHighVelocity())
all_tests.append(TestHighVelocityInViscousFluid(load_current=20, driver_current=40))
#all_tests.append(TestVelCtrlVsPosCtrl())
# TODO: test step/dir
# TODO: test sensorless
# TODO: test ASCII protocol
# TODO: test protocol over UART

print(str(args.ignore))
logger = Logger()


test_rig_yaml = yaml.load(args.test_rig_yaml)
os.chdir(script_path + '/../Firmware')

# Build a dictionary of odrive test contexts by name
odrives_by_name = {}
for odrv_idx, odrv_yaml in enumerate(test_rig_yaml['odrives']):
    name = odrv_yaml['name'] if 'name' in odrv_yaml else 'odrive{}'.format(odrv_idx)
    if not name in args.ignore:
        odrives_by_name[name] = ODriveTestContext(name, odrv_yaml)

# Build a dictionary of axis test contexts by name (e.g. odrive0.axis0)
axes_by_name = {}
for odrv_ctx in odrives_by_name.values():
    for axis_idx, axis_ctx in enumerate(odrv_ctx.axes):
        if not axis_ctx.name in args.ignore:
            axes_by_name[axis_ctx.name] = axis_ctx

# Ensure mechanical couplings are valid
couplings = []
if test_rig_yaml['couplings'] is None:
    test_rig_yaml['couplings'] = {}
else:
    for coupling in test_rig_yaml['couplings']:
        c = [axes_by_name[axis_name] for axis_name in coupling if (axis_name in axes_by_name)]
        if len(c) > 1:
            couplings.append(c)

app_shutdown_token = Event()

try:
    for test in all_tests:
        if isinstance(test, ODriveTest):
            def odrv_test_thread(odrv_name):
                odrv_ctx = odrives_by_name[odrv_name]
                logger.info('● running {} on {}...'.format(type(test).__name__, odrv_name))
                try:
                    test.check_preconditions(odrv_ctx,
                              logger.indent('  {}: '.format(odrv_name)))
                except:
                    raise PreconditionsNotMet()
                test.run_test(odrv_ctx,
                              logger.indent('  {}: '.format(odrv_name)))

            if test._exclusive:
                for odrv in odrives_by_name:
                    odrv_test_thread(odrv)
            else:
                for_all_parallel(odrives_by_name, lambda x: type(test).__name__ + " on " + x, odrv_test_thread)

        elif isinstance(test, AxisTest):
            def axis_test_thread(axis_name):
                # Get all axes that are mechanically coupled with the axis specified by axis_name
                conflicting_axes = sum([c for c in couplings if (axis_name in [a.name for a in c])], [])
                # Remove duplicates
                conflicting_axes = list(set(conflicting_axes))
                # Acquire lock for all conflicting axes
                conflicting_axes.sort(key=lambda x: x.name) # prevent deadlocks
                axis_ctx = axes_by_name[axis_name]
                for conflicting_axis in conflicting_axes:
                    conflicting_axis.lock.acquire()
                try:
                    if not app_shutdown_token.is_set():
                        # Run test on this axis
                        logger.info('● running {} on {}...'.format(type(test).__name__, axis_name))
                        try:
                            test.check_preconditions(axis_ctx,
                                        logger.indent('  {}: '.format(axis_name)))
                        except:
                            raise PreconditionsNotMet()
                        test.run_test(axis_ctx,
                                    logger.indent('  {}: '.format(axis_name)))
                    else:
                        logger.warn('⬛ skipping {} on {}'.format(type(test).__name__, axis_name))
                except:
                    app_shutdown_token.set()
                    raise
                finally:
                    # Release all conflicting axes
                    for conflicting_axis in conflicting_axes:
                        conflicting_axis.lock.release()

            for_all_parallel(axes_by_name, lambda x: type(test).__name__ + " on " + x, axis_test_thread)

        elif isinstance(test, DualAxisTest):
            def dual_axis_test_thread(coupling):
                coupling_name = "...".join([a.name for a in coupling])
                # Remove duplicates
                coupled_axes = list(set(coupling))
                # Acquire lock for all conflicting axes
                coupled_axes.sort(key=lambda x: x.name) # prevent deadlocks
                for axis_ctx in coupled_axes:
                    axis_ctx.lock.acquire()
                try:
                    if not app_shutdown_token.is_set():
                        # Run test on this axis
                        logger.info('● running {} on {}...'.format(type(test).__name__, coupling_name))
                        try:
                            test.check_preconditions(coupled_axes[0], coupled_axes[1],
                                        logger.indent('  {}: '.format(coupling_name)))
                        except:
                            raise PreconditionsNotMet()
                        test.run_test(coupled_axes[0], coupled_axes[1],
                                    logger.indent('  {}: '.format(coupling_name)))
                    else:
                        logger.warn('⬛ skipping {} on {}...'.format(type(test).__name__, coupling_name))
                except:
                    app_shutdown_token.set()
                    raise
                finally:
                    # Release all conflicting axes
                    for axis_ctx in coupled_axes:
                        axis_ctx.lock.release()

            for_all_parallel(couplings, lambda x: type(test).__name__ + " on " + "..".join([a.name for a in x]), dual_axis_test_thread)

        else:
            logger.warn("ignoring unknown test type {}".format(type(test)))

except:
    logger.error(traceback.format_exc())
    logger.debug('=> Test failed. Please wait while I secure the test rig...')
    try:
        dont_secure_after_failure = False # TODO: disable
        if not dont_secure_after_failure:
            def odrv_reset_thread(odrv_name):
                odrv_ctx = odrives_by_name[odrv_name]
                #run("make erase PROGRAMMER='" + odrv_ctx.yaml['programmer'] + "'", logger, timeout=30)
                odrv_ctx.handle.axis0.requested_state = AXIS_STATE_IDLE
                odrv_ctx.handle.axis1.requested_state = AXIS_STATE_IDLE
            for_all_parallel(odrives_by_name, lambda x: x['name'], odrv_reset_thread)
    except:
        logger.error('///////////////////////////////////////////')
        logger.error('/// CRITICAL: COULD NOT SECURE TEST RIG ///')
        logger.error('///     CUT THE POWER IMMEDIATELY!      ///')
        logger.error('///////////////////////////////////////////')
    else:
        logger.error('some test failed!')
else:
    logger.success('All tests succeeded!')
