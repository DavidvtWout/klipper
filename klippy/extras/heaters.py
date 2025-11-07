# Tracking of PWM controlled heaters and their temperature control
#
# Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import os
import threading
from typing import TYPE_CHECKING
import configparser

if TYPE_CHECKING:
    from heaters_scheduler import HeatersScheduler
    from ..gcode import GCodeCommand
    from ..klippy import Printer
    from ..reactor import Reactor


######################################################################
# Heater
######################################################################

KELVIN_TO_CELSIUS = -273.15
MAX_HEAT_TIME = 3.0
AMBIENT_TEMP = 25.
PID_PARAM_BASE = 255.
MAX_MAINTHREAD_TIME = 5.0
QUELL_STALE_TIME = 7.0

class Heater:
    def __init__(self, config, sensor):
        self.printer: 'Printer' = config.get_printer()
        self.reactor: 'Reactor' = self.printer.get_reactor()
        self.name = config.get_name()
        self.short_name = self.name.split()[-1]

        self.pwm_cycle_time = config.getfloat('pwm_cycle_time', 0.100, above=0., maxval=MAX_HEAT_TIME)
        self.max_power = config.getfloat('max_power', 1., above=0., maxval=1.)

        # Setup sensor
        self.sensor = sensor
        self.min_temp = config.getfloat('min_temp', minval=KELVIN_TO_CELSIUS)
        self.max_temp = config.getfloat('max_temp', above=self.min_temp)
        self.sensor.setup_minmax(self.min_temp, self.max_temp)
        self.sensor.setup_callback(self.temperature_callback)
        self.pwm_delay = self.sensor.get_report_time_delta()
        self._pwm_off_measure = config.getboolean('pwm_off_measure', default=False)
        if (self._pwm_off_measure and
                self.pwm_cycle_time * (1 - self.max_power) < self.sensor.get_report_time_delta() * 4):
            raise configparser.Error('If pwm_off_measure is enabled, the max_power must be reduced '
                                     'to allow enough time to measure sensors while heating elements are off.')

        # Setup temperature checks
        self.min_extrude_temp = config.getfloat(
            'min_extrude_temp', 170.,
            minval=self.min_temp, maxval=self.max_temp)
        is_fileoutput = (self.printer.get_start_args().get('debugoutput')
                         is not None)
        self.can_extrude = self.min_extrude_temp <= 0. or is_fileoutput
        self.smooth_time = config.getfloat('smooth_time', 1., above=0.)
        self.inv_smooth_time = 1. / self.smooth_time
        self.verify_mainthread_time = -999.
        self.lock = threading.Lock()
        self.last_temp = self.smoothed_temp = self.target_temp = 0.
        self._temperature_readings = dict()
        self.last_temp_time = 0.
        # pwm caching
        self.next_pwm_time = 0.
        self.next_pwm_value = 0.

        # Setup control algorithm sub-class
        algos = {'watermark': ControlBangBang, 'pid': ControlPID}
        algo = config.getchoice('control', algos)
        self.control = algo(self, config)
        self._pwm_override_value = -1.

        # Setup output heater pin
        heater_pin = config.get('heater_pin')
        ppins = self.printer.lookup_object('pins')
        self.mcu_pwm = ppins.setup_pin('pwm', heater_pin)
        self.mcu_pwm.setup_cycle_time(self.pwm_cycle_time)
        self.mcu_pwm.setup_max_duration(MAX_HEAT_TIME)
        self._pwm_timer = self.reactor.register_timer(self._pwm_timer_callback)
        # self.reactor.update_timer(self._pwm_timer, self.pwm_cycle_time)

        # Load additional modules
        self.printer.load_object(config, f'verify_heater {self.short_name}')
        self.printer.load_object(config, 'pid_calibrate')

        # Register commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command(
            'SET_HEATER_TEMPERATURE', 'HEATER', self.short_name,
            self.cmd_SET_HEATER_TEMPERATURE, desc='Sets a heater temperature'
        )
        gcode.register_mux_command(
            'HEATER_OVERRIDE', 'HEATER', self.short_name,
            self.cmd_HEATER_OVERRIDE, desc='Override the PWM value for a heater')
        gcode.register_mux_command(
            'HEATER_OVERRIDE_RELEASE', 'HEATER', self.short_name,
            self.cmd_HEATER_OVERRIDE_RELEASE, desc='Release PWM override for a heater')

        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.printer.register_event_handler("klippy:shutdown", self._handle_shutdown)

    def _handle_connect(self):
        self.reactor.update_timer(self._pwm_timer, self.reactor.NOW)

    def override_pwm_value(self, value):
        with self.lock:
            self.target_temp = 0.
            self._pwm_override_value = value

    def _pwm_timer_callback(self, eventtime: float):
        print_time = self.mcu_pwm.get_mcu().estimated_print_time(eventtime)
        with self.lock:
            self.next_pwm_time = print_time + self.pwm_delay + 0.1
            self.next_pwm_value = self.control.temperature_update(print_time, self.smoothed_temp, self.target_temp)
            if self.is_overridden():
                self.next_pwm_value = self._pwm_override_value
            if print_time > self.verify_mainthread_time:
                self.next_pwm_value = 0.
            self.mcu_pwm.set_pwm(self.next_pwm_time, self.next_pwm_value)

        return eventtime + self.pwm_cycle_time

    def temperature_callback(self, read_time, temp):
        with self.lock:
            report_time = self.sensor.get_report_time_delta()
            if self.next_pwm_time - report_time < read_time < self.next_pwm_time + self.pwm_cycle_time * self.next_pwm_value + 2 * report_time:
                return

            time_diff = read_time - self.last_temp_time
            self.last_temp = temp
            self.last_temp_time = read_time

            temp_diff = temp - self.smoothed_temp
            adj_time = min(time_diff * self.inv_smooth_time, 1.)
            self.smoothed_temp += temp_diff * adj_time
            self.can_extrude = (self.smoothed_temp >= self.min_extrude_temp)

    def _handle_shutdown(self):
        self.verify_mainthread_time = -999.
    # External commands
    def get_name(self):
        return self.name
    def get_pwm_delay(self):
        return self.pwm_delay
    def get_max_power(self):
        return self.max_power
    def get_smooth_time(self):
        return self.smooth_time
    def set_temp(self, degrees):
        if degrees and (degrees < self.min_temp or degrees > self.max_temp):
            raise self.printer.command_error(
                "Requested temperature (%.1f) out of range (%.1f:%.1f)"
                % (degrees, self.min_temp, self.max_temp))
        with self.lock:
            self._pwm_override_value = -1.
            self.target_temp = degrees

    def get_temp(self, eventtime):
        est_print_time = self.mcu_pwm.get_mcu().estimated_print_time(eventtime)
        quell_time = est_print_time - QUELL_STALE_TIME
        with self.lock:
            if self.last_temp_time < quell_time:
                return 0., self.target_temp
            return self.smoothed_temp, self.target_temp

    def is_overridden(self) -> bool:
        return self._pwm_override_value >= 0.

    def check_busy(self, eventtime):
        with self.lock:
            return self.is_overridden() or self.control.check_busy(
                eventtime, self.smoothed_temp, self.target_temp)

    def set_control(self, control):
        with self.lock:
            old_control = self.control
            self.control = control
            self.target_temp = 0.
        return old_control

    def alter_target(self, target_temp):
        if target_temp:
            target_temp = max(self.min_temp, min(self.max_temp, target_temp))
        self.target_temp = target_temp

    def stats(self, eventtime):
        est_print_time = self.mcu_pwm.get_mcu().estimated_print_time(eventtime)
        if not self.printer.is_shutdown():
            self.verify_mainthread_time = est_print_time + MAX_MAINTHREAD_TIME
        with self.lock:
            target_temp = self.target_temp
            last_temp = self.last_temp
            pwm_value = self.next_pwm_value
        is_active = target_temp or last_temp > 50.
        return is_active, '%s: target=%.0f temp=%.1f pwm=%.3f' % (
            self.short_name, target_temp, last_temp, pwm_value)

    def get_status(self, eventtime):
        with self.lock:
            target_temp = self.target_temp
            smoothed_temp = self.smoothed_temp
            pwm_value = self.next_pwm_value
        return {'temperature': round(smoothed_temp, 2), 'target': target_temp, 'power': pwm_value}

    def cmd_SET_HEATER_TEMPERATURE(self, gcmd: 'GCodeCommand'):
        temp = gcmd.get_float('TARGET', 0.)
        pheaters: 'PrinterHeaters' = self.printer.lookup_object('heaters')
        pheaters.set_temperature(self, temp)

    def cmd_HEATER_OVERRIDE(self, gcmd: 'GCodeCommand'):
        pwm_value = gcmd.get_float('PWM')
        self.override_pwm_value(pwm_value)

    def cmd_HEATER_OVERRIDE_RELEASE(self, _: 'GCodeCommand'):
        self.override_pwm_value(-1.)


######################################################################
# Bang-bang control algo
######################################################################

class ControlBangBang:
    def __init__(self, heater, config):
        self.heater = heater
        self.heater_max_power = heater.get_max_power()
        self.max_delta = config.getfloat('max_delta', 2.0, above=0.)
        self.heating = False

    def temperature_update(self, read_time, temp, target_temp) -> float:
        if self.heating and temp >= target_temp+self.max_delta:
            self.heating = False
        elif not self.heating and temp <= target_temp-self.max_delta:
            self.heating = True
        if self.heating:
            return self.heater_max_power
        else:
            return 0

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        return smoothed_temp < target_temp-self.max_delta


######################################################################
# Proportional Integral Derivative (PID) control algo
######################################################################

PID_SETTLE_DELTA = 1.
PID_SETTLE_SLOPE = .1

class ControlPID:
    def __init__(self, heater, config):
        self.heater = heater
        self.heater_max_power = heater.get_max_power()
        self.Kp = config.getfloat('pid_Kp') / PID_PARAM_BASE
        self.Ki = config.getfloat('pid_Ki') / PID_PARAM_BASE
        self.Kd = config.getfloat('pid_Kd') / PID_PARAM_BASE
        self.min_deriv_time = heater.get_smooth_time()
        self.temp_integ_max = 0.
        if self.Ki:
            self.temp_integ_max = self.heater_max_power / self.Ki
        self.prev_temp = AMBIENT_TEMP
        self.prev_temp_time = 0.
        self.prev_temp_deriv = 0.
        self.prev_temp_integ = 0.
        self._rate = 0.

    def set_rate(self, rate):
        self._rate = rate / 3600

    def temperature_update(self, read_time, temp, target_temp) -> float:
        time_diff = read_time - self.prev_temp_time

        # Calculate change of temperature (D)
        temp_diff = temp - self.prev_temp
        if time_diff >= self.min_deriv_time:
            temp_deriv = temp_diff / time_diff - self._rate
        else:
            temp_deriv = ((self.prev_temp_deriv * (self.min_deriv_time - time_diff)  + temp_diff) /
                          self.min_deriv_time - self._rate)

        # Calculate accumulated temperature "error" (K and I)
        temp_err = target_temp - temp
        temp_integ = self.prev_temp_integ + temp_err * time_diff
        temp_integ = max(0., min(self.temp_integ_max, temp_integ))

        # Calculate output
        co = self.Kp*temp_err + self.Ki*temp_integ - self.Kd*temp_deriv
        bounded_co = max(0., min(self.heater_max_power, co))

        # Store state for next measurement
        self.prev_temp = temp
        self.prev_temp_time = read_time
        self.prev_temp_deriv = temp_deriv
        if co == bounded_co:
            self.prev_temp_integ = temp_integ

        return bounded_co

    def check_busy(self, eventtime, smoothed_temp, target_temp):
        temp_diff = target_temp - smoothed_temp
        return (abs(temp_diff) > PID_SETTLE_DELTA
                or abs(self.prev_temp_deriv) > PID_SETTLE_SLOPE)


######################################################################
# Sensor and heater lookup
######################################################################

class PrinterHeaters:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.sensor_factories = {}
        self.heaters = {}
        self.gcode_id_to_sensor = {}
        self.available_heaters = []
        self.available_sensors = []
        self.available_monitors = []
        self.has_started = self.have_load_sensors = False
        self.printer.register_event_handler("klippy:ready", self._handle_ready)
        self.printer.register_event_handler("gcode:request_restart",
                                            self.turn_off_all_heaters)
        # Register commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("TURN_OFF_HEATERS", self.cmd_TURN_OFF_HEATERS,
                               desc=self.cmd_TURN_OFF_HEATERS_help)
        gcode.register_command("M105", self.cmd_M105, when_not_ready=True)
        gcode.register_command("TEMPERATURE_WAIT", self.cmd_TEMPERATURE_WAIT,
                               desc=self.cmd_TEMPERATURE_WAIT_help)

        self.scheduler: 'HeatersScheduler' = self.printer.load_object(config, 'heaters_scheduler')

    def load_config(self, config):
        self.have_load_sensors = True
        # Load default temperature sensors
        pconfig = self.printer.lookup_object('configfile')
        dir_name = os.path.dirname(__file__)
        filename = os.path.join(dir_name, 'temperature_sensors.cfg')
        try:
            dconfig = pconfig.read_config(filename)
        except Exception:
            logging.exception("Unable to load temperature_sensors.cfg")
            raise config.error("Cannot load config '%s'" % (filename,))
        for c in dconfig.get_prefix_sections(''):
            self.printer.load_object(dconfig, c.get_name())
    def add_sensor_factory(self, sensor_type, sensor_factory):
        self.sensor_factories[sensor_type] = sensor_factory
    def setup_heater(self, config, gcode_id=None):
        heater_name = config.get_name().split()[-1]
        if heater_name in self.heaters:
            raise config.error("Heater %s already registered" % (heater_name,))
        # Setup sensor
        sensor = self.setup_sensor(config)
        # Create heater
        self.heaters[heater_name] = heater = Heater(config, sensor)
        self.register_sensor(config, heater, gcode_id)
        self.available_heaters.append(config.get_name())
        return heater
    def get_all_heaters(self):
        return self.available_heaters
    def lookup_heater(self, heater_name):
        if heater_name not in self.heaters:
            raise self.printer.config_error(
                "Unknown heater '%s'" % (heater_name,))
        return self.heaters[heater_name]
    def setup_sensor(self, config):
        if not self.have_load_sensors:
            self.load_config(config)
        sensor_type = config.get('sensor_type')
        if sensor_type not in self.sensor_factories:
            raise self.printer.config_error(
                "Unknown temperature sensor '%s'" % (sensor_type,))
        return self.sensor_factories[sensor_type](config)
    def register_sensor(self, config, psensor, gcode_id=None):
        self.available_sensors.append(config.get_name())
        if gcode_id is None:
            gcode_id = config.get('gcode_id', None)
            if gcode_id is None:
                return
        if gcode_id in self.gcode_id_to_sensor:
            raise self.printer.config_error(
                "G-Code sensor id %s already registered" % (gcode_id,))
        self.gcode_id_to_sensor[gcode_id] = psensor
    def register_monitor(self, config):
        self.available_monitors.append(config.get_name())
    def get_status(self, eventtime):
        return {'available_heaters': self.available_heaters,
                'available_sensors': self.available_sensors,
                'available_monitors': self.available_monitors}
    def turn_off_all_heaters(self, print_time=0.):
        for heater in self.heaters.values():
            heater.set_temp(0.)
    cmd_TURN_OFF_HEATERS_help = "Turn off all heaters"
    def cmd_TURN_OFF_HEATERS(self, gcmd):
        self.turn_off_all_heaters()
    # G-Code M105 temperature reporting
    def _handle_ready(self):
        self.has_started = True
    def _get_temp(self, eventtime):
        # Tn:XXX /YYY B:XXX /YYY
        out = []
        if self.has_started:
            for gcode_id, sensor in sorted(self.gcode_id_to_sensor.items()):
                cur, target = sensor.get_temp(eventtime)
                out.append("%s:%.1f /%.1f" % (gcode_id, cur, target))
        if not out:
            return "T:0"
        return " ".join(out)
    def cmd_M105(self, gcmd):
        # Get Extruder Temperature
        reactor = self.printer.get_reactor()
        msg = self._get_temp(reactor.monotonic())
        did_ack = gcmd.ack(msg)
        if not did_ack:
            gcmd.respond_raw(msg)
    def _wait_for_temperature(self, heater):
        # Helper to wait on heater.check_busy() and report M105 temperatures
        if self.printer.get_start_args().get('debugoutput') is not None:
            return
        toolhead = self.printer.lookup_object("toolhead")
        gcode = self.printer.lookup_object("gcode")
        reactor = self.printer.get_reactor()
        eventtime = reactor.monotonic()
        while not self.printer.is_shutdown() and heater.check_busy(eventtime):
            print_time = toolhead.get_last_move_time()
            gcode.respond_raw(self._get_temp(eventtime))
            eventtime = reactor.pause(eventtime + 1.)
    def set_temperature(self, heater, temp, wait=False):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_lookahead_callback((lambda pt: None))
        if heater in self.scheduler.heaters:
            self.scheduler.pause()
        heater.set_temp(temp)
        if wait and temp:
            self._wait_for_temperature(heater)
    cmd_TEMPERATURE_WAIT_help = "Wait for a temperature on a sensor"
    def cmd_TEMPERATURE_WAIT(self, gcmd):
        sensor_name = gcmd.get('SENSOR')
        if sensor_name not in self.available_sensors:
            raise gcmd.error("Unknown sensor '%s'" % (sensor_name,))
        min_temp = gcmd.get_float('MINIMUM', float('-inf'))
        max_temp = gcmd.get_float('MAXIMUM', float('inf'), above=min_temp)
        if min_temp == float('-inf') and max_temp == float('inf'):
            raise gcmd.error(
                "Error on 'TEMPERATURE_WAIT': missing MINIMUM or MAXIMUM.")
        if self.printer.get_start_args().get('debugoutput') is not None:
            return
        if sensor_name in self.heaters:
            sensor = self.heaters[sensor_name]
        else:
            sensor = self.printer.lookup_object(sensor_name)
        toolhead = self.printer.lookup_object("toolhead")
        reactor = self.printer.get_reactor()
        eventtime = reactor.monotonic()
        while not self.printer.is_shutdown():
            temp, target = sensor.get_temp(eventtime)
            if temp >= min_temp and temp <= max_temp:
                return
            print_time = toolhead.get_last_move_time()
            gcmd.respond_raw(self._get_temp(eventtime))
            eventtime = reactor.pause(eventtime + 1.)

def load_config(config):
    return PrinterHeaters(config)
