# Tracking of PWM controlled heaters and their temperature control
#
# Copyright (C) 2016-2025  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import os, logging, threading
import configparser
import ast
import operator

######################################################################
# Heater
######################################################################

KELVIN_TO_CELSIUS = -273.15
MAX_HEAT_TIME = 3.0
AMBIENT_TEMP = 25.
PID_PARAM_BASE = 255.
MAX_MAINTHREAD_TIME = 5.0

class Heater:
    def __init__(self, config, sensor):
        self.printer = config.get_printer()
        self.name = config.get_name()
        self.short_name = short_name = self.name.split()[-1]
        # Setup sensor
        self.sensor = sensor
        self.min_temp = config.getfloat('min_temp', minval=KELVIN_TO_CELSIUS)
        self.max_temp = config.getfloat('max_temp', above=self.min_temp)
        self.sensor.setup_minmax(self.min_temp, self.max_temp)
        self.sensor.setup_callback(self.temperature_callback)
        self.pwm_delay = self.sensor.get_report_time_delta()
        # Setup temperature checks
        self.min_extrude_temp = config.getfloat(
            'min_extrude_temp', 170.,
            minval=self.min_temp, maxval=self.max_temp)
        is_fileoutput = (self.printer.get_start_args().get('debugoutput')
                         is not None)
        self.can_extrude = self.min_extrude_temp <= 0. or is_fileoutput

        self._max_power = self._max_power_formula = None
        max_power = config.get('max_power', 1.)
        if isinstance(max_power, int) or isinstance(max_power, float):
            if not 0 <= max_power <= 1:
                raise configparser.Error(f"Option 'max_power' in section '{config.section}' must be between 0 and 1")
            self.max_power = max_power
        else:
            self._max_power_formula = ast.parse(max_power, mode="eval")
            try:
                self._eval_max_power(temperature=AMBIENT_TEMP)
            except Exception as e:
                raise configparser.Error(f"Option 'max_power' in section '{config.section}' is not a valid formula: {e}")

        self.smooth_time = config.getfloat('smooth_time', 1., above=0.)
        self.inv_smooth_time = 1. / self.smooth_time
        self.verify_mainthread_time = -999.
        self.lock = threading.Lock()
        self.last_temp = self.smoothed_temp = self.target_temp = 0.
        self.last_temp_time = 0.
        # Heat at controlled rate.
        self.ramp_rate = 0.  # In degrees per hour
        self.ramp_start_time = self.ramp_start_temp = self.ramp_target = 0.
        # pwm caching
        self.next_pwm_time = 0.
        self.last_pwm_value = 0.
        # Setup control algorithm sub-class
        algos = {'watermark': ControlBangBang, 'pid': ControlPID}
        algo = config.getchoice('control', algos)
        self.control = algo(self, config)
        # Setup output heater pin
        heater_pin = config.get('heater_pin')
        ppins = self.printer.lookup_object('pins')
        self.mcu_pwm = ppins.setup_pin('pwm', heater_pin)
        pwm_cycle_time = config.getfloat('pwm_cycle_time', 0.100, above=0.,
                                         maxval=self.pwm_delay)
        self.mcu_pwm.setup_cycle_time(pwm_cycle_time)
        self.mcu_pwm.setup_max_duration(MAX_HEAT_TIME)
        # Load additional modules
        self.printer.load_object(config, "verify_heater %s" % (short_name,))
        self.printer.load_object(config, "pid_calibrate")
        self.printer.register_event_handler("klippy:shutdown",
                                            self._handle_shutdown)
    def set_pwm(self, read_time, value):
        if self.target_temp <= 0. or read_time > self.verify_mainthread_time:
            value = 0.
        if ((read_time < self.next_pwm_time or not self.last_pwm_value)
            and abs(value - self.last_pwm_value) < 0.05):
            # No significant change in value - can suppress update
            return
        pwm_time = read_time + self.pwm_delay
        self.next_pwm_time = pwm_time + 0.75 * MAX_HEAT_TIME
        self.last_pwm_value = value
        self.mcu_pwm.set_pwm(pwm_time, value)
        #logging.debug("%s: pwm=%.3f@%.3f (from %.3f@%.3f [%.3f])",
        #              self.name, value, pwm_time,
        #              self.last_temp, self.last_temp_time, self.target_temp)
    def temperature_callback(self, read_time, temp):
        with self.lock:
            time_diff = read_time - self.last_temp_time
            self.last_temp = temp
            self.last_temp_time = read_time
            if self.ramp_target:
                ramp_hours = (read_time - self.ramp_start_time) / 3600
                min_or_max = min if self.ramp_target > self.target_temp else max
                self.target_temp = min_or_max(
                    self.ramp_start_temp + self.ramp_rate * ramp_hours,
                    self.ramp_target
                )
                if self.target_temp == self.ramp_target:
                    self.ramp_rate = self.ramp_target = self.ramp_start_time = self.ramp_start_temp = 0.
            self.control.temperature_update(read_time, temp, self.target_temp, rate=self.ramp_rate)
            temp_diff = temp - self.smoothed_temp
            adj_time = min(time_diff * self.inv_smooth_time, 1.)
            self.smoothed_temp += temp_diff * adj_time
            self.can_extrude = (self.smoothed_temp >= self.min_extrude_temp)
        #logging.debug("temp: %.3f %f = %f", read_time, temp)
    def _handle_shutdown(self):
        self.verify_mainthread_time = -999.
    # External commands
    def get_name(self):
        return self.name
    def get_pwm_delay(self):
        return self.pwm_delay

    def _eval_max_power(self, temperature):
        allowed_ops = {
            ast.Add: operator.add,
            ast.Sub: operator.sub,
            ast.Mult: operator.mul,
            ast.Div: operator.truediv,
            ast.Pow: operator.pow,
            ast.USub: operator.neg,
            ast.Call: None,  # For min, max (we'll handle specially)
        }
        allowed_funcs = {
            "min": min,
            "max": max,
        }
        variables = {"T": temperature}

        def _eval(node):
            if isinstance(node, ast.Num):  # constant
                return node.n
            elif isinstance(node, ast.Name):
                return variables[node.id]
            elif isinstance(node, ast.BinOp):
                return allowed_ops[type(node.op)](_eval(node.left), _eval(node.right))
            elif isinstance(node, ast.UnaryOp):
                return allowed_ops[type(node.op)](_eval(node.operand))
            elif isinstance(node, ast.Call):
                if not isinstance(node.func, ast.Name) or node.func.id not in allowed_funcs:
                    raise ValueError("Unsupported function")
                func = allowed_funcs[node.func.id]
                args = [_eval(arg) for arg in node.args]
                return func(*args)
            else:
                raise TypeError(f"Unsupported expression: {ast.dump(node)}")

        return _eval(self._max_power_formula.body)

    def get_max_power(self, eventtime):
        if self._max_power_formula:
            temp, _ = self.get_temp(eventtime)
            return self._eval_max_power(temp)
        else:
            return self._max_power

    def get_smooth_time(self):
        return self.smooth_time
    def set_temp(self, degrees, rate=0.):
        if degrees and (degrees < self.min_temp or degrees > self.max_temp):
            raise self.printer.command_error(
                "Requested temperature (%.1f) out of range (%.1f:%.1f)"
                % (degrees, self.min_temp, self.max_temp))
        with self.lock:
            if rate:
                if self.ramp_target:
                    self.ramp_start_temp = self.target_temp
                elif abs(self.target_temp - self.smoothed_temp) < 5:
                    self.ramp_start_temp = self.target_temp
                else:
                    self.ramp_start_temp = self.smoothed_temp
                self.ramp_start_time = self.last_temp_time
                self.target_temp = self.ramp_start_temp
                self.ramp_target = degrees
            else:
                self.target_temp = degrees
                self.ramp_target = 0.
            self.ramp_rate = rate
    def get_temp(self, eventtime):
        print_time = self.mcu_pwm.get_mcu().estimated_print_time(eventtime) - 5.
        with self.lock:
            if self.last_temp_time < print_time:
                return 0., self.target_temp
            return self.smoothed_temp, self.target_temp
    def get_ramp_target(self):
        with self.lock:
            return self.ramp_target
    def check_busy(self, eventtime):
        with self.lock:
            return self.control.check_busy(
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
            last_pwm_value = self.last_pwm_value
        is_active = target_temp or last_temp > 50.
        return is_active, '%s: target=%.0f temp=%.1f pwm=%.3f' % (
            self.short_name, target_temp, last_temp, last_pwm_value)
    def get_status(self, eventtime):
        with self.lock:
            target_temp = self.target_temp
            smoothed_temp = self.smoothed_temp
            last_pwm_value = self.last_pwm_value
        return {'temperature': round(smoothed_temp, 2), 'target': target_temp,
                'power': last_pwm_value}


######################################################################
# Bang-bang control algo
######################################################################

class ControlBangBang:
    def __init__(self, heater, config):
        self.heater = heater
        self.max_delta = config.getfloat('max_delta', 2.0, above=0.)
        self.heating = False
    def temperature_update(self, read_time, temp, target_temp, rate=None):
        if self.heating and temp >= target_temp+self.max_delta:
            self.heating = False
        elif not self.heating and temp <= target_temp-self.max_delta:
            self.heating = True
        if self.heating:
            self.heater.set_pwm(read_time, self.heater.get_max_power(read_time))
        else:
            self.heater.set_pwm(read_time, 0.)
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
        self.Kp = config.getfloat('pid_Kp') / PID_PARAM_BASE
        self.Ki = config.getfloat('pid_Ki') / PID_PARAM_BASE
        self.Kd = config.getfloat('pid_Kd') / PID_PARAM_BASE
        self.min_deriv_time = heater.get_smooth_time()
        self.prev_temp = AMBIENT_TEMP
        self.prev_temp_time = 0.
        self.prev_temp_deriv = 0.
        self.prev_temp_integ = 0.

    def temperature_update(self, read_time, temp, target_temp, rate=None):
        max_power = self.heater.get_max_power(read_time)
        time_diff = read_time - self.prev_temp_time
        # Calculate change of temperature
        temp_diff = temp - self.prev_temp
        if time_diff >= self.min_deriv_time:
            temp_deriv = temp_diff / time_diff
        else:
            temp_deriv = (self.prev_temp_deriv * (self.min_deriv_time-time_diff)
                          + temp_diff) / self.min_deriv_time
        if rate:
            temp_deriv -= rate / 3600
        # Calculate accumulated temperature "error"
        temp_err = target_temp - temp
        temp_integ_max = 0.
        if self.Ki:
            temp_integ_max = max_power / self.Ki
        temp_integ = self.prev_temp_integ + temp_err * time_diff
        temp_integ = max(0., min(temp_integ_max, temp_integ))
        # Calculate output
        co = self.Kp*temp_err + self.Ki*temp_integ - self.Kd*temp_deriv
        #logging.debug("pid: %f@%.3f -> diff=%f deriv=%f err=%f integ=%f co=%d",
        #    temp, read_time, temp_diff, temp_deriv, temp_err, temp_integ, co)
        bounded_co = max(0., min(max_power, co))
        self.heater.set_pwm(read_time, bounded_co)
        # Store state for next measurement
        self.prev_temp = temp
        self.prev_temp_time = read_time
        self.prev_temp_deriv = temp_deriv
        if co == bounded_co:
            self.prev_temp_integ = temp_integ
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
        gcode.register_command("SET_HEATER_TEMPERATURE", self.cmd_SET_HEATER_TEMPERATURE,
                                   desc="Sets a heater temperature")
        gcode.register_command("TURN_OFF_HEATERS", self.cmd_TURN_OFF_HEATERS,
                               desc="Turn off all heaters")
        gcode.register_command("M105", self.cmd_M105, when_not_ready=True)
        gcode.register_command("TEMPERATURE_WAIT", self.cmd_TEMPERATURE_WAIT,
                               desc="Wait for a temperature on a sensor")

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

    def cmd_TURN_OFF_HEATERS(self, gcmd):
        self.turn_off_all_heaters()

    def _handle_ready(self):
        # G-Code M105 temperature reporting
        self.has_started = True

    def _get_temp_msg(self, eventtime):
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
        msg = self._get_temp_msg(reactor.monotonic())
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
            gcode.respond_raw(self._get_temp_msg(eventtime))
            eventtime = reactor.pause(eventtime + 1.)

    def set_temperature(self, heater, temp, rate=0., wait=False):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_lookahead_callback((lambda pt: None))
        heater.set_temp(temp, rate)
        if wait and temp:
            self._wait_for_temperature(heater)

    def cmd_SET_HEATER_TEMPERATURE(self, gcmd):
        heater_name = gcmd.get('HEATER', '')
        heater = None
        if heater_name:
            heater = self.get_heater(heater_name)
            if heater is None:
                raise gcmd.error(f"Error on 'SET_HEATER_TEMPERATURE': unknown heater '{heater_name}'")
        else:
            heater = self.get_default_heater()
        temp = gcmd.get_float('TARGET', 0.)
        rate = gcmd.get_float('RATE', 0.)
        hold = gcmd.get_float('HOLD', 0.)
        if hold and not rate:
            logging.warning('Ignoring HOLD because RATE is not provided')
        self.set_temperature(heater, temp, rate)
        if rate:
            self.temperature_wait_ramp(heater, hold=hold)

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
            temp, target = sensor._get_temp_msg(eventtime)
            if temp >= min_temp and temp <= max_temp:
                return
            print_time = toolhead.get_last_move_time()
            gcmd.respond_raw(self._get_temp_msg(eventtime))
            eventtime = reactor.pause(eventtime + 1.)

    def get_heater(self, heater_name: str):
        if heater_name not in self.available_sensors:
            return None
        if heater_name in self.heaters:
            return self.heaters[heater_name]
        else:
            return self.printer.lookup_object(heater_name)

    def get_default_heater(self):
        return None
        # TODO:
        # - bed
        # - only one heater defined -> return this heater
        # - raise error / return None

    def temperature_wait_ramp(self, sensor, hold=0.):
        err_msg = f"Unknown sensor '{sensor}'"
        if isinstance(sensor, str):
            sensor = self.get_heater(sensor)
        if sensor is None:
            gcode = self.printer.lookup_object("gcode")
            raise gcode.error(err_msg)

        reactor = self.printer.get_reactor()
        eventtime = reactor.monotonic()
        gcode = self.printer.lookup_object("gcode")

        # Wait for ramp target to be reached.
        while not self.printer.is_shutdown() and sensor.get_ramp_target():
            msg = self._get_temp_msg(eventtime)
            gcode.respond_raw(msg)
            eventtime = reactor.pause(eventtime + 1.)

        # Wait for 'hold' amount of hours.
        hold_start = eventtime
        while not self.printer.is_shutdown() and eventtime - hold_start <= hold * 3600:
            msg = self._get_temp_msg(eventtime)
            gcode.respond_raw(msg)
            eventtime = reactor.pause(eventtime + 1.)


def load_config(config):
    return PrinterHeaters(config)
