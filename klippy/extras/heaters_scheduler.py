from threading import Lock
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from ..configfile import ConfigWrapper
    from ..gcode import GCodeCommand
    from ..klippy import Printer
    from ..reactor import Reactor
    from .heaters import Heater, PrinterHeaters


class ScheduleStep:
    def __init__(self, target_temperature: float, rate: float, hold: float):
        self.target_temperature = target_temperature
        self.rate = rate
        self.hold = hold


class HeatersScheduler:
    def __init__(self, config: 'ConfigWrapper'):
        self.printer: 'Printer' = config.get_printer()
        self.reactor: 'Reactor' = self.printer.get_reactor()
        self._heater_names: list[str] = config.getlist("heaters", [])
        self.heaters: list['Heater'] = []
        self.update_interval = config.getfloat("update_interval", 1.0)

        self._schedule: list['ScheduleStep'] = []
        self._current_step: 'ScheduleStep' = None
        self._step_start_time = 0.
        self._step_start_temperature = 0.
        self.paused = True
        self.lock = Lock()
        self.timer = self.reactor.register_timer(self._update_targets)

        # Register commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command("HEATING_SCHEDULE_SHOW", self.cmd_HEATING_SCHEDULE_SHOW,
                               desc="Show the current heating schedule")
        gcode.register_command("HEATING_SCHEDULE_START", self.cmd_HEATING_SCHEDULE_START,
                               desc="Start the heating schedule")
        gcode.register_command("HEATING_SCHEDULE_PAUSE", self.cmd_HEATING_SCHEDULE_PAUSE,
                               desc="Pause the heating schedule")
        gcode.register_command("HEATING_SCHEDULE_ADD_STEP", self.cmd_HEATING_SCHEDULE_ADD_STEP,
                               desc="Add step to the heating schedule")
        gcode.register_command("HEATER_OVERRIDE", self.cmd_HEATER_OVERRIDE,
                               desc="Override the PWM value for a heater")

        self.printer.register_event_handler("klippy:connect", self._handle_connect)
        self.printer.register_event_handler("gcode:request_restart", self.flush_schedule)

    def _handle_connect(self):
        pheaters: 'PrinterHeaters' = self.printer.lookup_object('heaters')
        self.heaters = [pheaters.lookup_heater(heater_name) for heater_name in self._heater_names]
        self.reactor.update_timer(self.timer, self.reactor.NOW)

    def lowest_temperature(self) -> float:
        if self.heaters:
            return min(h.last_temp for h in self.heaters)
        return 0.

    def highest_temperature(self) -> float:
        if self.heaters:
            return max(h.last_temp for h in self.heaters)
        return float("inf")

    def _next_step(self, eventtime: float):
        previous_step = self._current_step
        self._current_step = self._schedule.pop(0)
        self._step_start_time = eventtime
        if previous_step:
            self._step_start_temperature = previous_step.target_temperature
        else:
            if self._current_step.rate > 0:
                self._step_start_temperature = self.lowest_temperature()
            else:
                self._step_start_temperature = self.highest_temperature()

    def _update_targets(self, eventtime):
        if self.paused:
            self._step_start_time += self.update_interval
            return eventtime + self.update_interval

        with self.lock:
            if not self._current_step:
                if self._schedule:
                    self._next_step(eventtime)
                else:
                    return eventtime + self.update_interval

            while True:
                rate = self._current_step.rate
                target = self._current_step.target_temperature
                # Skip steps that already have completed. This makes restarts of full schedules mid-fire easier.
                if (rate > 0 and target < self.lowest_temperature()) or (rate < 0 and target > self.highest_temperature()):
                    self._current_step = None
                    if not self._schedule:
                        return eventtime + self.update_interval
                    self._next_step(eventtime)
                else:
                    break

            rate = self._current_step.rate
            current_target = self._step_start_temperature + (rate / 3600) * (eventtime - self._step_start_time)
            if rate > 0:
                current_target = min(current_target, self._current_step.target_temperature)
            else:
                current_target = max(current_target, self._current_step.target_temperature)

            temp_delta = self._current_step.target_temperature - self._step_start_temperature
            end_time = self._step_start_time + temp_delta / (rate / 3600) + self._current_step.hold * 60
            if eventtime >= end_time:
                # TODO: print something to commandline
                self._next_step(eventtime)

        for heater in self.heaters:
            heater.set_temp(current_target)
        return eventtime + self.update_interval

    def flush_schedule(self):
        with self.lock:
            self._schedule = []

    def pause(self):
        self.paused = True

    def resume(self):
        self.paused = False

    def cmd_HEATING_SCHEDULE_PAUSE(self, _gcmd: 'GCodeCommand'):
        self.pause()

    def cmd_HEATING_SCHEDULE_START(self, _gcmd: 'GCodeCommand'):
        self.resume()

    def cmd_HEATING_SCHEDULE_SHOW(self, gcmd: 'GCodeCommand'):
        lines = ["| step |     rate |  target |    hold |  time |"]
        hours = minutes = seconds = 0
        with self.lock:
            previous_target = self.lowest_temperature()
            for i, step in enumerate(self._schedule):
                temp_delta = step.target_temperature - previous_target
                seconds += temp_delta / (step.rate / 3600) + step.hold * 60
                previous_target = step.target_temperature
                minutes += int(seconds // 60)
                seconds %= 60
                hours += int(minutes // 60)
                minutes %= 60

                lines.append(
                    f"|{str(i).rjust(5)} "
                    f"|{str(round(step.rate)).rjust(4)} °C/h "
                    f"|{str(round(step.target_temperature)).rjust(5)} °C "
                    f"|{str(int(step.hold)).rjust(4) + ' min ' if step.hold else '       - '}"
                    f"|{str(hours).rjust(3)}:{str(minutes).zfill(2)} |")

        msg = "\n".join(lines)
        did_ack = gcmd.ack(msg)
        if not did_ack:
            gcmd.respond_raw(msg)

    def cmd_HEATING_SCHEDULE_ADD_STEP(self, gcmd: 'GCodeCommand'):
        target = gcmd.get_float("TARGET")
        rate = gcmd.get_float("RATE")
        if rate == 0.:
            raise gcmd.error("RATE must be non-zero")
        hold = gcmd.get_float("HOLD", default=0.)
        # TODO: convert to bool
        clear = gcmd.get("CLEAR", default=False)
        start = gcmd.get("START", default=False)
        self.add_heating_schedule_step(target, rate, hold, clear, start)

    def add_heating_schedule_step(self, target: float, rate: float, hold: float = 0.,
                                  clear: bool = False, start: bool = True):
        step = ScheduleStep(target_temperature=target, rate=rate, hold=hold)
        with self.lock:
            if clear:
                self._previous_step = None
                self._schedule = []
            self._schedule.append(step)

        if start:
            self.paused = False

    def cmd_HEATER_OVERRIDE(self, gcmd: 'GCodeCommand'):
        heater_name = gcmd.get('HEATER')
        pwm_value = gcmd.get_float('PWM')

        pheaters = self.printer.lookup_object('heaters')
        heater = pheaters.lookup_heater(heater_name)
        heater.override_pwm_value(pwm_value)


def load_config(config):
    scheduler = HeatersScheduler(config)
    config.get_printer().add_object('heaters_scheduler', scheduler)
    return scheduler
