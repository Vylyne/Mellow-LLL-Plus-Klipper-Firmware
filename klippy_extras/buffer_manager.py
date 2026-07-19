# extras/buffer_manager.py
#
# Host-side interface to the LLL Buffer Plus firmware module (buffer.c).
# Forwards the toolhead filament sensor state to the buffer MCU on change,
# and issues load/unload/mode commands. No periodic host-side polling -
# the buffer firmware owns its own load/unload state machine.
#
# This file may be distributed under the terms of the GNU GPLv3 license.

MODE_AUTO = 0
MODE_FORWARD = 1
MODE_BACK = 2
MODE_STOP = 3
MODE_NAMES = {
    "AUTO": MODE_AUTO,
    "FORWARD": MODE_FORWARD,
    "BACK": MODE_BACK,
    "STOP": MODE_STOP,
}

# Mirrors buffer.c's JOB_* / JOB_RESULT_* enums - for readability only,
# the firmware sends raw ints either way.
JOB_LOAD = 1
JOB_UNLOAD = 2
JOB_RESULT_OK = 0
JOB_RESULT_STALLED = 1
JOB_RESULT_TIMEOUT = 2
JOB_RESULT_INTERRUPTED = 3


class BufferManager:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]
        self.tool = config.getint("tool", -1)
        self.mcu_name = config.get("mcu", self.name)
        self.mcu = self.printer.lookup_object("mcu " + self.mcu_name)

        self.post_extruder_sensor_name = config.get("post_extruder_sensor", None)
        self.post_extruder_sensor = None
        self.post_extruder_sensor_button_handler = None

        self.set_mode_cmd = None
        self.load_cmd = None
        self.unload_cmd = None
        self.set_sensor_cmd = None
        self.job_pending = False
        self.last_result = {"status": None, "job": None}

        self.printer.register_event_handler("klippy:connect", self._handle_connect)

        self.printer.register_event_handler("klippy:ready", self._handle_klippy_ready)

        gcode = self.printer.lookup_object("gcode")
        gcode.register_mux_command(
            "BUFFER_SET_MODE",
            "BUFFER",
            self.name,
            self.cmd_BUFFER_SET_MODE,
            desc="Override buffer motor mode: MODE=FORWARD|BACK|STOP|AUTO",
        )
        gcode.register_mux_command(
            "BUFFER_LOAD",
            "BUFFER",
            self.name,
            self.cmd_BUFFER_LOAD,
            desc="Trigger firmware-driven load sequence",
        )
        gcode.register_mux_command(
            "BUFFER_UNLOAD",
            "BUFFER",
            self.name,
            self.cmd_BUFFER_UNLOAD,
            desc="Trigger firmware-driven unload sequence",
        )

    def _handle_connect(self):
        self.set_mode_cmd = self.mcu.lookup_command(
            "buffer_set_mode mode=%c timeout=%u"
        )
        self.load_cmd = self.mcu.lookup_command("buffer_load timeout=%u")
        self.unload_cmd = self.mcu.lookup_command("buffer_unload timeout=%u")
        self.set_sensor_cmd = self.mcu.lookup_command(
            "buffer_set_toolhead_sensor state=%c"
        )
        result_format = "buffer_result status=%c job=%c"
        if hasattr(self.mcu, "register_serial_response"):
            self.mcu.register_serial_response(self._handle_result, result_format)
        else:
            self.mcu.register_response(self._handle_result, result_format)

    def _handle_klippy_ready(self):
        if self.post_extruder_sensor_name is not None:
            try:
                self.post_extruder_sensor = self.printer.lookup_object(
                    self.post_extruder_sensor_name
                )
            except self.printer.config_error as e:
                return
            self.post_extruder_sensor_button_handler = (
                self.post_extruder_sensor._button_handler
            )
            self.post_extruder_sensor._button_handler = self._post_extruder_sensor_event

    def _post_extruder_sensor_event(self, eventtime, state):
        if self.set_sensor_cmd is not None:
            self.set_sensor_cmd.send([1 if state else 0])
        if self.post_extruder_sensor_button_handler is not None:
            self.post_extruder_sensor_button_handler(eventtime, state)

    def _handle_result(self, params):
        self.job_pending = False
        self.last_result = {"status": params["status"], "job": params["job"]}

    def cmd_BUFFER_SET_MODE(self, gcmd):
        mode_str = gcmd.get("MODE").upper()
        if mode_str not in MODE_NAMES:
            raise gcmd.error(
                "Unknown MODE '%s', expected one of %s"
                % (mode_str, ", ".join(MODE_NAMES))
            )
        timeout_ms = gcmd.get_int("TIMEOUT", 1000)
        self.set_mode_cmd.send([MODE_NAMES[mode_str], timeout_ms])
        gcmd.respond_info("Buffer %s mode set to %s" % (self.name, mode_str))

    def cmd_BUFFER_LOAD(self, gcmd):
        timeout_ms = gcmd.get_int("TIMEOUT", 60000)
        self.job_pending = True
        self.last_result = {"status": None, "job": None}
        self.load_cmd.send([timeout_ms])
        gcmd.respond_info("Buffer %s: load triggered" % self.name)

    def cmd_BUFFER_UNLOAD(self, gcmd):
        timeout_ms = gcmd.get_int("TIMEOUT", 60000)
        self.job_pending = True
        self.last_result = {"status": None, "job": None}
        self.unload_cmd.send([timeout_ms])
        gcmd.respond_info("Buffer %s: unload triggered" % self.name)

    def get_status(self, eventtime):
        return {
            "tool": self.tool,
            "mcu": self.mcu_name,
            "job_pending": self.job_pending,
            "last_result_status": self.last_result["status"],
            "last_result_job": self.last_result["job"],
        }


def load_config_prefix(config):
    return BufferManager(config)
