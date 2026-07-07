# extras/buffer_manager.py
#
# Host-side interface to the LLL Buffer Plus firmware module (buffer.c).
# Provides:
#  - BUFFER_SET_MODE / BUFFER_QUERY gcode commands for load/unload macros
#  - a position_sources-compatible object (get_position/get_direction) for
#    [filament_watcher] backlash tracking
#  - live hall/error state exposed via printer status
#
# This file may be distributed under the terms of the GNU GPLv3 license.

MODE_AUTO = 0
MODE_FORWARD = 1
MODE_BACK = 2
MODE_STOP = 3
MODE_NAMES = {'AUTO': MODE_AUTO, 'FORWARD': MODE_FORWARD,
              'BACK': MODE_BACK, 'STOP': MODE_STOP}
MODE_DIRECTION = {MODE_AUTO: 0, MODE_FORWARD: 1, MODE_BACK: -1, MODE_STOP: 0}

KEEPALIVE_INTERVAL = 0.25   # seconds between re-armed sends while overridden
MCU_TIMEOUT_MS = 1000       # firmware watchdog window per send - must be > KEEPALIVE_INTERVAL
QUERY_INTERVAL = 0.25       # seconds between buffer_query_state polls


class BufferManager:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.name = config.get_name().split()[-1]

        mcu_name = config.get('mcu', self.name)
        self.mcu = self.printer.lookup_object('mcu ' + mcu_name)

        # mm/s of buffer travel at commanded VACTUAL - measure this, don't
        # trust a value derived from the firmware's RPM/microstep constants,
        # since it also depends on pulley/wheel diameter.
        self.feed_speed = config.getfloat('feed_speed_mm_s', above=0.)

        self.host_mode = MODE_AUTO
        self.direction = 0
        self.est_position = 0.
        self.last_update_t = None

        self.state = {'hall1': 0, 'hall2': 0, 'hall3': 0, 'fil': 0,
                      'error': 0, 'motor_state': 0, 'host_mode': 0}
        self.last_state_t = None

        self.set_mode_cmd = None
        self.query_cmd = None
        self.keepalive_timer = self.reactor.register_timer(
            self._keepalive, self.reactor.NEVER)
        self.query_timer = self.reactor.register_timer(
            self._do_query, self.reactor.NEVER)

        self.printer.register_event_handler('klippy:connect',
                                             self._handle_connect)
        self.printer.register_event_handler('klippy:disconnect',
                                             self._handle_disconnect)

        gcode = self.printer.lookup_object('gcode')
        gcode.register_mux_command(
            'BUFFER_SET_MODE', 'BUFFER', self.name,
            self.cmd_BUFFER_SET_MODE,
            desc="Override buffer motor mode: MODE=FORWARD|BACK|STOP|AUTO")
        gcode.register_mux_command(
            'BUFFER_QUERY', 'BUFFER', self.name,
            self.cmd_BUFFER_QUERY,
            desc="Report last known buffer sensor/motor state")

    def _handle_connect(self):
        self.set_mode_cmd = self.mcu.lookup_command(
            'buffer_set_mode mode=%c timeout=%u')
        self.query_cmd = self.mcu.lookup_command('buffer_query_state')
        state_format = ('buffer_state hall1=%c hall2=%c hall3=%c fil=%c '
                        'error=%c state=%c host_mode=%c')
        if hasattr(self.mcu, 'register_serial_response'):
            self.mcu.register_serial_response(self._handle_state, state_format)
        else:
            self.mcu.register_response(self._handle_state, state_format)
        self.reactor.update_timer(self.query_timer, self.reactor.NOW)

    def _handle_disconnect(self):
        self.reactor.update_timer(self.keepalive_timer, self.reactor.NEVER)
        self.reactor.update_timer(self.query_timer, self.reactor.NEVER)

    # ---------------- sensor state ----------------

    def _handle_state(self, params):
        self.state = {
            'hall1': params['hall1'], 'hall2': params['hall2'],
            'hall3': params['hall3'], 'fil': params['fil'],
            'error': params['error'], 'motor_state': params['state'],
            'host_mode': params['host_mode'],
        }
        self.last_state_t = self.reactor.monotonic()

    def _do_query(self, eventtime):
        if self.query_cmd is not None:
            self.query_cmd.send()
        return eventtime + QUERY_INTERVAL

    # ---------------- mode control ----------------

    def _send_mode(self, mode):
        if self.set_mode_cmd is not None:
            self.set_mode_cmd.send([mode, MCU_TIMEOUT_MS])

    def _keepalive(self, eventtime):
        if self.host_mode == MODE_AUTO:
            return self.reactor.NEVER
        self._send_mode(self.host_mode)
        return eventtime + KEEPALIVE_INTERVAL

    def set_mode(self, mode):
        eventtime = self.reactor.monotonic()
        self._integrate_position(eventtime)
        self.host_mode = mode
        self.direction = MODE_DIRECTION[mode]
        self._send_mode(mode)
        if mode == MODE_AUTO:
            self.reactor.update_timer(self.keepalive_timer, self.reactor.NEVER)
        else:
            self.reactor.update_timer(self.keepalive_timer,
                                       eventtime + KEEPALIVE_INTERVAL)

    def cmd_BUFFER_SET_MODE(self, gcmd):
        mode_str = gcmd.get('MODE').upper()
        if mode_str not in MODE_NAMES:
            raise gcmd.error(
                "Unknown MODE '%s', expected one of %s"
                % (mode_str, ', '.join(MODE_NAMES)))
        self.set_mode(MODE_NAMES[mode_str])
        gcmd.respond_info("Buffer %s mode set to %s" % (self.name, mode_str))

    def cmd_BUFFER_QUERY(self, gcmd):
        age = "unknown"
        if self.last_state_t is not None:
            age = "%.2fs" % (self.reactor.monotonic() - self.last_state_t)
        gcmd.respond_info(
            "Buffer %s: hall1=%d hall2=%d hall3=%d fil=%d error=%d "
            "motor_state=%d host_mode=%d age=%s"
            % (self.name, self.state['hall1'], self.state['hall2'],
               self.state['hall3'], self.state['fil'], self.state['error'],
               self.state['motor_state'], self.state['host_mode'], age))

    # ------------- position_sources interface (for filament_watcher) -------------

    def _integrate_position(self, eventtime):
        if self.last_update_t is not None:
            dt = eventtime - self.last_update_t
            self.est_position += self.direction * self.feed_speed * dt
        self.last_update_t = eventtime

    def get_position(self, eventtime=None):
        if eventtime is None:
            eventtime = self.reactor.monotonic()
        self._integrate_position(eventtime)
        return self.est_position

    def get_direction(self):
        return self.direction

    def get_status(self, eventtime):
        status = dict(self.state)
        status['host_mode_active'] = self.host_mode != MODE_AUTO
        return status


def load_config_prefix(config):
    return BufferManager(config)