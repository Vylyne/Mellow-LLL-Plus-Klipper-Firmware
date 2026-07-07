# extras/filament_watcher.py

class FilamentWatcher:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.confirm_window = config.getfloat('confirm_window', 5.0, above=0.)
        self.detection_length = config.getfloat('detection_length', 3.5, above=0.)
        
        # lets compile the runout gcode template now, so we catch errors at startup
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.runout_template = None
        if config.get('runout_gcode', None) is not None:
            self.runout_template = gcode_macro.load_template(config, 'runout_gcode')
        self.warn_template = None
        if config.get('warn_gcode', None) is not None:
            self.warn_template = gcode_macro.load_template(config, 'warn_gcode')

        raw_sources = [s.strip() for s in config.get('position_sources').split(',')]

        self.jam_pending_since = None
        self.escalated = False   # true once confirmed - suppresses repeat escalation

        self.source_names, self.backlash = [], {}
        for entry in raw_sources:
            name, backlash_mm = entry.split(':')
            self.source_names.append(name.strip())
            self.backlash[name.strip()] = float(backlash_mm)

        self.sources = {}
        self.last_pos = {}
        self.last_dir = {}
        self.grace_remaining = {}
        self.distance_since_pulse = 0.
        self.printer.register_event_handler('klippy:ready', self._handle_ready)

        pin = config.get('motion_pin')
        buttons = self.printer.load_object(config, 'buttons')
        buttons.register_buttons([pin], self._encoder_event)

        reactor = self.printer.get_reactor()
        reactor.register_timer(self._poll, reactor.NOW)

        self.jam_pending_since = None
        self.printer.register_event_handler('idle_timeout:printing', self._reset_pending)
        self.printer.register_event_handler('idle_timeout:ready', self._reset_pending)
        
        self.enabled = config.getboolean('enable', True)
        gcode.register_mux_command(
            'SET_FILAMENT_SENSOR', 'SENSOR', self.name,
            self.cmd_SET_FILAMENT_SENSOR,
            desc="Enable/disable jam detection for this filament_watcher")

    def get_status(self, eventtime):
        return {
            'enabled': self.enabled,
            'filament_detected': not self.escalated,
        }

    def cmd_SET_FILAMENT_SENSOR(self, gcmd):
        self.enabled = bool(gcmd.get_int('ENABLE', 1))
        gcmd.respond_info("Filament watcher %s: %s" %
                        (self.name, "enabled" if self.enabled else "disabled"))

    def _read_position(self, obj, eventtime):
        if hasattr(obj, 'find_past_position'):
            mcu = self.printer.lookup_object('mcu')
            print_time = mcu.estimated_print_time(eventtime)
            return obj.find_past_position(print_time)
        return obj.get_position(eventtime)

    def _handle_ready(self):
        eventtime = self.printer.get_reactor().monotonic()
        for name in self.source_names:
            obj = self.printer.lookup_object(name)
            self.sources[name] = obj
            self.last_pos[name] = self._read_position(obj, eventtime)
            self.last_dir[name] = 0
            self.grace_remaining[name] = 0.

    def _encoder_event(self, eventtime, state):
        self.distance_since_pulse = 0.

    def _poll(self, eventtime):
        if not self.enabled:
            for name, obj in self.sources.items():
                self.last_pos[name] = self._read_position(obj, eventtime)
            return eventtime + 0.1
        deltas = {}
        in_grace = False
        for name, obj in self.sources.items():
            pos = self._read_position(obj, eventtime)
            delta = pos - self.last_pos[name]
            deltas[name] = delta
            direction = (delta > 0) - (delta < 0)
            if direction and self.last_dir[name] and direction != self.last_dir[name]:
                self.grace_remaining[name] = self.backlash[name]
            if direction:
                self.last_dir[name] = direction
            if self.grace_remaining[name] > 0:
                self.grace_remaining[name] -= abs(delta)
                if self.grace_remaining[name] > 0:
                    in_grace = True
            self.last_pos[name] = pos

        if in_grace:
            self.distance_since_pulse = 0.  # reversal still absorbing backlash - don't count it
        else:
            for delta in deltas.values():
                self.distance_since_pulse += abs(delta)
            if self.distance_since_pulse > self.detection_length:
                self._trigger_runout(eventtime)
                self.distance_since_pulse = 0.

        return eventtime + 0.1

    def _reset_pending(self, eventtime):
        self.jam_pending_since = None
        self.distance_since_pulse = 0.
        self.escalated = False
        for name in self.sources:
            self.grace_remaining[name] = 0.
            self.last_dir[name] = 0

    def _trigger_runout(self, eventtime):
        if self.escalated:
            return # Already executed escalation - don't repeat.
        if self.jam_pending_since is not None and \
                (eventtime - self.jam_pending_since) <= self.confirm_window:
            self.jam_pending_since = None
            self.escalated = True
            self._escalate(eventtime)
        else:
            self.jam_pending_since = eventtime
            self._warn(eventtime)

    def _warn(self, eventtime):
        gcode = self.printer.lookup_object('gcode')
        gcode.respond_info("Possible filament jam on %s, watching..." % self.name)
        if self.warn_template is not None:
            context = self.warn_template.create_template_context()
            context['params'] = {'TOOL': self.name}
            gcode.run_script(self.warn_template.render(context))

    def _escalate(self, eventtime):
        if self.runout_template is not None:
            context = self.runout_template.create_template_context()
            context['params'] = {'TOOL': self.name}
            gcode = self.printer.lookup_object('gcode')
            gcode.run_script(self.runout_template.render(context))

def load_config_prefix(config):
    return FilamentWatcher(config)