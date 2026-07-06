# extras/filament_watcher.py
class FilamentWatcher:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.detection_length = config.getfloat('detection_length', 3.5, above=0.)
        self.runout_gcode = config.get('runout_gcode', None)
        raw_sources = [s.strip() for s in config.get('position_sources').split(',')]
        self.source_names, self.backlash = [], {}
        for entry in raw_sources:
            name, backlash_mm = entry.split(':')
            self.source_names.append(name.strip())
            self.backlash[name.strip()] = float(backlash_mm)

        self.sources = {}          # name -> object, resolved at klippy:ready
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

    def _handle_ready(self):
        for name in self.source_names:
            obj = self.printer.lookup_object(name)
            self.sources[name] = obj
            self.last_pos[name] = 0.
            self.last_dir[name] = 0
            self.grace_remaining[name] = 0.

    def _encoder_event(self, eventtime, state):
        self.distance_since_pulse = 0.

    def _poll(self, eventtime):
        in_grace = False
        for name, obj in self.sources.items():
            if hasattr(obj, 'find_past_position'):
                mcu = self.printer.lookup_object('mcu')
                print_time = mcu.estimated_print_time(eventtime)
                pos = obj.find_past_position(print_time)
            else:
                pos = obj.get_position(eventtime)
            delta = pos - self.last_pos[name]
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
            self.distance_since_pulse += abs(delta)

        if not in_grace and self.distance_since_pulse > self.detection_length:
            self._trigger_runout()
            self.distance_since_pulse = 0.  # avoid re-firing every tick

        return eventtime + 0.1  # 100ms poll

    def _trigger_runout(self):
        if self.runout_gcode:
            gcode = self.printer.lookup_object('gcode')
            template = self.printer.lookup_object('gcode_macro').load_template(
                self.config, 'runout_gcode')
            gcode.run_script(template.render())

def load_config_prefix(config):
    return FilamentWatcher(config)