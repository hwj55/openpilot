from dragonpilot.settings import tr

ITEMS = [
  {
    "section": "Longitudinal",
    "key": "dp_lon_apm",
    "type": "toggle_item",
    "title": lambda: tr("Adaptive Personality Mode (APM)"),
    "description": lambda: tr("The mode automatically switches based on the vehicle in front. It is recommended to maintain the standard mode."),
    "condition": "openpilotLongitudinalControl",
    "flags": "PERSISTENT",
    "param_type": "BOOL",
    "default": "0",
  },
  {
    "section": "Longitudinal",
    "key": "dp_lon_dasr",
    "type": "toggle_item",
    "title": lambda: tr("Dynamic Accel Slew Rate (DASR)"),
    "description": lambda: tr("Speed-dependent acceleration smoothing. Allows faster accel changes at low speeds for responsive city driving, smoother changes at highway speeds for comfort."),
    "condition": "openpilotLongitudinalControl",
    "flags": "PERSISTENT",
    "param_type": "BOOL",
    "default": "0",
  },
  {
    "section": "Longitudinal",
    "key": "AccelPersonalityEnabled",
    "type": "toggle_item",
    "title": lambda: tr("Dynamic Accel Personality"),
    "description": lambda: tr("Dynamic acceleration switch"),
    "condition": "openpilotLongitudinalControl",
    "flags": "PERSISTENT",
    "param_type": "BOOL",
    "default": "0",
    "on_change": [{
      "target": "AccelPersonality",
      "action": "set_enabled",
      "condition": "value"
    }]
  },
  {
    "section": "Longitudinal",
    "key": "AccelPersonality",
    "type": "text_spin_button_item",
    "title": lambda: tr("Accel Personality"),
    "description": lambda: tr("Dynamic acceleration selection mode"),
    "condition": "openpilotLongitudinalControl",
    "flags": "PERSISTENT",
    "param_type": "INT",
    "default": "1",
    "options": [
      lambda: tr("Sport"),
      lambda: tr("Normal"),
      lambda: tr("Eco"),
    ],
  },
]