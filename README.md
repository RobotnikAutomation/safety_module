# Safety Module

### Main features

| Feature | Description |
|---------|-------------|
| **Laser mode switching** | Selects the safety zones of the front/rear lasers. |
| **Watchdog** | Sends a toggling signal so the PLC knows that PC↔PLC communication is alive. |
| **State aggregation** | Publishes a single “global state” derived from multiple PLC inputs (E-Stop, key switch, selector, etc.). |
| **Speed feedback** | Writes the robot's internally computed speed as a 12-bit word to the PLC. |

### Configuration file
The node reads a YAML file organised in four sections. Each section defines a set of high-level tags that are mapped to the Modbus bits defined in the `robotnik_io_msgs` message definitions.

#### 2.1 Global tags

Aggregates high-level safety signals, used to publish general status.

```yaml
global:
  emergency_stop: emergency_stop
  safety_stop: safety_stop

  selector_mode_auto: selector_mode_auto
  selector_mode_manual: selector_mode_manual
  selector_mode_maintenance: selector_mode_maintenance

  laser_mute: laser_mute
```

*Left-hand keys are **fixed** and must be filled right-hand values are the bit-names defined in `robotnik_modbus`.*

---

#### 2.2 Watchdog

This feature sends a toggling signal to the PLC to indicate that the PC is still alive and communicating.

```yaml
watchdog:
  enabled: true
  period_ms: 1400
  signal_a: watchdog_signal_a
  signal_b: watchdog_signal_b
```

* When `enabled` is `false` no watchdog is transmitted.
* `period_ms` is the full square-wave period.
* `signal_b` is the logic-negated copy of `signal_a`.

---

#### 2.3 Speed controller

This feature writes the robot's speed as a 12-bit word to the PLC, allowing the PLC to read the current speed.

```yaml
speed:
  enabled: true
  period_ms: 100
  prefix: speed_bit_
```

* Writes 12 bits (`<prefix>0` … `<prefix>11`) at the specified period.
* All 12 bit-names must exist in `robotnik_modbus`.

---

#### 2.4 Laser controller

This feature allows switching between different laser modes, which can be used to adapt the robot's behavior based on the current task or environment.

```yaml
laser:
  default_mode: standard        # Mode used at node start-up or when PLC comms are lost
  modes:
    standard:                   # --- Mode definition ---------------------------
      input:
        laser_mode_standard: true
      output:
        laser_mode_standard_legacy_1: false
        laser_mode_standard_legacy_2: false
        laser_mode_standard_legacy_3: false
        laser_mode_standard_legacy_4: false

    charging_station:
      input:
        laser_mode_charging_station: true
      output:
        laser_mode_standard_legacy_1: true
        laser_mode_standard_legacy_2: true
        laser_mode_standard_legacy_3: true
        laser_mode_standard_legacy_4: true

  attributes:                   # --- Laser diagnostic bits ---------------------
    front_laser:
      detecting_obstacles: front_laser_detecting_obstacles
      contamination: front_laser_contamination_led
      free_warning: front_laser_free_warning

    rear_laser:
      detecting_obstacles: rear_laser_detecting_obstacles
      contamination: rear_laser_contamination_led
      free_warning: rear_laser_free_warning
```

* **`laser.default_mode`**
  *If empty, no mode is forced on start-up.*

* **`laser.modes.<name>`**
  *Each mode describes*
  - **`input`** - which bit(s) confirm that the PLC has applied the mode.
  - **`output`** - which bit(s) must be written to request the mode.

* Add as many modes as needed; simply copy an existing block and adjust the bit-names.

* **`laser.attributes`**
  Diagnostics for each laser (obstacle detection, contamination LED, etc.).
  The field names (`detecting_obstacles`, `contamination`, `free_warning`) are fixed; change only the bit-names on the right.

---

## License

Distributed under the BSD-3-Clause license. See [LICENSE](LICENSE) for details.
