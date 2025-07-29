# Safety Module & Robotnik Modbus Integration

This repository contains two closely-related ROS nodes:

1. **`robotnik_modbus`** - Exposes the PLC's discrete I/O as a Modbus bit-map that other ROS nodes can read and write.
2. **`safety_module`** - Encapsulates safety-related logic (laser modes, watchdog, speed feedback, etc.) and exchanges information with the PLC through the bit-map provided by `robotnik_modbus`.

---

## 1. Robotnik Modbus

### Purpose
Map human-readable **names** to individual **bits** in the PLC's Modbus table so that the rest of the ROS stack can address I/O by name instead of by numeric register/bit indices.

### Configuration file
The node expects a YAML file such as:

```yaml
digital_inputs:
  - name: emergency_stop
    id: 228
  - name: laser_enabled
    id: 230
  # …

digital_outputs:
  - name: speed_bit_0
    id: 49
  - name: speed_bit_1
    id: 50
  # …
```

- **Names** were formerly known as *named_inputs_outputs* in ROS and **must be unique**.
- **IDs start at 1**.
- In the PLC each entry is an 8-bit register.
  - The ID can be computed with

    `id = n * 8 + b`

    where `n` is the **register number** (starting at 0) and `b` is the **bit position** inside that register (also starting at 0).

- Both **topics** and **services** accept either the `id` *or* the `name`.

---

## 2. Safety Module

### Main features

| Feature | Description |
|---------|-------------|
| **Laser mode switching** | Selects the safety zones of the front/rear lasers. |
| **Watchdog** | Sends a toggling signal so the PLC knows that PC↔PLC communication is alive. |
| **State aggregation** | Publishes a single “global state” derived from multiple PLC inputs (E-Stop, key switch, selector, etc.). |
| **Speed feedback** | Writes the robot's internally computed speed as a 12-bit word to the PLC. |

### Configuration file
The node reads a YAML file organised in four sections:

#### 2.1 Global tags

Aggregates high-level safety signals.

```yaml
global:
  emergency_stop: emergency_stop
  safety_stop: safety_stop
  selector_mode_auto: selector_mode_auto
  selector_mode_manual: selector_mode_manual
  selector_mode_maintenance: selector_mode_maintenance
  laser_mute: laser_mute
```

*Left-hand keys are **fixed**; right-hand values are the bit-names defined in `robotnik_modbus`.*

---

#### 2.2 Watchdog

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
