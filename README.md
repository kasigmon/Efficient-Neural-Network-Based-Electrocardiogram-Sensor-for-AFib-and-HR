# Resource-Efficient Neural Network-Based Electrocardiogram Sensor for Atrial Fibrillation Classification and Heart-Rate Extraction

Kirk A. Sigmon

_Completed as part of coursework at the Thayer School of of Engineering at Dartmouth College_

The current working target:

```text
arduino_sketch/arduino_final_sketch.ino
```

## Quick Install

Assuming that the Arduino CLI and Arduino SAMD board core are already installed:

1. Compile:

   ```bash
   arduino-cli compile --fqbn arduino:samd:nano_33_iot --libraries common/Arduino/libraries arduino_sketch
   ```

2. Connect the Arduino Nano 33 IoT and find its port:

   ```bash
   arduino-cli board list
   ```

3. Upload Lab 9, replacing `COM8` with your board's port:

   ```bash
   arduino-cli upload -p COM8 --fqbn arduino:samd:nano_33_iot --libraries common/Arduino/libraries arduino_sketch
   ```

## Build Modes

The default build is for the live electrode board with Bluetooth Low Energy enabled.  It can be disabled (where, for example, an Analog Discovery 3 is being used to analyze information via pins).

Compile for electrode mode with BLE (default):

```bash
arduino-cli compile --fqbn arduino:samd:nano_33_iot --libraries common/Arduino/libraries arduino_sketch
```

Compile for AD3 playback with no BLE:

```bash
arduino-cli compile --fqbn arduino:samd:nano_33_iot --libraries common/Arduino/libraries --build-property compiler.cpp.extra_flags=-DLAB9_USE_ELECTRODES=0 arduino_sketch
```

## Repository Layout

| Path | Purpose |
| --- | --- |
| `arduino_sketch/` | Latest Arduino sketch and logging scripts/results |
| `common/Arduino/libraries/ENGG499/` | Project-specific Arduino classifier, ADC, BLE, and signal-processing code |
| `common/Arduino/libraries/` | Pre-established Arduino dependencies (some omitted due to size/complexity) |
| `common/Python/` | Python helper scripts |

## Optional BLE Logger

The board advertises itself as `ENGG499_BOARD`. To subscribe to BLE output from a computer:

```bash
pip install bleak
python sketches/lab9/ble_subscribe_logger.py
```

## Notes

- The Arduino target is `arduino:samd:nano_33_iot`.
- Run commands from the repository root.
