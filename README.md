
# Bike Braking & Lean Detection – Technical Prototype

## Overview

This repository contains the firmware for an early-stage **bike motion sensing prototype** built to evaluate whether braking and left/right leaning can be detected reliably using low-cost sensors and simple signal processing.

The system was intentionally built as a **hardware-first feasibility test**, not a production design. The focus is on signal validity, not packaging, power efficiency, or robustness.

## Prototype Architecture

### Core Components

Based on the physical setup:

- **Microcontroller**
  - Arduino-class board (5V logic)
  - Responsible for sensor sampling, signal processing, and output indication

- **Inertial Measurement Unit (IMU)**
  - 3-axis accelerometer and gyroscope
  - Used to detect:
    - Longitudinal acceleration (braking)
    - Roll angle or lateral acceleration (lean left/right)

- **Power Source**
  - Single 18650 Li-ion cell
  - Directly wired (no charging or protection circuitry in this prototype)

- **Indicators**
  - Discrete LEDs on the breadboard
  - Used to visualize detected states (e.g. braking, left lean, right lean)

- **Prototyping Hardware**
  - Solderless breadboard
  - Jumper wires
  - No mechanical fixation or vibration isolation

### Physical Layout Notes

- Sensors are breadboard-mounted and free-floating
- Orientation consistency matters and is assumed, not enforced
- Wiring length and noise are not controlled
- This setup is **not vibration-safe** and not intended for extended riding

## Detection Logic

### Braking Detection

Braking is inferred from **negative longitudinal acceleration**:

- Acceleration data is sampled from the IMU
- A threshold-based approach is used:
  - If deceleration exceeds a defined limit for a minimum duration, a braking event is flagged
- Filtering is minimal (basic smoothing or averaging where needed)

This approach proved sufficient to distinguish braking from normal riding noise in controlled tests.

### Lean Detection (Left / Right)

Lean direction is estimated using either:
- Roll angle derived from accelerometer data, or
- Lateral acceleration combined with gyro input

Logic:
- Neutral zone defined around upright position
- Lean left if signal exceeds negative threshold
- Lean right if signal exceeds positive threshold

Indicators:
- LEDs are activated to represent detected lean direction in real time

## Firmware Structure

Typical structure:

- `setup()`
  - Sensor initialization
  - Pin configuration
  - Calibration (static offset assumptions)

- `loop()`
  - Read IMU data
  - Apply basic filtering
  - Evaluate braking and lean thresholds
  - Update LED indicators
  - Serial output (optional, for debugging)

The code prioritizes readability and iteration speed over abstraction.

## Calibration Assumptions

- Initial power-on occurs while the bike is stationary and upright
- Sensor biases are assumed stable for the duration of testing
- No dynamic recalibration or drift compensation is implemented

## Known Limitations

This prototype intentionally ignores:

- Mechanical stability and enclosure design
- Sensor alignment guarantees
- Power management and battery safety
- Weather resistance
- Long-term drift and temperature effects
- Signal fusion sophistication

As designed, the system is **not suitable for real-world deployment** without a full redesign.

## Results

- Braking events are clearly detectable
- Left/right lean separation is reliable within practical riding angles
- Signal-to-noise ratio is sufficient for event classification
- The core question of feasibility is answered positively

## Status

**Prototype complete and parked.**

Further work would require:
- Custom PCB
- Rigid sensor mounting
- Proper power regulation and protection
- More advanced filtering or sensor fusion
- Real-world vibration and durability testing

## Purpose of This Repository

This code exists to document:
- A validated technical direction
- The logic used to interpret raw sensor data
- A proof that the problem is solvable with modest hardware

It is shared for learning, exploration, and iteration, not reuse as-is.
