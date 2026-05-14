# Boat Engine Governor Project Summary

## Overview

This project is a closed-loop electronic throttle governor for a small marine engine using:

- Raspberry Pi Pico / Pico W
- High-torque RC servo (~20 kg class)
- Hall-effect / inductive RPM sensor (NJK)
- 12V boat electrical system
- XL4015-based 5V buck regulator
- Manual / automatic throttle control

The goal is to maintain very low stable engine RPM under varying load conditions while preventing stalls and unsafe overspeed conditions.

## Core Problem

The engine's throttle backstop screw sets a fixed minimum idle position, but the correct idle position varies with engine temperature and load:

- **Cold engine:** Needs more throttle to avoid stalling — backstop must be set high
- **Hot engine in neutral:** Same backstop causes engine to race at high RPMs (dangerous when docking)
- **Hot engine in gear at low speed:** Backstop prevents throttle from closing enough for slow docking maneuvers

No single backstop position works across all conditions. The governor replaces the fixed backstop with closed-loop control: the backstop screw is set low enough for hot-engine operation, and the controller adds throttle as needed to prevent stalling when cold or under load.

## Critical Operating Range

The most important RPM range is 530–800 RPM (idle and docking). This is where:
- The margin between stalling and racing is smallest
- Engine temperature effects are most pronounced
- Feed-forward accuracy matters most
- The controller must react quickly but not overshoot

Higher RPMs (1500+) are more forgiving — the engine responds linearly and the PID can easily compensate for feed-forward inaccuracy.

---

# System Architecture

## Inputs

### RPM Sensor
- NJK inductive sensor observing rotating metal target
- 4 pulses per revolution
- Sensor isolated using optocoupler
- RP2040 interrupt-based pulse timing measurement

### Potentiometer
Used for:
- Manual throttle control
- Automatic mode RPM target selection

Mapped:
- 0% -> 400 RPM target
- 100% -> 2500 RPM target

### Manual / Automatic Switch
- LOW = MANUAL
- HIGH = AUTO

Connected:
- GP16 (physical pin 21)
- Switch to GND
- Internal pull-up enabled

---

# Outputs

## Throttle Servo
- 20 kg RC servo
- Controlled via PWM on GP15
- Calibrated:
  - 1000 us = minimum throttle
  - 2000 us = maximum throttle

---

# Power System

## Input
- 12V boat electrical system
- Fused
- TVS protection recommended

---

## Buck Converter

### Current Design
XL4015 CC/CV buck regulator module

Identified components:
- XL4015E1 regulator
- SS54 Schottky diode
- LM358 op-amp (current limiting)

Configured:
- Output voltage ≈ 5.1V
- Current limit tuned using:
  - 12V 10W halogen lamp
  - CC LED indication

---

# Power Distribution

## Star Topology

Separate wire pairs from buck output:

### Servo Rail
- Thick wire pair
- 1000–2200 uF electrolytic
- 100 nF ceramic

### MCU Rail
- Separate wire pair
- 1–2.2 ohm series resistor
- 100 uF electrolytic
- 100 nF ceramic

Purpose:
- Prevent servo current spikes from disturbing MCU rail
- Reduce brownout/reset risk

---

# Servo Brownout Detection

## Purpose
Detect:
- Servo rail voltage dips
- Buck current limiting effects
- Cranking brownouts
- Wiring issues

---

## Hardware

Voltage divider into Pico ADC.

Recommended implementation:

### Divider
- R1 = 10k
- R2 = 10k
- Ratio = 0.5

### Filter
- R3 = 1k series resistor
- 100 nF capacitor to GND

### ADC Pin
- GP27 / ADC1
- Physical pin 32

---

# RPM Measurement

## Method
Interrupt-driven pulse interval timing.

### Protections
- Glitch rejection
- Minimum pulse interval filtering
- Timeout detection

### Smoothing
Exponential moving average:
- RPM_ALPHA ≈ 0.25

---

# Control Modes

## Manual Mode

Pot directly controls servo position.

Added:
- ADC averaging
- Servo update hysteresis
- Update rate limiting

Purpose:
- Eliminate servo jitter from noisy potentiometer readings

---

## Automatic Mode

Closed-loop RPM governor using:

- Feed-forward throttle map
- PI controller
- Adaptive load compensation

---

# Feed-Forward System

## Motivation
Linear throttle mapping was insufficient:
- Too little throttle at low RPM
- Excessive throttle at high RPM in neutral

---

## Solution
Piecewise linear feed-forward table:

```cpp
target RPM -> nominal servo position