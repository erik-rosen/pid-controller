# Boat Engine Governor

A closed-loop electronic throttle governor for a small marine engine, built on a Raspberry Pi Pico.

## What it does

Maintains a target engine RPM by automatically adjusting the throttle via an RC servo and Bowden cable. A potentiometer sets the desired RPM (530–2500), and a toggle switch selects between manual throttle control and automatic closed-loop mode.

## Hardware

- **MCU:** Raspberry Pi Pico / Pico W (RP2040)
- **Throttle actuator:** 20 kg RC servo with Bowden cable linkage
- **RPM sensor:** NJK inductive sensor (4 pulses/rev) with optocoupler isolation
- **Power:** 12V boat electrical system via XL4015 buck converter (~5.1V out)
- **Controls:** Potentiometer (GP26), manual/auto toggle switch (GP16)

### Pin assignments

| Function | GPIO | Physical pin |
|----------|------|-------------|
| Servo PWM | GP15 | 20 |
| Mode switch | GP16 | 21 |
| RPM sensor | GP22 | 29 |
| Potentiometer | GP26 (ADC0) | 31 |
| Status LED | GP25 | onboard |

## Control strategy

- **Feed-forward table** maps target RPM to an approximate servo position
- **PI controller** corrects the remaining error (Kp = 0.3)
- **Adaptive FF offset** slowly shifts the feed-forward curve to compensate for load changes (in gear, wind, current)
- **Watchdog timer** (500ms) reboots the Pico if the loop ever hangs

## Servo calibration

The servo range is calibrated to the physical throttle linkage:
- 1072 us = throttle fully closed
- 1732 us = throttle fully open

## Helper sketches

- **`servo-calibrate/`** — Step through servo positions with spacebar to find mechanical limits
- **`servo-sweep/`** — Continuously sweep the servo across its full range for testing

## Building

Open `pid-controller.ino` in the Arduino IDE with the [Arduino-Pico core](https://github.com/earlephilhower/arduino-pico) installed. Select "Raspberry Pi Pico" as the board and upload.

## License

MIT
