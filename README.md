# SPRO4: winch tension controller

Firmware for an ATmega328p at 16 MHz that holds a set cable tension on a motor-driven winch. 4th semester project at SDU Sønderborg.

## Hardware

| Part | Connection |
|---|---|
| HX711 load cell amplifier | DT on PD2, SCK on PD3 |
| H-bridge motor driver | IN1 on PB1 (OC1A), IN2 on PB2 (OC1B) |
| Current sensor (bidirectional, 2.5 V at 0 mA) | ADC0 |
| UART telemetry | 9600 baud, 8N1 |
| Motor | 6 V, 1:120 gear, 25 mm winch radius |

## How it works

- The load cell sits under an 84.4° deflection pulley. Tension = force / (2·sin(θ/2)).
- Motor current is converted to torque and then to tension.
- A Kalman filter combines the two tension values.
- The outer tension PID (80 Hz, the HX711 rate) gives the current setpoint. The inner current PID (1 kHz, TIMER0 ISR) gives the PWM.
- Both loops have anti-windup. Reverse is limited to -200 mA.
- The motor stops above 50 N and after about 15 s.
- Each telemetry line has these fields: `SP`, `T_k`, `err`, `T_l`, `T_c`, `mA_sp`, `mA`, `PWM`.

## Known issue

The PID gains and Kalman parameters on main are not the right values.

## Build

Use PlatformIO:

```
pio run
pio run -t upload
```

With `#define TESTING 1` in `src/main.c`, the calibration mode drives PWM steps of ±100/200/300 and prints the measured current for comparison with a multimeter.
