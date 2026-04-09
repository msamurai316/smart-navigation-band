# 🦾 Smart Navigation Band for Blind People

> A wearable obstacle-detection device providing real-time directional haptic feedback to visually impaired users — no audio dependency, no smartphone required.

![Arduino](https://img.shields.io/badge/Platform-Arduino%20Nano-00979D?style=flat-square&logo=arduino)
![Language](https://img.shields.io/badge/Language-C%2B%2B%20(Arduino)-blue?style=flat-square)
![License](https://img.shields.io/badge/License-MIT-green?style=flat-square)
![Status](https://img.shields.io/badge/Status-Prototype%20Ready-brightgreen?style=flat-square)
![Budget](https://img.shields.io/badge/Budget-Under%20₹1500-orange?style=flat-square)

---

## 📌 Overview

The Smart Navigation Band is a wrist-worn assistive device designed for visually impaired individuals. It uses **three ultrasonic sensors** to detect obstacles in the left, center, and right directions, and delivers **proportional vibration feedback** through three vibration motors — stronger buzz = closer obstacle.

Unlike audio-based navigation aids, this device works silently, making it usable in noisy environments and non-intrusive in public spaces.

---

## ✨ Key Features

- **Directional awareness** — separate left, center, and right feedback
- **Proportional intensity** — vibration strength increases as obstacle gets closer
- **Noise filtering** — median-of-5 + moving average filter eliminates false readings
- **Smooth output** — PWM ramping prevents jarring sudden motor changes
- **Non-blocking code** — runs continuously using `millis()`, no freezing
- **Rechargeable** — 18650 Li-ion with TP4056 USB charging

---

## 🧰 Hardware Components

| Component | Quantity | Approx. Cost |
|---|---|---|
| Arduino Nano | 1 | ₹180 |
| HC-SR04 Ultrasonic Sensor | 3 | ₹150 |
| 3V Coin Vibration Motor | 3 | ₹90 |
| BC547 NPN Transistor | 3 | ₹15 |
| 1N4007 Flyback Diode | 3 | ₹10 |
| 1kΩ Resistor | 3 | ₹5 |
| 18650 Li-ion Battery | 1 | ₹180 |
| TP4056 Charging Module | 1 | ₹60 |
| MT3608 Boost Converter (5V) | 1 | ₹80 |
| Wrist Band / Velcro | 1 | ₹80 |
| PCB / Breadboard + Wires | — | ₹120 |
| **Total** | | **~₹970–₹1100** |

> All components available on [Robu.in](https://robu.in), [Electronicscomp.com](https://electronicscomp.com), or SP Road / Lamington Road.

---

## 📐 System Architecture

```
[ 18650 Li-ion ] → [ TP4056 ] → [ MT3608 Boost 5V ] → [ Arduino Nano (VIN) ]
                                                               │
                    ┌──────────────────────────────────────────┤
                    │                   │                      │
             HC-SR04 (Left)     HC-SR04 (Center)      HC-SR04 (Right)
              D8/D9               D4/D5                 D6/D7
                    │                   │                      │
                    └──────────────────────────────────────────┤
                                        │
                         Median Filter + Moving Average
                         Distance → PWM mapping
                                        │
                    ┌──────────────────────────────────────────┤
                    │                   │                      │
             Motor L (D10)       Motor C (D11)         Motor R (D3)
            BC547 + Diode       BC547 + Diode          BC547 + Diode
```

---

## 🔌 Pin Connections

### Ultrasonic Sensors

| Sensor | TRIG Pin | ECHO Pin | VCC | GND |
|---|---|---|---|---|
| Left | D8 | D9 | 5V | GND |
| Center | D4 | D5 | 5V | GND |
| Right | D6 | D7 | 5V | GND |

### Vibration Motors (via BC547 driver)

| Motor | Arduino PWM Pin | Transistor Base Resistor |
|---|---|---|
| Left | D10 | 1kΩ |
| Center | D11 | 1kΩ |
| Right | D3 | 1kΩ |

### Motor Driver Wiring (per motor)
```
Arduino PWM ──[1kΩ]──► BC547 Base
                        BC547 Emitter ──► GND
                        BC547 Collector ──► Motor (−)
5V ──────────────────────────────────► Motor (+)
1N4007 diode: Anode → Collector, Cathode → 5V  (flyback protection)
```

---

## 💻 Software

### Requirements
- [Arduino IDE](https://www.arduino.cc/en/software) (v1.8+ or v2.x)
- No external libraries required — uses only built-in Arduino functions

### Upload Instructions
1. Clone this repository:
   ```bash
   git clone https://github.com/YOUR_USERNAME/smart-navigation-band.git
   ```
2. Open `src/smart_nav_band.ino` in Arduino IDE
3. Select **Board**: Arduino Nano
4. Select **Processor**: ATmega328P (Old Bootloader) ← important for clone Nanos
5. Select the correct **COM Port**
6. Click **Upload**

### How the Code Works

The firmware runs a continuous non-blocking loop:

1. Every 80ms, all three sensors are read
2. Each sensor takes **5 rapid readings**; the median is selected (eliminates spikes)
3. A **moving average buffer** (last 4 readings) smooths the output further
4. Distance is **mapped inversely to PWM**: 150cm → PWM 30, 20cm → PWM 255
5. PWM values **ramp gradually** to the target (no sudden jumps)
6. Center obstacle triggers both left + right motors at 70% intensity (directional cue)
7. `analogWrite()` drives the motors via transistors

---

## 📊 Vibration Feedback Logic

| Situation | Left Motor | Center Motor | Right Motor |
|---|---|---|---|
| Nothing detected | Off | Off | Off |
| Obstacle on left | Vibrates (∝ distance) | Off | Off |
| Obstacle in center | Vibrates (70%) | Vibrates (100%) | Vibrates (70%) |
| Obstacle on right | Off | Off | Vibrates (∝ distance) |
| Multiple obstacles | Each responds independently | | |

---

## 🧪 Testing Procedure

1. **Power test** — Verify MT3608 output is exactly 5.0V before connecting Arduino
2. **Sensor test** — Run sensor-only sketch; hold hand at 10/30/100cm, verify stable Serial output
3. **Motor test** — Write `analogWrite(MOTOR_L, 200)` and confirm motor vibrates
4. **Integration test** — Upload full code, walk toward a wall, verify increasing vibration
5. **Directional test** — Move objects left/right, confirm correct motor responds
6. **Blind simulation** — Close eyes, wear band, navigate a simple path

---

## 🔧 Optional Upgrades

- **Buzzer backup** — passive buzzer on A0 for obstacles under 20cm (₹15)
- **Sensitivity pot** — 10kΩ potentiometer on A1 lets user adjust detection range
- **Waterproofing** — conformal coating on PCB; heat-shrink on motors
- **Custom PCB** — designed in EasyEDA, ordered from JLCPCB (~₹200 for 5 boards)
- **OLED debug display** — 0.96" I2C OLED (SDA→A4, SCL→A5) for distance readout

---

## 📁 Repository Structure

```
smart-navigation-band/
├── src/
│   └── smart_nav_band.ino      # Main Arduino firmware
├── docs/
│   ├── block_diagram.png       # System block diagram
│   ├── circuit_diagram.png     # Full circuit schematic
│   └── components_list.pdf     # Detailed BOM
├── images/
│   └── prototype.jpg           # Photo of built prototype
└── README.md
```

---

## 👤 Author

Nishat Dhanpati Pramanick 
B.E. Electronics & Telecommunication Engineering — 2nd Year  
Viva Institute of Technology

---

## 📄 License

This project is licensed under the MIT License — free to use, modify, and distribute with attribution.

---

## 🤝 Contributing

Pull requests are welcome. If you test this and improve the filtering algorithm or add new features (GPS integration, BLE companion app, etc.), feel free to open a PR.

---

*Built as a minor project for Electronics & Telecommunication Engineering. Designed to be practical, low-cost, and genuinely useful.*
