# ARGUS

**ARGUS** (*Autonomous Real-time Gimbal for User Sensing*) is a face detection and tracking system. This project utilizes real-time visual feedback to control a pan-tilt gimbal, keeping a detected face centered in view.

---

## Features

- Real-time face detection and recognition using ESP-DL
- Gimbal control (pan/tilt) via PWM-driven MG90/SG90 servos
- Fully embedded — no external PC required
- Built with ESP-IDF for maximum performance

---

## Face Tracking Flow

- Camera captures frame.
- Frame is passed to esp-dl for face detection.
- Detected face coordinates are compared against frame center.
- PID control logic calculates error values.
- Servo adjustment commands (to be integrated) use calculated PD output.

---

## Hardware

- ESP32-CAM (AI-Thinker)
- Pan-tilt servo mount (SG90 or MG90)
- External 5V power supply for servos
- Optional: FTDI USB-to-Serial adapter for flashing

---

## Requirements

- ESP-IDF v4.4 (ESP32 recommended)
- Submodules:
  - [esp-dl](https://github.com/espressif/esp-dl)
  - [esp32-camera](https://github.com/espressif/esp32-camera)

Make sure submodules are initialized and updated:

```bash
git submodule update --init --recursive
```

---

## Getting Started

1. Set up [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html)
2. Clone this repository:
   ```bash
   git clone --recursive https://github.com/kimsniper/argus.git
   cd argus
   idf.py set-target esp32
   idf.py build
   idf.py flash monitor

---

## Video Demo

![Demo](./images/demo.gif)

https://www.linkedin.com/posts/mezaeldocoy_embeddedai-esp32-robotics-activity-7365576737612562432-wWZq?utm_source=share&utm_medium=member_android&rcm=ACoAACmY-xYBsymlj36REm4IhJ-hJ5gTkK0J9l0
