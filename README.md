# ARGUS

**ARGUS** (*Autonomous Real-time Gimbal for User Sensing*) is a face recognition and tracking system powered by the **ESP32-CAM** and **ESP-IDF**. This project utilizes real-time visual feedback to control a pan-tilt gimbal, keeping a detected face centered in view.

---

## Features

- Real-time face detection and recognition using ESP-WHO
- Gimbal control (pan/tilt) via PWM-driven MG90/SG90 servos
- Fully embedded — no external PC required
- Built with ESP-IDF for maximum performance

---

## Hardware

- ESP32-CAM (AI-Thinker)
- Pan-tilt servo mount (SG90 or MG90)
- External 5V power supply for servos
- Optional: FTDI USB-to-Serial adapter for flashing

---

## Getting Started

1. Set up [ESP-IDF](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html)
2. Clone this repository:
   ```bash
   git clone https://github.com/kimsniper/argus.git
   cd argus
   idf.py set-target esp32
   idf.py build
   idf.py -p /dev/ttyUSB0 flash monitor
