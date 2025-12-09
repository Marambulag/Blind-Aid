# Blind Aid System

## Overview

## 🚀 Key Features

* **Hybrid Sensing Architecture:** Fuses high-resolution LiDAR data (VL53L5CX) with robust Ultrasonic measurements (HC-SR04) to detect obstacles including glass and black materials.
* **Distributed Processing:**
    * **Raspberry Pi:** Handles heavy DSP (Median Filtering, Pooling, NumPy) and Computer Vision.
    * **ESP32:** Executes Real-Time Control using **FreeRTOS** for jitter-free haptic feedback.
* **Smart Haptics:** 3x3 Vibration Matrix with linear intensity mapping (Open-Loop Control) to represent distance.
* **Silent Operation:** Motors operate at **25 kHz PWM** (Ultrasonic frequency) to avoid audible noise.
* **Safety First:** Includes brownout protection, sensor timeouts, and startup self-diagnostics.
