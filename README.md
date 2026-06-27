## Raspberry Pi Object Tracking System (OpenCV + Servo + PID)
## Table of Contents

- [How It Works](#How-It-Works)
- [How to Run](#How-to-Run)

**About the Project**

This project is a real-time object tracking and control system using a Raspberry Pi, OpenCV, and servo motors.
The system detects a red-colored object from a live camera feed and automatically tracks it using a PID-based control mechanism.

Additionally, a relay can be triggered when the object is detected.

## Features
- Real-time camera stream processing (OpenCV)
- Color-based object detection (HSV filtering)
- Object tracking using contour detection
- PID control for smoother movement
- Dual-axis servo motor control (X & Y)
- GPIO relay control (on object detection)
- Raspberry Pi hardware integration

## How It Works
1. Camera captures live video feed
2. Image is converted to HSV color space
3. Red object is filtered using color thresholds
4. Largest contour is selected as target object
5. Object center is calculated
6. Error between object center and frame center is computed
7. PID controller adjusts servo movement
8. Servo motors track the object in real time
9. Relay is activated when object is detected

## Hardware Requirements
- Raspberry Pi (with GPIO support)
- USB or CSI Camera
- 2x Servo Motors (X and Y axis)
- Relay Module
- External power supply (recommended for servos)

## Dependencies
```bash
pip install opencv-python numpy RPi.GPIO
```
## How to Run
```bash
python3 main.py <unused> <camera_width> <camera_height> <camera_index>
```
**Example:**
```bash
python3 main.py 0 640 480 0
```
## GPIO Configuration
- Servo X - GPIO 11
- Servo Y - GPIO 13
- Relay - GPIO 15
