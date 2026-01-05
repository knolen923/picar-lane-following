# Autonomous Lane-Following Robot (PiCar-V)

This project implements a real-time autonomous lane-following system using computer vision on a Raspberry Pi–based PiCar-V.

## Demo
  YouTube Demo: [https://youtu.be/lVgorBeg3Mk](https://youtu.be/N12Zk2cqA-o?si=1rm8CjtS2xzKE3Ma)

## Features
- Lane detection using HSV color masking and contour analysis
- Region-of-interest segmentation to reduce noise
- Centroid-based lane offset calculation
- Proportional steering control for smooth navigation
- Red stop-tape detection with cooldown logic
- Real-time processing on embedded hardware

## Hardware
- Raspberry Pi 4
- Raspberry Pi AI Camera
- SunFounder PiCar-V
- Servo motor (steering)
- DC motors (drive)

## Software
- Python
- OpenCV
- Picamera2
- NumPy

## How It Works
Video frames are captured and converted to HSV color space to isolate yellow lane markings.
A region of interest is applied to focus on the lower portion of the image.
Image moments are used to compute the centroid of the detected lane, and the horizontal offset
from the image center is converted into steering commands using proportional control.

A horizontal red strip of tape is detected using combined HSV red masks and contour filtering.
When detected, the vehicle stops for three seconds, with a cooldown timer to prevent repeated stops.

## Performance
The system demonstrates consistent lane tracking and reliable stop detection while maintaining
low latency on Raspberry Pi hardware.

## Report
A full technical project report is included in this repository.
