# Autonomous Glider 

![Build Status](https://img.shields.io/github/last-commit/RaymondBello/Glider-Autopilot) ![Build Status](https://img.shields.io/github/issues-raw/RaymondBello/Glider-Autopilot) ![Build Status](https://img.shields.io/github/contributors/RaymondBello/Glider-Autopilot?color) 
![Build Status](https://img.shields.io/github/languages/top/RaymondBello/Glider-Autopilot) ![Build Status](https://img.shields.io/github/languages/count/RaymondBello/Glider-Autopilot) 
![Build Status](https://img.shields.io/github/repo-size/RaymondBello/Glider-Autopilot?color=red) 

This an embedded flight control system along with various GUI's used to monitor the state of the Aircraft.
Such as the GCS (Ground Control System)```GCS.py``` used to debug/test along with a PFD 
(Primary Flight Display)```PFD.py``` used to plan flight waypoints as well as monitor the gliders attitude in real-time

## Hardware Used
1. ESP-32-WROOM-32 - MCU (@ 240MHz)
2. MPU-9250 - IMU (Accelerometer, Gyroscope & Magnetometer) for attitude estimation
3. BMP-280 - Barometric Pressure & Altitude Sensor 
4. BN-220 - GPS Module (U-Blox M8030-KT) [GLONASS format]