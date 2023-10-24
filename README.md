[![Maintenance](https://img.shields.io/badge/Maintained%3F-yes-green.svg)](https://github.com/CuboiLeo/Robomaster_Mecanum_Infantry_2022/pulse) 
[![Maintainer](https://img.shields.io/badge/Maintainer-Leo-blue)](https://github.com/CuboiLeo)
[![MIT license](https://img.shields.io/badge/License-MIT-blue.svg)](https://lbesson.mit-license.org/)

# Purdue Robomaster Two Wheeled Self Balancing Standard 2023
- Written using STM32 HAL Library and CubeMX.
- Main Control Board: DJI Robomaster Board A(STM32F427IIH6)

## Basic Components Include:
- 2 * MF9025 Motors for Chassis
- 2 * M3508 Motors for Friction Wheels
- 2 * GM6020 Motors for Gimbal (Yaw & Pitch)
- 1 * M2006 Motor for Trigger

## Current Function & Future Improvements
- Communications
  - [x] DR16 Remote Control
  - [x] Board A IMU
  - [x] Motors
  - [x] Super Capacitor
  - [x] Board A to Board A
  - [x] Referee System
  - [x] Orin 

- Basic Controls
  - [x] Chassis (Balance, Forward/Backward, Rotate)
  - [x] Gimbal (Yaw, Pitch)
  - [x] Shooting (Friction Wheel On/Off, Single Shot, Burst)
  
- Advance Controls
  - [x] Follow Gimbal Mode (Chassis Follow Gimbal)
  - [x] SpinTop Mode (Gimbal Holds in Position while Chassis Spins)
  - [x] Follow Wheel Mode (Wheel Side Facing Front)
  - [x] Swing Mode (Continuous 180 Degree Rotation About Wheels Axis)  
  - [x] Power Limiting (Requires Referee System Communication)
  - [x] Shooter Heat Regulation (Requires Referee System Communication)
  - [x] Level Up Adjustments (Requires Referee System Communication)
