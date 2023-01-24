# EW309_FY24_Turret_Pico
RP2040 Pico printed circuit board for EW309 Azimuth-Elevation Nerf Gun Turret Assembly

![Capture](https://user-images.githubusercontent.com/5246863/214314373-920b3747-fa5b-4128-b171-6c80cf0abbd8.PNG)
Picture of the populated printed circuit board above

This project uses a RasPi RP2040 Pico based microcontroller to control a Nerf Gun on a yaw/pitch turret for the Weapons, Robotics, and Control Engineering Departrment EW309 course at the US Naval Academy. The board has two motor control ports that each provide a digital PWM output, with two direction contorl signals, power, and ground. These are used to interface to an L298 H-Bridge motor driver (or other compatible H-Bridge motor driver) to control the dual axis turret. There are also two MOSFET drivers for powering the motors that drive the spinners to project the ball on the nerf gun, and the belt driven feed motor that moves the nerf balls through the spinners to project them. Position data on the gun/turret is provided by a BNO-055 inertial measurement unit (IMU) which is capable of providing yaw and pitch angles directly as Euler angles in radians or degrees.

![EW309_Pico_Turret_Schematic_20230124](https://user-images.githubusercontent.com/5246863/214342758-4d78a506-b367-4cef-b39b-55aba5aec4f1.png)
  Schematic Above


![ExpressPCB_snapshot_Top_pico](https://user-images.githubusercontent.com/5246863/214318634-cd5d8f31-4db1-49f6-95f0-2aa96f9cf7c7.JPG)
  Top RP2040 Pico Printed Circuit Board snapshot
  
![ExpressPCB_snapshot_Bottom_pico](https://user-images.githubusercontent.com/5246863/214318661-e23cb401-6d22-4628-9a5b-887b278809f9.JPG)
  Bottom RP2040 Pico Printed Circuit Board snapshot
  
