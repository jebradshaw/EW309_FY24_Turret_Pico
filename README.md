# EW309_FY24_Turret_Pico
RP2040 Pico printed circuit board for EW309 Azimuth-Elevation Nerf Gun Turret Assembly

![20231130_080210](https://github.com/jebradshaw/EW309_FY24_Turret_Pico/assets/5246863/947c87b8-b7a2-4f77-9d6c-0e61a3c891c7)
Picture of the populated printed circuit board above

This project uses a RasPi RP2040 Pico based microcontroller to control a Nerf Gun on a yaw/pitch turret for the Weapons, Robotics, and Control Engineering Departrment EW309 course at the US Naval Academy. The board has two motor control ports that each provide a digital PWM output, with two direction contorl signals, power, and ground. These are used to interface to an L298 H-Bridge motor driver (or other compatible H-Bridge motor driver) to control the dual axis turret. There are also two MOSFET drivers for powering the motors that drive the spinners to project the ball on the nerf gun, and the belt driven feed motor that moves the nerf balls through the spinners to project them. Position data on the gun/turret is provided by a BNO-055 inertial measurement unit (IMU) which is capable of providing yaw and pitch angles directly as Euler angles in radians or degrees.

![EW309_Pico_Turret_Schematic_20240103](https://github.com/jebradshaw/EW309_FY24_Turret_Pico/assets/5246863/366907c0-4db7-420e-ab9e-d32864da6967)
  Schematic Above


![ExpressPCB_EW309_Turret_PCB_Top](https://github.com/jebradshaw/EW309_FY24_Turret_Pico/assets/5246863/7a3f0db3-1952-401e-a6c0-6a8dff6b6018)
  Top RP2040 Pico Printed Circuit Board snapshot
  
![ExpressPCB_EW309_Turret_PCB_Bottom](https://github.com/jebradshaw/EW309_FY24_Turret_Pico/assets/5246863/140b176e-a45b-4b63-a66c-3db24994a390)
  Bottom RP2040 Pico Printed Circuit Board snapshot
  
