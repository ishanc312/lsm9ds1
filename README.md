<div align=center>
<pre>
  888      .d8888b.  888b     d888  .d8888b.  8888888b.   .d8888b.   d888   
888     d88P  Y88b 8888b   d8888 d88P  Y88b 888  "Y88b d88P  Y88b d8888   
888     Y88b.      88888b.d88888 888    888 888    888 Y88b.        888   
888      "Y888b.   888Y88888P888 Y88b. d888 888    888  "Y888b.     888   
888         "Y88b. 888 Y888P 888  "Y888P888 888    888     "Y88b.   888   
888           "888 888  Y8P  888        888 888    888       "888   888   
888     Y88b  d88P 888   "   888 Y88b  d88P 888  .d88P Y88b  d88P   888   
88888888 "Y8888P"  888       888  "Y8888P"  8888888P"   "Y8888P"  8888888 
</pre>
</div>

## Overview
Test firmware written to interface with the LSM9DS1, a 9-Axis IMU (Inertial Mass Unit) connected to a STM32 Nucleo L432KCU3 and mounted on the Bruin Formula Mk. 10 Vehicle. 
- A calibration routine in which digital offsets/slopes are recorded and stored in the FLASH can be found in `calibration.h` and `calibration.c`.
- CAN Transmission to our data bus is integrated in the branch `accel_only_with_can`
- A rudimentary implementation for CAN Transmission of data from both this IMU and the NEO-M8P-2 GPS Module can be found in the branch `imu_with_gps`
