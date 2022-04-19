# GimbalProject
Stabilization techniques and tools for  cameras are frequently sought by videographers looking to obtain fluid footage when in motion. The development and design of a three-axis electronic phone gimbal is explored in this paper as per the request of the University of Alberta Student Photography Club (UASPC). Specific requirements for the gimbal outlined by the UASPC include a method to turn on and off the gimbal, implementation of a control system for each axis, a method to manually set a focus point, availability of cinematic modes that execute preset movements, a method to discern battery percentage  and implementation of safety redundant systems.

The underlying software design of the gimbal is based around a real time operating system (RTOS). Using the STM32CubeIDE, the freeRTOS framework was utilized for the development of this project.

## Hardware:
- IMU: BNO055 (https://github.com/ivyknob/bno055_stm32)
- 3x HiTEC HS-65HB Servomotors


## DEMO

https://user-images.githubusercontent.com/43630890/163904313-8b588629-8b4d-4844-ad1a-d163da9f6cf5.mov

