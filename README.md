# Inverted Pendulum

This project consists of an inverted pendulum control system using an MPU6050 sensor, an ESP32 microcontroller, and two DC motors. The goal is to maintain the pendulum in a vertical position by controlling the motor speeds.

[Video1](https://youtu.be/fJx0cJZ3p6E)
[Video2](https://youtu.be/VGf259euCRA)

## Operation

The code controls the inverted pendulum as follows:

1. **Sensor and Motor Initialization**: In the `setup()` function, serial communication is initialized, the MPU6050 sensor is initialized, and the motor pins are configured.

2. **MPU6050 Data Reading**: In the `loop()` loop, data from the MPU6050 accelerometer and gyroscope is read to calculate the pendulum's tilt angles in both the X and Y axes.

3. **Inverted Pendulum Control**: The error between the desired angle and the current pendulum angle is calculated. Then, a PID (Proportional-Integral-Derivative) controller is used to determine the motor speed needed to correct this error.

4. **Motor Control**: The motor speeds are adjusted according to the output of the PID controller to keep the pendulum in a vertical position.

5. **Data Visualization**: Relevant data, such as rotation around the X-axis, error, and motor speeds, are printed via serial communication for visualization on a serial monitor.

## Bill of Materials (BOM)

Table of materials used in the project.
| Component | Quantity |
|------------|----------|
| ESP32 Dev Board | 1 |
| MPU6050 Sensor | 1 |
| DC Motor | 2 |
| TB6612FNG Motor Driver | 1 |
| Power Switch | 2 |
| Electrolytic Capacitor 330uF 16V | 1 |
| Ceramic Capacitor 100nF | 2 |
| TO-220 L7805CV Voltage Regulator | 1 |
| JST-HX Female 2-pin Connector | 2 |
| JST-HX Male 2-pin Connector | 2 |
| Connection Wires | - |
| DC Motor Wheels | 2 |
| LiPo Battery 11.1V 1500mAh | 1 |
| XT60 Female Connector | 1 |
| PCB | 1 |
| Chassis | 1 |
| M3 8mm Screws | 4 |
| M3 25mm Screws | 4 |
| LiPo Straps | 2 |

## Adjustments and Customization
- It is important to calibrate the MPU6050 sensor for precise measurements. A detailed explanation of how to calibrate the sensor can be found [here](https://naylampmechatronics.com/blog/45_tutorial-mpu6050-acelerometro-y-giroscopio.html).
- The PID controller parameters (Kp, Ki, Kd) should be adjusted for the robot to remain stable. A guide for adjusting PID controller parameters can be found [here](https://www.luisllamas.es/como-ajustar-un-controlador-pid-en-arduino/).

## PCB
The PCB design was done in KiCad. The PCB manufacturing files are located in the `PCB` folder. The JLCPCB service was used for PCB manufacturing.