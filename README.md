Firmware for a quadcopter, running on an ESP32-C6 using the ESP-IDF framework, freeRTOS

Overview of classes:

PID.c:
Generic PID implementation

PWM.c:
- Includes some stable flight loop control using PID class and a modeSelector

Server.c:
- Takes keystroke inputs and sends packets back
- Packets are run through a command handler, the PWM modeSelector variable changes
- Displays important values

Log.c:
- Custom log library that accepts "log elements" from other tasks and outputs them inline with its own task

MPU6050.c:
- Generic MPU6050 driver
