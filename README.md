# Gesture Controlled RC Car (Embedded Systems Project)

This project is a gesture-controlled RC car implemented using microcontrollers, an IMU sensor, and wireless communication.  
Hand movements are captured using an MPU6050 (GY-521) sensor and translated into motion commands for the vehicle.

The project focuses on embedded firmware development, sensor data processing, real-time control, and wireless communication between two microcontroller units.

This is a hardware-based prototype developed for learning and experimentation.

---

## Project Overview

The system is divided into two main parts:

1. **Transmitter Unit (Controller)**
   - Worn or held by the user
   - Reads hand orientation using an IMU sensor
   - Processes sensor data to estimate pitch and roll angles
   - Sends control commands wirelessly

2. **Receiver Unit (RC Car)**
   - Mounted on the vehicle
   - Receives wireless control data
   - Controls motor direction and speed accordingly

The motion of the RC car depends on the orientation of the user’s hand:
- Tilting forward/backward → forward or reverse motion
- Tilting left/right → turning motion

---

## Hardware Used

### Transmitter Side
- ESP32
- MPU6050 (GY-521 IMU)
- NRF24L01 wireless module
- I2C and SPI interfaces

### Receiver Side
- Arduino Uno / ESP32
- NRF24L01 wireless module
- Motor driver (L298N or similar)
- DC motors and RC car chassis

---

## Wireless Communication

Wireless communication between the transmitter and receiver is implemented using the **NRF24L01** module.

A simple data structure is used to transmit control information:

```cpp
struct DataPacket {
  int power_f_b;  // Forward / Backward control
  int power_l_r;  // Left / Right control
};
