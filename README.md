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

## Conceptual Working of the System

The core idea of this project is to convert **hand motion into vehicle motion** using an embedded sensing and control approach.

Human hand movement is not directly usable by a microcontroller. It must first be:
1. Sensed
2. Interpreted
3. Converted into meaningful control signals
4. Transmitted wirelessly
5. Actuated using motors

This project implements all five stages.

---

## Role of the IMU (MPU6050) in Motion Detection

The MPU6050 is a motion sensor that continuously measures:
- Linear acceleration along X, Y, and Z axes
- Angular velocity around X, Y, and Z axes

When the hand is tilted:
- The **direction of gravity relative to the sensor changes**
- The **accelerometer values change accordingly**
- The **gyroscope measures how fast the hand is rotating**

These changes together describe the **orientation of the hand in space**.

Instead of detecting gestures as discrete events, this project uses **continuous orientation tracking**, which allows smooth and proportional control.

---

## Interpreting Hand Orientation

Two orientation parameters are primarily used:
- **Pitch**: forward/backward tilt of the hand
- **Roll**: left/right tilt of the hand

These two angles are sufficient to control a differential-drive vehicle:
- Pitch controls forward and backward motion
- Roll controls turning direction

The system does not rely on absolute position or complex gesture patterns.  
Only **relative tilt** is used, which keeps the control intuitive and stable.

---

## Why Raw Sensor Data Is Not Used Directly

Raw IMU data is:
- Noisy
- Sensitive to vibration
- Unstable for control purposes

If raw values were used directly:
- The RC car would jitter
- Small hand tremors would cause unintended motion
- Control would feel unstable

To address this, the project processes sensor data before using it for control.

---

## Sensor Data Stabilization Approach

The project applies multiple stabilization steps:

1. **Multiple Sampling**
   - Several IMU readings are taken in each loop cycle
   - This reduces the effect of sudden spikes

2. **Low-Pass Filtering**
   - High-frequency noise is suppressed
   - Slow, intentional hand movements are preserved

3. **Averaging**
   - Filtered values are averaged to obtain stable inputs

This layered approach improves signal quality without introducing complex algorithms.

---

## Combining Accelerometer and Gyroscope Information

The accelerometer and gyroscope each have limitations:
- Accelerometer data is stable but noisy
- Gyroscope data is smooth but drifts over time

To balance these characteristics, the system combines both:
- Gyroscope data is used for short-term smooth motion
- Accelerometer data corrects long-term drift

This combination allows the system to maintain:
- Stability
- Responsiveness
- Control accuracy

The result is a reliable estimate of hand orientation suitable for real-time control.

---

## Mapping Hand Motion to Control Signals

Once orientation is estimated:
- Tilt angles are limited to a safe range
- These angles are converted into proportional control values

Instead of simple ON/OFF control:
- Small tilts result in slow motion
- Larger tilts increase speed

This creates an **analog-style control experience**, similar to a joystick but using hand motion.

---

## Wireless Command Transmission Logic

The transmitter does not send raw sensor data.  
Instead, it sends **processed control values**.

This design choice:
- Reduces wireless bandwidth usage
- Simplifies receiver-side logic
- Makes the system more robust

Each transmitted packet represents the **current intended motion of the vehicle**, not sensor readings.

---

## Receiver-Side Interpretation

The receiver continuously listens for incoming control packets.

Upon receiving data:
- Forward/backward and left/right values are separated
- These values are combined to calculate individual motor speeds

The receiver does not perform sensor processing.  
Its responsibility is limited to:
- Interpreting commands
- Driving motors accordingly

This separation of responsibilities improves modularity.

---

## Vehicle Motion Control Strategy

The RC car uses a differential drive mechanism:
- Two independent motors control left and right wheels

By varying motor speed and direction:
- Forward motion
- Reverse motion
- Turning
- Smooth curves

can all be achieved without steering mechanisms.

---

## Real-Time Behavior of the System

The system operates in a continuous loop:
- Hand motion is constantly sensed
- Control signals are updated in real time
- Wireless commands are sent repeatedly

This creates the perception of:
- Immediate response
- Smooth control
- Natural interaction

There is no predefined sequence or automation.  
The vehicle responds purely based on **current hand orientation**.

---

## Design Philosophy

This project prioritizes:
- Simplicity
- Understandability
- Practical embedded concepts

Advanced techniques were intentionally avoided where simpler approaches were sufficient.  
The focus was on building a **complete working system**, not optimizing individual blocks in isolation.

---

## Summary of System Flow

1. Hand is tilted
2. IMU senses motion
3. Sensor data is stabilized
4. Orientation is estimated
5. Control values are generated
6. Data is transmitted wirelessly
7. Receiver interprets commands
8. Motors move the vehicle

This completes one control cycle, repeated continuously during operation.

