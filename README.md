# **Stepper Motor Controller with CANbus and Encoder Feedback**

## **Overview**

This firmware controls stepper motors with **closed-loop position and speed control** using:

- **CANbus** for command and feedback exchange with an onboard computer.
- **AS5600 magnetic encoders** for position and velocity feedback.
- **PID control loops** for smooth position and speed regulation.
- **Accel-limited motion profiles** consistent with commercial stepper controllers.

The system supports **two stepper motors** and is suitable for robotics and automation applications.

---

## **Key Features**

### **1. CANbus Communication**
- Receives motor commands:
  - **Position Mode (`0xF5`)** → Target position, max velocity, and acceleration.
  - **Speed Mode (`0xF6`)** → Target velocity and acceleration.
  - **Calibration (`0x80`)** → Reserved for homing.
- Sends feedback:
  - **Absolute Position (`0x31`)** in signed 48-bit cumulative turns (`+0x4000` ticks per CW turn).
  - **Velocity (`0x32`)** in RPM (CCW positive, CW negative).
- Uses CRC for data integrity.

### **2. Encoder Feedback**
- Reads from **AS5600 magnetic encoders**.
- Tracks **multi-turn cumulative position** with wraparound detection.
- Calculates instantaneous **velocity** (degrees/sec).

### **3. PID-Controlled Motion**
- **Position Control** → PID outer loop generates a velocity demand.
- **Speed Control** → PID inner loop regulates RPM.
- **Acceleration limiting** consistent with commercial stepper controllers:
  - `acc` (0–255) → `(256 - acc) * 50 µs` between 1 RPM increments.

### **4. Multi-I2C Support**
- Uses:
  - **`Wire`** (I2C0) for Motor 0.
  - **`Wire1`** (I2C1) for Motor 1.
- Automatically assigns I2C bus based on motor ID.

### **5. Unit Testing & PID Tuning**
- Built-in unit tests for:
  - CAN command reception.
  - Encoder feedback transmission.
  - Motor driver PWM generation.
  - Full position/speed control loop.
- **Interactive PID tuning mode** via Serial terminal.

---

## **Code Structure**

### **1. `main.cpp`**
- Entry point and test harness.
- Runs all **unit tests**:
  - `test_CAN_receiver()` → Verifies CAN command reception.
  - `test_CAN_enc_feedback()` → Sends dummy feedback.
  - `test_MotorController()` → Tests PWM and direction toggling.
  - `test_StepperMotor_position()` → Runs a position control loop.
  - `test_StepperMotor_speed()` → Runs a speed control loop.
  - `test_system()` → Full workflow (receive → execute → feedback).
  - `tune_PID()` → Interactive PID tuning.

---

### **2. `CANbus` Class**
Handles CAN communication.

#### **Main Functions**
- **`begin()`** → Initializes CAN hardware.
- **`updateCommand()`** → Receives and validates frames; routes commands to motors.
- **`processFrame()`** → Decodes commands (position/speed/calibration).
- **`sendPosition()` / `sendSpeed()`** → Transmits feedback in datasheet format.
- **`computeCRC()`** → Shared CRC generation for sending/receiving.

#### **Static Members**
- `StepperMotor* motor[2];` → References to controlled motors.

---

### **3. `StepperMotor` Class**
Manages each motor’s state and PID control.

#### **Initialization**
- Selects correct **I2C bus** based on `id`:
  - `id=0 → Wire (SDA=21, SCL=22)`
  - `id=1 → Wire1 (SDA=25, SCL=26)`
- Initializes encoder and motor driver.

#### **Motion Control**
- **`setPositionCommand()` / `setSpeedCommand()`** → Updates targets.
- **`runMotor()`** → Main control loop (IDLE/POSITION/SPEED).
- **`runPositionControl()`** → Position PID + acceleration limiting.
- **`runSpeedControl()`** → Speed PID + acceleration limiting.

#### **Feedback Updates**
- **`updatePosition()`** → Tracks **multi-turn absolute position** (CW positive, CCW negative).
- **`updateVelocity()`** → Instantaneous velocity (deg/s).
- **`computeAccel()`** → Acceleration limiting per datasheet.

---

### **4. `MotorController` Class**
Low-level motor driver interface.

#### **Key Functions**
- **`setDirection()`** → CW (HIGH) / CCW (LOW).
- **`setPulseFrequency()`** → Updates step pulse frequency.
- **`setSpeed()`** → Adjusts PWM duty cycle.

---

## **Hardware Setup**

### **Stepper Motors**
- Motor 0:  
  - **PUL:** 23  
  - **DIR:** 19  
  - **EN:** 21  
- Motor 1:  
  - **Define accordingly in `main.cpp`.**

### **AS5600 Encoders**
- Motor 0: **SDA=21, SCL=22** (`Wire`)  
- Motor 1: **SDA=25, SCL=26** (`Wire1`)
