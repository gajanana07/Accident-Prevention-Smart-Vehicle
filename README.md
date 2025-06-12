# 🚗 Accident Prevention via Auto-Braking Sensors in Smart Vehicle

An intelligent RC car-based prototype that prevents collisions using ultrasonic and infrared sensors, real-time feedback through LCD display, and Bluetooth-based control with auto-braking features. This project demonstrates scalable, cost-effective vehicle safety solutions ideal for academic and real-world applications.

## Table of Contents
- [About the Project](#about-the-project)
  - [Abstract](#abstract)
  - [Key Features](#key-features)
- [Getting Started](#getting-started)
  - [Prerequisites](#prerequisites)
  - [Installation](#installation)
- [System Architechture](#system-architechture)
  - [Block Diagram](#block-diagram)
  - [Circuit Diagram](#circuit-diagram)
- [Technologies Used](#technologies-used)
- [Screenshots](#screenshots)
- [Future Enhancements](#future-enhancements)
- [Team](#the-team)
## About the Project

### Abstract
This project introduces a smart vehicle system built on an RC car, controlled via an ESP32 microcontroller. It integrates ultrasonic and IR sensors to detect obstacles and blind spots. If an object is detected within 25 cm, the car slows down; if it's within 8 cm, the vehicle automatically halts, preventing a collision. Alerts and system status are displayed on an LCD screen, and commands are received through Bluetooth.

### Key Features
- **Obstacle Detection** using ultrasonic sensors.
- **Auto-Braking** based on proximity.
- **Blind Spot Monitoring** using IR sensors with real-time alerts.
- **Real-Time LCD Feedback** for speed, temperature, and warnings.
- **Bluetooth-Controlled Navigation** using a mobile app.
- **Modular and Scalable Design** for educational and automotive prototypes.
  
## Getting Started

### Prerequisites

Before you begin, ensure the following tools and components are installed or available:

- **Arduino IDE**: For writing and uploading the code to the ESP32.
- **ESP32 Board Configuration**: Add ESP32 support via the Boards Manager in Arduino IDE.
- **Required Libraries** (install via Library Manager):
  - `LiquidCrystal`
  - `BluetoothSerial`
  - `Ultrasonic`
- **Bluetooth Terminal App**: Install on your smartphone (e.g., Serial Bluetooth Terminal on Android) for sending commands.


### Installation

1. **Clone the Repository**

   Open a terminal and run:
   ```bash
   git clone https://github.com/gajanana07/Accident-Prevention-Smart-Vehicle.git
   cd Accident-Prevention-Smart-Vehicle
   ```

2. **Open Project in Arduino IDE**

- Launch Arduino IDE.
- Open the `.ino` file located inside the cloned repository folder.
- Go to:
  - Tools > Board > ESP32 Dev Module
  - Tools > Port and select the correct COM port for your ESP32 board.


3. **Connect and Configure Components**

- **Ultrasonic Sensors:**
  - Connect Trig pin to GPIO (e.g., D12).
  - Connect Echo pin to GPIO (e.g., D14).
- **IR Sensors:**
  - Connect to available digital GPIO pins (e.g., D26, D27).
- **LCD (16x2 Display):**
  - Use I2C or parallel interface depending on your module.
  - If using I2C, connect:
    - SDA to GPIO21
    - SCL to GPIO22
- **DC Motors:**
  - Connect to L298N Motor Driver.
  - Connect IN1, IN2, IN3, IN4 from L298N to ESP32 digital pins.
- **Power Supply:**
  - Use a 9V battery or 7.4V Li-ion pack.
  - Ensure stable power supply to ESP32 and motors.


4. **Upload the Code**

- In Arduino IDE, click the ✔️ Verify button to compile the code.
- Click the Upload button (right-arrow icon) to flash the code onto the ESP32.
- Wait for the "Done Uploading" message.
- Open Serial Monitor (optional) to debug or verify logs.


5. **Connect Bluetooth and Test**

- Enable Bluetooth on your smartphone.
- Pair with the ESP32 module (usually named something like `ESP32_BT`).
- Open a Bluetooth Terminal App on your phone (e.g., Serial Bluetooth Terminal for Android).
- Connect to the ESP32 and send the following commands:
  - `F100` → Move Forward at speed 100
  - `B50` → Move Backward at speed 50
  - `L` → Turn Left
  - `R` → Turn Right
  - `S` → Stop the vehicle


Once completed, your accident-prevention smart car is ready for real-time testing with obstacle detection, auto-braking, and blind-spot alert features.

## System Architechture

### Block Diagram

<div style="display: flex; justify-content: center; gap: 50px;align-items: center;">

<img src="https://github.com/gajanana07/Accident-Prevention-Smart-Vehicle/blob/main/Pictures/block-diagram.png" width="50%">

</div>



### Circuit Diagram

<div style="display: flex; justify-content: center; gap: 50px;align-items: center;">

<img src="https://github.com/gajanana07/Accident-Prevention-Smart-Vehicle/blob/main/Pictures/circuit-diagram.png" width="50%">

</div>

## Technologies Used

| Component             | Description                                           |
|-----------------------|-------------------------------------------------------|
| ESP32             | Microcontroller for processing and communication      |
| Ultrasonic Sensor | For front/rear obstacle detection                     |
| IR Sensors        | For blind spot detection (left & right)               |
| L298N Motor Driver| For motor control and speed regulation                |
| LCD (16x2)        | To show speed, temperature, and alerts                |
| Bluetooth Module  | Built-in ESP32 support for Bluetooth commands         |

## Screenshots
##  Future Enhancements

- Integration with Wi-Fi for remote monitoring and control.
- Implementation of voice command interface for hands-free operation.
- Upgrade to LiDAR or camera-based sensors for more accurate obstacle detection and 3D environment mapping.
- Addition of a touchscreen dashboard for better user interaction and control.
- Incorporation of cloud storage and analytics to log speed, distance, and sensor data for later analysis.
## The Team

**S Gajanana Nayak**

[![GitHub](https://img.shields.io/badge/GitHub-black?style=flat&logo=github)](https://github.com/gajanana07)

[![LinkedIn](https://img.shields.io/badge/LinkedIn-blue?style=flat&logo=linkedin)](https://www.linkedin.com/in/s-gajanana-nayak-b0854a29a/)

**Raghavendra R**

[![GitHub](https://img.shields.io/badge/GitHub-black?style=flat&logo=github)](https://github.com/RaghavendraCodes)

[![LinkedIn](https://img.shields.io/badge/LinkedIn-blue?style=flat&logo=linkedin)](https://www.linkedin.com/in/raghavendra-r-363701202/)

**Rahul N Raju**

[![GitHub](https://img.shields.io/badge/GitHub-black?style=flat&logo=github)](https://github.com/Rahul-891)

[![LinkedIn](https://img.shields.io/badge/LinkedIn-blue?style=flat&logo=linkedin)](https://www.linkedin.com/in/rahul-n-raju-ab6919247/)

**Revanth K**

[![GitHub](https://img.shields.io/badge/GitHub-black?style=flat&logo=github)](https://github.com/)
