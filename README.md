# Chat2Drive: Smart Transport Prototype

## Project Overview
Smart transportation is an evolving field, but most existing projects focus on large-scale autonomous driving (e.g., Tesla). This project explores smart transport for short distances, developing a prototype capable of safely carrying loads to a set destination.

This prototype is represented as a **smart car**, but the system can be adapted for smart carts and other transport devices. The vehicle integrates GPS navigation, real-time user control via WiFi, and obstacle detection to achieve autonomous and semi-autonomous transport.

## Functional Definition
The transport prototype is built on a **4-wheel DC motor-powered system**, connected to a GPS module and a **WiFi-enabled control board**. Users can control the prototype using the **Telegram messaging app**, which sends commands to an Arduino via the **Universal Telegram Bot Library**. The car also features a **distance sensor system** to detect and avoid obstacles.

## Hardware Components
### Sensors
- **Ultrasonic Sensors (HC-SR04)**: Detect obstacles and calculate distances.
- **QMC5883L Triple-Axis Compass Magnetometer**: Determines the car’s direction using Earth's magnetic field.
- **NEO-6M GPS Module**: Provides real-time location tracking.

### Mechanical and Electrical Systems
- **Chassis Kit**: Includes the frame and **four DC motors**.
- **Arduino Mega Board**: The main microcontroller.
- **ESP8266 WiFi Board (Wemos D1 Mini)**: Enables wireless communication with the user.
- **BTS7960 Motor Driver Modules**: Controls motor movement precisely.
- **Power Supply**: A 9V battery powers the Arduino, while a separate 5.5V supply powers the motors.
- **Level Shifter**: Ensures proper voltage communication between the ESP8266 and Arduino Mega.

## Software Implementation
The project uses several **Arduino libraries** for sensor calibration, GPS integration, and communication:
- **QMC5883L Compass Driver**: Calibrates and reads heading direction.
- **TinyGPSPlus & Adafruit GPS**: Handles GPS data parsing.
- **NewPing**: Converts ultrasonic sensor readings to distances.
- **ESP8266-TelegramBot**: Receives user commands from Telegram.

### User Control
1. Users send directional commands (e.g., "Left", "Right") via Telegram.
2. The ESP8266 reads and transmits these commands to the Arduino Mega.
3. The Arduino processes user inputs and adjusts movement accordingly.

### Autonomous Navigation
- The vehicle moves from **waypoint to waypoint** based on **predefined GPS coordinates**.
- Uses the **compass sensor** for heading direction adjustments.
- Ultrasonic sensors detect and avoid obstacles dynamically.

## Testing and Performance
The project was tested in multiple environments, including **indoor spaces and outdoor locations like Revelle Plaza**. Key testing phases:
1. **Individual sensor performance** (GPS, compass, motor drivers, WiFi connection).
2. **Integrated system testing** with hardcoded GPS waypoints.
3. **User input testing** for seamless control via Telegram.

### Challenges Encountered
- **Motor Power Issues**: The car sometimes struggled on rough surfaces.
- **GPS Signal Reception**: The NEO-6M GPS module took up to **30 minutes** to lock onto satellites.
- **User Input Delay**: Telegram command execution experienced minor lags.
- **Library Maintenance Issues**: Some community libraries required fixes or modifications.

## Future Improvements
- Implementing **real-time GPS-based rerouting** instead of fixed waypoints.
- Enhancing motor power to handle uneven terrain.
- Integrating **Google Maps API** for more dynamic user input.
- Exploring **sensor fusion** for improved obstacle detection and navigation.

## Project Deliverables
- **Codebase**: Includes programs for autonomous driving, Telegram-based control, and sensor integration.
- **Hardware Schematics**: Pin connections and wiring diagrams.
- **Demonstration Video**: Showcases the smart car navigating autonomously.

## Shopping List & Estimated Cost
| Component | Quantity | Price (USD) | Notes |
|-----------|----------|------------|------|
| QMC5883L Compass Sensor | 1 | $12 | Reusable |
| NEO-6M GPS Module | 1 | $13 | Reusable |
| ESP8266 WiFi Board | 1 | $10 | Reusable |
| BTS7960 Motor Driver | 2 | $12 each | Reusable |
| Smart Car Chassis Kit | 1 | $20 | Includes DC motors |
| Lithium Batteries | 4AA | Varies | |

**Total Estimated Cost:** ~$60

## Conclusion
This project successfully demonstrates a **low-cost, smart transport prototype** with basic autonomous navigation and remote user control. While challenges remain in **power management and GPS accuracy**, the system lays the groundwork for further enhancements in **precision navigation and real-world application**.

---
For more details, check the full **Project Report** or explore the **code on GitHub**!

