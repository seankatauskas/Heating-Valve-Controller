# Heating Valve Controller

This project demonstrates a Heating Valve Controller utilizing the Matter protocol and the ESP Matter data model. Written in C++ with C APIs from the ESP32 platform, it highlights my strong skills in embedded systems and end-to-end IoT development.

### Why This Project?

My apartment has a steam radiator with a manual valve requiring me to constantly turn it on and off to maintain an comfortable indoor temperature. After exploring available solutions, I found that most options required significant modifications to the existing heating system and weren't easy to install. To address this, I developed a solution that regulates heating in real-time, has a universal attachment, and works with all smart home platforms.

### Features

- Matter Protocol Integration: The controller is compatible with any Matter supported device.
- Servo Motor Control: The servo is halted with feedback from a current sensor when hitting physical resistance.
- Power Optimization: The code implements low power optimization for the ESP board and disconnects the motor's power source when not in use.
- 3D Prining: The design has an adjustable mounting bracket and features a motor shaft with an internal sliding rail to accommodate the vertical movement of rotating the handle.

## Project Overview

### 1. Hardware Components

-   ESP32-C6-DevKitC-1-N8 Development Board
-   HiTEC HSR-2648CR Continuous Servo
-   Tiny Breadboard
-   Assorted Jumper Wires
-   INA219 High Side DC Current Sensor Breakout
-   LM3671 3.3V Buck Converter Breakout
-   **2** 2x2 AA Battery Holder
-   **7** AA Batteries
-   3D Printed Custom Components

### 2. Installation & Building
   1. Hardware: Connect the components as follows in the connection diagram below.
   2. Software: Set up the ESP-IDF and ESP Matter environment and flash the firmware.
   3. Run the controller: Configure the controller with your Matter enabled device and set automations for the on/off switch to certain temperatures.

   For detailed instruction, visit the [ESP Matter Documentation](https://docs.espressif.com/projects/esp-matter/en/latest/esp32/developing.html). This project is based on the [light example](https://github.com/espressif/esp-matter/blob/42b6412/examples/light/README.md) in the esp-matter documentation.

### 3. Connection Diagram

```
               Power  Power
               (4.5V) (6V)    
                 |     |
+-------------+  |     |  +-------------------+         +---------------------------+ 
| LM3671 Buck |  |     |  |  INA219           |      <|-|  HiTEC HSR-2648CR         |
| Converter   |  |     |  |  Current Sensor   |         |  Continuous Servo         |
|             |  |     |  |                   |     +-->|                           |
|         Vin +<-+     +->+ VIN+          GND +-|>  |   |                           |
|             |           |                   |     |   +----|   |------------------+ 
|         GND +-|>  +---->+ VCC          VIN- +-----+        |   |     |           
|             |     |     |                   |             |||||||    |
|        3.3v +-----+   +-+ SCL           SCA |-+           -------    |
|             |     |   | |                   | |                      |
+-------------+     |   | +-------------------+ |                      |
                    |   +----+         +--------+                      |
                    v        v         v                               |
         +----------+--------+---------+--------+                      |
         |         3.3V    GPIO-3   GPIO-2      |                      |
         |                               GPIO-0 +<---------------------+
         |                                      |
         |                                  GND +-|>
         |                ESP32-C6              |
         +--------------------------------------+

```

## Blog & Deep Dive

Explore the technical aspects of the project in my blog:

- [Designing a Matter Heating Valve Controller: Challenges and Solutions](blog-link-section1)
- [Heating Valve Controller: Product Design](blog-link-section2)
- [Heating Valve Controller: Power Optimization](blog-link-section3)

These posts provide insights into design choices, challenges faced, and optimizations.
