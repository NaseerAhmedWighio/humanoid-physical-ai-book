---
sidebar_position: 2
title: "Appendix B: Hardware Specifications"
---

# Appendix B: Hardware Specifications

This appendix provides detailed information about the hardware platforms referenced in this course, including specifications, capabilities, and limitations. Understanding hardware constraints and capabilities is crucial for developing effective robotic systems that can operate reliably in real-world environments.

## Computing Platforms

### NVIDIA Jetson Series
The NVIDIA Jetson platform is widely used for AI-powered robotics due to its high performance per watt and GPU acceleration capabilities.

#### Jetson AGX Orin
- **GPU**: 2048-core NVIDIA Ampere architecture GPU with 64 Tensor Cores
- **CPU**: 12-core ARM v8.2 64-bit CPU
- **Memory**: 32GB LPDDR5 memory
- **Performance**: Up to 275 TOPS AI performance
- **Power**: 15W to 60W configurable
- **Use Case**: High-performance humanoid robotics with complex AI models

#### Jetson Orin NX
- **GPU**: 1024-core NVIDIA Ampere architecture GPU with 32 Tensor Cores
- **CPU**: 8-core ARM v8.2 64-bit CPU
- **Memory**: 8GB LPDDR4x memory
- **Performance**: Up to 100 TOPS AI performance
- **Power**: 10W to 25W configurable
- **Use Case**: Mid-range humanoid robotics applications

#### Jetson Nano
- **GPU**: 128-core NVIDIA Maxwell architecture GPU
- **CPU**: Quad-core ARM A57 CPU
- **Memory**: 4GB LPDDR4 memory
- **Performance**: 0.5 TOPS AI performance
- **Power**: 5W to 15W configurable
- **Use Case**: Educational and prototype humanoid robots

### Intel-Based Platforms

#### Intel NUC (Next Unit of Computing)
- **CPU**: Intel Core i3/i5/i7/i9 processors
- **Memory**: Up to 64GB DDR4 RAM
- **GPU**: Integrated Intel Iris Xe or discrete NVIDIA/AMD GPU
- **Connectivity**: Multiple USB ports, HDMI, Ethernet
- **Use Case**: Simulation and development environments

#### Raspberry Pi 4
- **CPU**: Broadcom BCM2711, Quad core Cortex-A72 (ARM v8) 64-bit SoC @ 1.5GHz
- **Memory**: 2GB, 4GB, or 8GB LPDDR4-3200 SDRAM
- **Connectivity**: 2× USB 3.0, 2× USB 2.0, Gigabit Ethernet
- **Power**: 5V DC via USB-C connector, 5.1W power over Ethernet
- **Use Case**: Low-cost robotic controllers and sensor processing

## Actuator Systems

### High-Performance Servo Motors

#### Dynamixel Series (Robotis)
- **XL-320**
  - Torque: 0.39 N·m at 12V
  - Speed: 77 rev/min
  - Communication: TTL asynchronous serial
  - Feedback: Position, temperature, load, input voltage
  - Weight: 15.6g

- **XM430-W350-T**
  - Torque: 3.5 N·m (with reduction)
  - Speed: 4.8 rev/sec
  - Communication: RS485 with protocol 2.0
  - Feedback: Position, velocity, current, temperature
  - Weight: 155g

- **XH540-W270-T**
  - Torque: 7.6 N·m (with reduction)
  - Speed: 7.8 rev/sec
  - Communication: RS485 with protocol 2.0
  - Feedback: Position, velocity, current, temperature, voltage
  - Weight: 204g

#### Herkulex Series
- High-resolution position control (12-bit encoder)
- Built-in PID control
- Multiple communication protocols
- Higher torque-to-weight ratio than standard servos

### Brushless DC Motors with Gearboxes

#### Maxon EC-i Series
- **EC-i 40**
  - Power: 50W continuous, 150W peak
  - Torque: 0.163 N·m continuous
  - Speed: up to 11,700 rpm
  - Encoder: 4096 counts/revolution absolute encoder
  - Hall sensors for commutation

#### Faulhaber Series
- Various frame sizes (12mm to 43mm)
- Gear ratios from 4:1 to 1541:1
- Encoder resolutions up to 16384 counts/revolution
- Integrated controllers available

## Sensor Systems

### Vision Sensors

#### RGB-D Cameras
- **Intel RealSense D435**
  - Depth Technology: Stereo vision
  - Depth Range: 0.2m to 10m
  - Resolution: 1280×720 at 30/60/90 FPS
  - RGB Resolution: 1920×1080 at 30 FPS
  - Field of View: 87° × 58° × 103°
  - Connectivity: USB 3.0

- **Intel RealSense D435i**
  - Includes IMU (gyroscope and accelerometer)
  - Enhanced for SLAM applications
  - Same specifications as D435 plus IMU data

#### LIDAR Sensors
- **Hokuyo UAM-05LP**
  - Range: 0.06m to 5.0m
  - Angular Resolution: 0.25°
  - Scan Rate: 25Hz
  - Points per Scan: 1081
  - Interface: Ethernet

- **SICK TiM571**
  - Range: 0.05m to 10.0m
  - Angular Resolution: 0.33° to 1.0°
  - Scan Rate: 15.625Hz
  - Points per Scan: 541
  - Protection: IP65/IP67

### Inertial Measurement Units (IMUs)

#### High-End IMUs
- **VectorNav VN-300**
  - Accelerometer: ±6g, 100μg stability
  - Gyroscope: ±2000°/s, 0.0035°/s stability
  - Magnetometer: ±2.5Gauss
  - Temperature: -40°C to +85°C
  - Interface: RS232/RS422, USB, Ethernet

#### Medium-Range IMUs
- **MTI-300 series (Xsens)**
  - Accelerometer: ±160m/s² range
  - Gyroscope: ±2000°/s range
  - Magnetometer: ±2.5Gauss range
  - Sample Rate: Up to 2000Hz
  - Interface: RS232, USB, CAN, Ethernet

#### Low-Cost IMUs
- **MPU-6050 (InvenSense)**
  - 3-axis gyroscope and accelerometer
  - Digital motion processor
  - I2C interface
  - Low power consumption
  - Common in educational robots

### Force/Torque Sensors

#### ATI Multi-Axis Force/Torque Sensors
- **Mini40 Series**
  - 6-axis force/torque measurement
  - Capacity: Various models from 10N to 200N (force), 1Nm to 20Nm (torque)
  - Resolution: &lt;0.001% of capacity
  - Bandwidth: DC to 3kHz
  - Interface: USB, CAN, Ethernet

#### Single-Axis Load Cells
- **Futek LTH Series**
  - Tension and compression
  - Capacity: 50N to 5000N
  - Non-linearity: ±0.1% of rated output
  - Operating Temperature: -10°C to +60°C

## Humanoid Robot Platforms

### Research Platforms

#### Boston Dynamics Atlas
- **Height**: 5'9" (175 cm)
- **Weight**: 180 lbs (82 kg)
- **Degrees of Freedom**: 28
- **Actuation**: Hydraulic and electric
- **Sensors**: Stereo vision, LIDAR, IMU
- **Computing**: Custom control computer
- **Power**: 24V lithium-ion battery system

#### Honda ASIMO (Legacy)
- **Height**: 4'3" (130 cm)
- **Weight**: 119 lbs (54 kg)
- **Degrees of Freedom**: 57
- **Actuation**: Electric servomotors
- **Sensors**: Vision, audio, force/torque, position, tactile
- **Speed**: 1.6 km/h walking, 6 km/h running

### Educational Platforms

#### NAO (SoftBank Robotics)
- **Height**: 58 cm
- **Weight**: 5.2 kg
- **Degrees of Freedom**: 25
- **Actuation**: 25 servo motors
- **Sensors**: 2 HD cameras, 4 microphones, 2 speakers, 18 tactile sensors, 9 accelerometers, 2 gyrometers, 1 sonar, 2 infrared emitters/receivers
- **Computing**: Intel Atom Z3795, 2GB RAM, 32GB SSD
- **Connectivity**: WiFi, Ethernet, Bluetooth

#### Pepper (SoftBank Robotics)
- **Height**: 120 cm
- **Weight**: 28 kg
- **Degrees of Freedom**: 20
- **Actuation**: 20 motors for mobility and upper body
- **Sensors**: 3D sensor, 2 cameras, 3 microphones, 1 speaker, 1 touch sensor, 1 sonar, 1 gyrometer, 1 accelerometer
- **Computing**: Intel Atom, 2GB RAM, 2GB Flash Memory
- **Connectivity**: WiFi, 3G (optional)

### Open Source Platforms

#### Darwin OP
- **Height**: 46 cm
- **Weight**: 2.9 kg
- **Degrees of Freedom**: 20
- **Actuation**: 20 Dynamixel AX-12A servos
- **Sensors**: 2 cameras, IMU, 2 microphones, 2 speakers
- **Computing**: Intel Atom Z530, 1GB RAM
- **Software**: ROS support

#### Poppy Humanoid
- **Height**: 1m
- **Weight**: 2.5 kg
- **Degrees of Freedom**: 25 (varies by configuration)
- **Actuation**: Dynamixel servos
- **Sensors**: Cameras, IMU (optional)
- **Computing**: Raspberry Pi or similar SBC
- **Open Source**: All designs and code available

## Communication Interfaces

### Network Interfaces
- **Ethernet**: 100Mbps/1Gbps for high-bandwidth data
- **WiFi**: 802.11n/ac for wireless communication
- **Bluetooth**: For short-range communication with devices
- **CAN Bus**: For real-time, deterministic communication

### Serial Interfaces
- **UART/RS232**: For legacy devices and sensors
- **RS485**: For multi-drop communication with multiple devices
- **SPI/I2C**: For communication with embedded sensors

## Power Systems

### Battery Technologies
- **Li-ion (Lithium-ion)**: High energy density, 3.7V per cell
- **LiPo (Lithium Polymer)**: Flexible form factor, 3.7V per cell
- **NiMH (Nickel Metal Hydride)**: Lower energy density but safer

### Power Management
- **Voltage Regulators**: Step-down/step-up for different voltage requirements
- **Power Distribution**: Proper wiring and connectors for high-current applications
- **Battery Management Systems (BMS)**: For safe charging and monitoring

## Selection Guidelines

### For High-Performance Applications
- Use high-torque actuators with high-resolution encoders
- Select computing platforms with GPU acceleration
- Implement redundant sensor systems for safety
- Consider custom hardware for specific applications

### For Educational Applications
- Prioritize ease of use and programming
- Select platforms with good documentation and community support
- Consider cost-effectiveness for multiple units
- Ensure safety features are in place

### For Research Applications
- Select modular platforms that can be easily modified
- Prioritize sensor accuracy and communication bandwidth
- Consider power consumption for mobile platforms
- Ensure real-time performance capabilities

## Integration Considerations

### Mechanical Integration
- Proper mounting and vibration isolation
- Cable management and strain relief
- Thermal management for high-power components
- Weight distribution for balance

### Electrical Integration
- Proper grounding and EMI/RFI considerations
- Power distribution and voltage regulation
- Signal integrity for high-speed communication
- Safety interlocks and emergency stops

### Software Integration
- Real-time operating system requirements
- Middleware compatibility (ROS/ROS 2)
- Driver availability and maintenance
- Debugging and monitoring capabilities

## Future Trends

### Emerging Technologies
- **Edge AI Chips**: Specialized processors for AI inference
- **New Actuator Technologies**: Series elastic actuators, variable stiffness actuators
- **Advanced Sensors**: Event-based cameras, solid-state LIDAR
- **Wireless Power**: For improved mobility

### Performance Improvements
- Higher power density in smaller form factors
- Improved energy efficiency
- Better integration between components
- Enhanced safety and reliability

Understanding these hardware specifications and capabilities will enable you to make informed decisions when designing and implementing humanoid robotic systems. The choice of hardware components significantly impacts the performance, capabilities, and limitations of your robotic system.