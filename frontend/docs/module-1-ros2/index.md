---
sidebar_position: 2
title: "Module 1: Introduction to ROS 2"
---

# Module 1: Introduction to ROS 2

## Overview of Robot Operating System 2

Welcome to Module 1 of our comprehensive course on Physical AI and Humanoid Robotics. In this module, we'll explore the Robot Operating System 2 (ROS 2), which serves as the foundational middleware for developing complex robotic applications. ROS 2 is not an operating system in the traditional sense, but rather a collection of tools, libraries, and conventions that provide a framework for building robotic systems.

ROS 2 has become the de facto standard for robotic development due to its modular architecture, extensive ecosystem, and strong community support. It enables developers to create complex robotic applications by combining pre-built components, sharing data between processes, and managing the complexities of distributed robotic systems.

The importance of ROS 2 in humanoid robotics cannot be overstated. It provides the communication backbone that allows different components of a humanoid robot—sensors, controllers, perception systems, and AI modules—to work together seamlessly. Understanding ROS 2 is essential for anyone working in humanoid robotics, as it forms the foundation upon which more complex systems are built.

## Historical Context: From ROS 1 to ROS 2

To fully appreciate ROS 2, it's important to understand its predecessor, ROS 1. The original Robot Operating System (ROS) was developed in 2007 by Willow Garage and quickly became popular in academic and research environments. ROS 1 introduced many concepts that are still fundamental to ROS 2, including the node-based architecture, message passing, and the package management system.

However, as robotic applications became more complex and moved from research labs to real-world deployment, several limitations of ROS 1 became apparent:

- Lack of real-time support
- No security features
- Single-point-of-failure with the master node
- Limited support for multi-robot systems
- Difficulty in cross-platform deployment

ROS 2 was developed to address these limitations while maintaining the core concepts that made ROS 1 successful. The transition from ROS 1 to ROS 2 involved a complete rewrite of the underlying communication layer, incorporating modern technologies and best practices.

## Core Architecture of ROS 2

The architecture of ROS 2 is built around several key concepts that work together to provide a flexible and robust framework for robotic development:

### Nodes
Nodes are the fundamental building blocks of any ROS 2 system. A node is a process that performs computation and communicates with other nodes. In a humanoid robot, different nodes might handle sensor data processing, motor control, perception, planning, and other functions. Nodes are typically implemented as individual programs that can run on the same machine or distributed across multiple machines.

### Topics and Messages
Topics provide a publish-subscribe communication mechanism between nodes. A node publishes data to a topic, and other nodes subscribe to that topic to receive the data. This decouples the publisher from the subscriber, allowing for flexible system design. Messages are the data structures that are passed between nodes via topics. ROS 2 provides a rich set of standard message types and allows users to define custom message types.

### Services
Services provide a request-response communication pattern between nodes. A client node sends a request to a service, and a server node processes the request and returns a response. Services are useful for operations that require immediate feedback or for operations that should only be performed by a single node.

### Actions
Actions are similar to services but are designed for long-running operations. They provide feedback during execution and can be canceled if needed. Actions are particularly useful for navigation, manipulation, and other tasks that take time to complete.

### Parameters
Parameters provide a way for nodes to configure their behavior at runtime. Parameters can be set when launching a node or changed dynamically during execution. This allows for flexible system configuration without requiring code changes.

## Installation and Setup

Before diving into ROS 2 development, you'll need to install the appropriate ROS 2 distribution. For this course, we recommend using ROS 2 Humble Hawksbill, which is an LTS (Long Term Support) distribution that provides stability and long-term maintenance.

The installation process varies depending on your operating system. For Ubuntu systems, the process typically involves adding the ROS 2 repository, installing the packages, and setting up the environment. Similar processes exist for other operating systems including Windows and macOS.

After installation, it's important to source the ROS 2 setup script in your shell environment. This script sets up the necessary environment variables and makes ROS 2 commands available in your terminal.

## Creating Your First ROS 2 Package

A ROS 2 package is a container for related functionality. Packages contain source code, configuration files, launch files, and other resources needed for a particular feature or application. Creating a package is the first step in developing any ROS 2 application.

The `ros2 pkg create` command is used to create a new package. This command generates the basic directory structure and configuration files needed for a package. When creating a package, you'll specify the build type (CMake for C++ packages, ament_python for Python packages), dependencies, and other metadata.

A typical package structure includes:
- `CMakeLists.txt` or `setup.py` for build configuration
- `package.xml` for package metadata
- `src/` directory for source code
- `include/` directory for header files (C++)
- `launch/` directory for launch files
- `config/` directory for configuration files
- `test/` directory for unit tests

## Understanding the DDS Communication Layer

ROS 2 uses Data Distribution Service (DDS) as its underlying communication middleware. DDS is a standard for real-time, distributed data exchange that provides quality of service (QoS) controls, reliability, and other features important for robotic applications.

Understanding DDS is important for optimizing ROS 2 performance and troubleshooting communication issues. DDS provides several QoS policies that control how data is transmitted between nodes:

- Reliability: Controls whether all messages must be delivered (RELIABLE) or if some loss is acceptable (BEST_EFFORT)
- Durability: Controls whether late-joining subscribers receive previously published messages (TRANSIENT_LOCAL) or only new messages (VOLATILE)
- History: Controls how many messages are stored for delivery to subscribers
- Deadline: Specifies the maximum time between consecutive messages
- Lifespan: Specifies how long messages are considered valid

## Quality of Service (QoS) Profiles

QoS profiles allow you to fine-tune the communication behavior between nodes. Different types of data have different requirements for reliability, latency, and other factors. For example, sensor data might use BEST_EFFORT reliability with a small history depth, while critical control commands might use RELIABLE delivery with TRANSIENT_LOCAL durability.

ROS 2 provides several predefined QoS profiles that are appropriate for common use cases:
- `sensor_data_qos`: Optimized for sensor data with BEST_EFFORT reliability
- `services_qos`: Optimized for service calls with RELIABLE delivery
- `parameters_qos`: Optimized for parameter updates
- `default_qos`: A balanced profile for general use

## Launch Files and System Management

Launch files allow you to start multiple nodes with a single command and configure their parameters. This is essential for complex robotic systems where dozens of nodes need to be started and configured in a coordinated manner.

Launch files are written in Python and provide a powerful system for managing complex robotic applications. They support conditional launching, parameter passing, remapping, and other advanced features. Launch files can include other launch files, allowing for modular system composition.

## Testing and Debugging in ROS 2

ROS 2 provides extensive tools for testing and debugging robotic applications. The `rclpy` and `rclcpp` client libraries include built-in support for Google Test and other testing frameworks. Unit tests, integration tests, and system tests can all be implemented using ROS 2's testing infrastructure.

Debugging tools include `rqt`, a Qt-based GUI for visualizing and controlling ROS 2 systems, and `ros2 topic`, `ros2 service`, and other command-line tools for inspecting system state. The ROS 2 ecosystem also includes tools for performance analysis, memory debugging, and other specialized debugging tasks.

## ROS 2 in Humanoid Robotics Context

In the context of humanoid robotics, ROS 2 provides the communication infrastructure that allows different subsystems to work together. A typical humanoid robot might have nodes for:

- Sensor processing (IMU, cameras, force/torque sensors)
- Motor control and feedback
- Perception and object recognition
- Path planning and navigation
- High-level behavior control
- Human-robot interaction

These nodes communicate through ROS 2 topics, services, and actions to create a cohesive robotic system. The modular nature of ROS 2 allows different teams to work on different subsystems independently while ensuring they can be integrated effectively.

## Security Considerations

Security is a critical concern for modern robotic systems, especially those operating in human environments. ROS 2 includes built-in security features based on DDS Security, including authentication, access control, and encryption.

Understanding security configuration is important for deploying robots in real-world environments. Security policies can be configured to meet specific requirements for different applications, from research environments to commercial deployments.

## Performance Optimization

Performance is crucial for real-time robotic applications. ROS 2 provides several mechanisms for optimizing performance:

- Intra-process communication for nodes that run in the same process
- Shared memory for high-bandwidth data transfer
- Custom memory allocators for reducing allocation overhead
- Real-time scheduling support for time-critical applications

Understanding these optimization techniques is important for developing responsive and efficient robotic systems.

## Integration with Simulation

ROS 2 integrates seamlessly with simulation environments like Gazebo, allowing for safe and efficient development of robotic applications. Simulation nodes can publish the same message types as real sensors, allowing the same processing code to work in both simulation and reality.

This simulation integration is crucial for humanoid robotics development, where testing on real robots can be expensive and potentially dangerous during development phases.

## ROS 2 Ecosystem and Tools

The ROS 2 ecosystem includes numerous tools and packages that extend its functionality:

- `rviz2`: A 3D visualization tool for displaying sensor data, robot models, and other information
- `rosbag2`: Tools for recording and playing back ROS 2 messages for analysis and testing
- `ros2_control`: A framework for hardware abstraction and control
- `navigation2`: A complete navigation stack for mobile robots
- `moveit2`: Motion planning for manipulator robots

These tools provide building blocks that can be used to develop complex robotic applications more efficiently.

## Best Practices and Design Patterns

Developing effective ROS 2 applications requires understanding best practices and design patterns:

- Node design: Keep nodes focused on single responsibilities
- Message design: Use appropriate data types and consider bandwidth requirements
- Parameter management: Use parameters for configuration rather than hardcoding values
- Error handling: Implement robust error handling and recovery mechanisms
- Testing: Write comprehensive tests for all components
- Documentation: Maintain clear documentation for all interfaces

## Practical Exercise: Creating a Simple Publisher-Subscriber System

Let's implement a simple ROS 2 system to demonstrate the core concepts. We'll create a publisher node that publishes sensor data and a subscriber node that processes this data.

First, create a new package:
```bash
ros2 pkg create --build-type ament_python sensor_demo
```

In the package, create a publisher node that publishes temperature data:
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')
        self.publisher = self.create_publisher(Float32, 'temperature', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Float32()
        msg.data = 20.0 + random.uniform(-5.0, 5.0)  # Simulated temperature with noise
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    temperature_publisher = TemperaturePublisher()
    rclpy.spin(temperature_publisher)
    temperature_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Create a subscriber node that processes the temperature data:
```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class TemperatureSubscriber(Node):
    def __init__(self):
        super().__init__('temperature_subscriber')
        self.subscription = self.create_subscription(
            Float32,
            'temperature',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        if msg.data > 22.0:
            self.get_logger().warn(f'High temperature detected: {msg.data}')
        elif msg.data < 18.0:
            self.get_logger().warn(f'Low temperature detected: {msg.data}')
        else:
            self.get_logger().info(f'Normal temperature: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    temperature_subscriber = TemperatureSubscriber()
    rclpy.spin(temperature_subscriber)
    temperature_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Advanced Topics in ROS 2

As you become more proficient with ROS 2, you'll encounter advanced topics that are important for complex robotic systems:

### Custom Message Types
While ROS 2 provides many standard message types, you'll often need to define custom messages for your specific application. Custom messages are defined using the `.msg` file format and are automatically converted to language-specific data structures.

### Lifecycle Nodes
Lifecycle nodes provide a standardized way to manage the state of complex nodes. They support states like unconfigured, inactive, active, and finalized, with defined transitions between states. This is particularly useful for nodes that need to initialize hardware or load large data sets.

### Composition
ROS 2 supports node composition, where multiple nodes can run in the same process to reduce communication overhead. This is useful for performance-critical applications where the overhead of inter-process communication is too high.

### Real-time Support
For applications with strict timing requirements, ROS 2 provides support for real-time execution. This includes real-time safe allocators, lock-free data structures, and integration with real-time operating systems.

## ROS 2 and AI Integration

Modern humanoid robots increasingly incorporate AI and machine learning components. ROS 2 provides interfaces for integrating these components, including support for TensorFlow, PyTorch, and other ML frameworks.

AI nodes can subscribe to sensor data, perform inference, and publish results that can be used by other nodes in the system. This integration enables capabilities like object recognition, natural language processing, and decision-making.

## Troubleshooting Common Issues

As you work with ROS 2, you'll encounter common issues that can be resolved with proper understanding:

- Network configuration problems
- Time synchronization issues
- Memory management in long-running systems
- Performance bottlenecks in high-bandwidth applications
- Debugging distributed systems

## Future Developments

ROS 2 continues to evolve with new features and improvements. Ongoing development includes better real-time support, enhanced security features, improved tooling, and expanded platform support.

The ROS 2 community is also working on standardizing interfaces for common robotic capabilities, making it easier to develop reusable components and integrate different robotic systems.

## Summary

In this module, we've covered the fundamentals of ROS 2, including its architecture, core concepts, and practical applications in humanoid robotics. You've learned about nodes, topics, services, actions, and parameters, and you've seen how to create and manage ROS 2 packages.

The skills you've developed in this module form the foundation for all subsequent work in humanoid robotics. The modular, communication-based approach of ROS 2 enables the development of complex, distributed robotic systems that can integrate multiple sensors, actuators, and AI components.

In the next module, we'll explore simulation environments that allow you to test and develop your ROS 2 applications in a safe, controlled environment before deploying them on real hardware.

## Exercises

1. Create a ROS 2 package that implements a simple publisher-subscriber system for a sensor of your choice.
2. Implement a service that performs a calculation based on multiple sensor inputs.
3. Create a launch file that starts multiple nodes with appropriate parameters.
4. Use rosbag2 to record and replay a simple data stream.
5. Implement a node that uses parameters for configuration and can be reconfigured at runtime.

[Continue to Week 1: ROS 2 Architecture Overview](../../docs/weeks/week-01-02.md)

## References and Further Reading

- ROS 2 Documentation: https://docs.ros.org/en/humble/
- DDS Specification: https://www.omg.org/spec/DDS/
- ROS 2 Design Papers: Available through the Open Robotics website
- Academic papers on ROS 2 architecture and performance