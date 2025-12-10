---
sidebar_position: 5
title: "Week 3: Advanced ROS 2 Concepts"
---

# Week 3: Advanced ROS 2 Concepts

## Introduction

Welcome to Week 3 of our comprehensive course on Physical AI and Humanoid Robotics. In this week, we'll dive deeper into advanced ROS 2 concepts that are essential for developing sophisticated humanoid robotic systems. Building on the foundational knowledge from Weeks 1-2, we'll explore complex topics including custom message types, lifecycle nodes, composition, and real-time considerations.

This week focuses on practical skills needed for real-world robotic development, with hands-on exercises that will prepare you for more complex system integration challenges. We'll also begin to explore how these advanced concepts apply specifically to humanoid robotics applications.

## Custom Message Types

While ROS 2 provides many standard message types, complex humanoid systems often require custom message definitions to represent specialized data. Creating custom messages involves several steps:

### Defining Custom Messages

Custom messages are defined using the `.msg` file format. Here's an example of a custom message for humanoid joint state:

```
# HumanoidJointState.msg
std_msgs/Header header
string[] joint_names
float64[] positions
float64[] velocities
float64[] efforts
float64[] torques
float64[] commanded_positions
float64[] commanded_velocities
float64[] commanded_efforts
```

This message type extends the standard JointState message to include commanded values, which are important for humanoid control systems where you need to track both current and desired states.

### Creating the Message Package

To create custom messages, you need to:

1. Create a package specifically for messages (e.g., `humanoid_msgs`)
2. Create a `msg/` directory in the package
3. Define your `.msg` files in this directory
4. Update `package.xml` to include message generation dependencies
5. Update `CMakeLists.txt` to include message generation

```xml
<!-- In package.xml -->
<buildtool_depend>ament_cmake</buildtool_depend>
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

```cmake
# In CMakeLists.txt
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HumanoidJointState.msg"
  "msg/BalanceCommand.msg"
  "msg/StepPlan.msg"
  DEPENDENCIES std_msgs builtin_interfaces
)
```

### Using Custom Messages

Once generated, custom messages can be used in nodes just like standard messages:

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from humanoid_msgs.msg import HumanoidJointState
from sensor_msgs.msg import JointState

class JointStateAggregator(Node):
    def __init__(self):
        super().__init__('joint_state_aggregator')

        # Subscribe to standard joint states
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # Publish custom humanoid joint state
        self.humanoid_pub = self.create_publisher(
            HumanoidJointState,
            '/humanoid_joint_states',
            10
        )

        # Store commanded states (from controller)
        self.commanded_positions = {}
        self.commanded_velocities = {}
        self.commanded_efforts = {}

        # Timer for aggregating data
        self.timer = self.create_timer(0.01, self.publish_aggregated_state)

    def joint_callback(self, msg):
        """Update commanded states from controller commands"""
        # This would typically come from separate command topics
        # For this example, we'll simulate commanded values
        pass

    def publish_aggregated_state(self):
        """Publish aggregated joint state with commanded values"""
        if not hasattr(self, 'last_joint_state'):
            return

        # Create custom message
        humanoid_msg = HumanoidJointState()
        humanoid_msg.header.stamp = self.get_clock().now().to_msg()
        humanoid_msg.header.frame_id = 'base_link'

        # Copy standard joint state data
        humanoid_msg.joint_names = self.last_joint_state.name
        humanoid_msg.positions = self.last_joint_state.position
        humanoid_msg.velocities = self.last_joint_state.velocity
        humanoid_msg.efforts = self.last_joint_state.effort

        # Add commanded values (simplified for example)
        humanoid_msg.commanded_positions = self.last_joint_state.position  # In real system, these would be different
        humanoid_msg.commanded_velocities = self.last_joint_state.velocity
        humanoid_msg.commanded_efforts = self.last_joint_state.effort

        self.humanoid_pub.publish(humanoid_msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateAggregator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Lifecycle Nodes

Lifecycle nodes provide a standardized way to manage the state of complex nodes. They support states like unconfigured, inactive, active, and finalized, with defined transitions between states. This is particularly useful for humanoid robots where nodes may need to initialize hardware or load large datasets.

### Lifecycle Node States

- **Unconfigured**: Node has been created but not configured
- **Inactive**: Node is configured but not running
- **Active**: Node is running and processing data
- **Finalized**: Node has been shut down and cleaned up

### Implementing a Lifecycle Node

```python
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.lifecycle import Publisher
from std_msgs.msg import String

class HumanoidController(LifecycleNode):
    def __init__(self):
        super().__init__('humanoid_controller')
        self.get_logger().info('Creating HumanoidController')

        # Initialize publishers and subscribers (but don't activate them yet)
        self.pub = None
        self.sub = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure the node - create publishers/subscribers but don't activate them"""
        self.get_logger().info('Configuring HumanoidController')

        # Create publisher
        self.pub = self.create_publisher(String, 'controller_status', 10)

        # Create subscriber
        self.sub = self.create_subscription(
            String,
            'controller_commands',
            self.command_callback,
            10
        )

        # Initialize hardware or load parameters
        self.initialize_hardware()

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate the node - start processing data"""
        self.get_logger().info('Activating HumanoidController')

        # Activate publishers and subscribers
        self.pub.on_activate()

        # Start control loops
        self.control_timer = self.create_timer(0.01, self.control_loop)

        # Publish activation status
        status_msg = String()
        status_msg.data = 'CONTROLLER_ACTIVE'
        self.pub.publish(status_msg)

        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate the node - stop processing data"""
        self.get_logger().info('Deactivating HumanoidController')

        # Deactivate publishers
        self.pub.on_deactivate()

        # Stop timers
        self.control_timer.destroy()

        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Clean up the node - destroy publishers/subscribers"""
        self.get_logger().info('Cleaning up HumanoidController')

        # Destroy publishers and subscribers
        self.destroy_publisher(self.pub)
        self.destroy_subscription(self.sub)

        # Clean up hardware
        self.cleanup_hardware()

        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Shutdown the node"""
        self.get_logger().info('Shutting down HumanoidController')
        return TransitionCallbackReturn.SUCCESS

    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Handle errors"""
        self.get_logger().error('Error in HumanoidController')
        return TransitionCallbackReturn.FAILURE

    def initialize_hardware(self):
        """Initialize hardware components"""
        self.get_logger().info('Initializing humanoid hardware...')
        # In a real system, this would initialize actuators, sensors, etc.
        pass

    def cleanup_hardware(self):
        """Clean up hardware components"""
        self.get_logger().info('Cleaning up humanoid hardware...')
        # In a real system, this would safely shut down hardware
        pass

    def command_callback(self, msg):
        """Handle incoming commands"""
        self.get_logger().info(f'Received command: {msg.data}')
        # Process command and update control state

    def control_loop(self):
        """Main control loop"""
        # Implement control logic here
        self.get_logger().debug('Control loop executing...')

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidController()

    # Transition through lifecycle states
    node.trigger_configure()
    node.trigger_activate()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.trigger_deactivate()
        node.trigger_cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Node Composition

Node composition allows multiple nodes to run in the same process to reduce communication overhead. This is particularly useful for performance-critical humanoid applications where inter-process communication latency is too high.

### Creating a Composite Node

```python
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import Float64
import threading

class CompositeHumanoidNode:
    def __init__(self):
        # Initialize rclpy
        rclpy.init()

        # Create the executor
        self.executor = SingleThreadedExecutor()

        # Create multiple nodes in the same process
        self.sensor_processor = SensorProcessor()
        self.controller = BalanceController()
        self.trajectory_planner = TrajectoryPlanner()

        # Add nodes to executor
        self.executor.add_node(self.sensor_processor)
        self.executor.add_node(self.controller)
        self.executor.add_node(self.trajectory_planner)

        # Set up inter-node communication
        self.setup_internal_communication()

    def setup_internal_communication(self):
        """Set up communication between nodes without ROS middleware overhead"""
        # Use shared memory or direct function calls
        self.controller.set_sensor_callback(self.sensor_processor.get_sensor_data)
        self.trajectory_planner.set_controller_callback(self.controller.send_command)

    def spin(self):
        """Run the composite node"""
        try:
            self.executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()

    def shutdown(self):
        """Clean shutdown of all nodes"""
        self.sensor_processor.destroy_node()
        self.controller.destroy_node()
        self.trajectory_planner.destroy_node()
        rclpy.shutdown()

class SensorProcessor(Node):
    def __init__(self):
        super().__init__('sensor_processor')

        # Publishers for processed sensor data
        self.imu_pub = self.create_publisher(Float64, 'imu_processed', 10)
        self.encoder_pub = self.create_publisher(Float64, 'encoder_processed', 10)

        # Subscribers for raw sensor data
        self.imu_sub = self.create_subscription(Float64, 'imu_raw', self.imu_callback, 10)
        self.encoder_sub = self.create_subscription(Float64, 'encoder_raw', self.encoder_callback, 10)

        # Internal state
        self.processed_imu_data = 0.0
        self.processed_encoder_data = 0.0

        # Processing timer
        self.process_timer = self.create_timer(0.001, self.process_sensors)

    def imu_callback(self, msg):
        # Process raw IMU data
        self.processed_imu_data = self.filter_imu_data(msg.data)

    def encoder_callback(self, msg):
        # Process raw encoder data
        self.processed_encoder_data = self.filter_encoder_data(msg.data)

    def process_sensors(self):
        # Publish processed data
        imu_msg = Float64()
        imu_msg.data = self.processed_imu_data
        self.imu_pub.publish(imu_msg)

        encoder_msg = Float64()
        encoder_msg.data = self.processed_encoder_data
        self.encoder_pub.publish(encoder_msg)

    def get_sensor_data(self):
        """Get processed sensor data for internal communication"""
        return {
            'imu': self.processed_imu_data,
            'encoder': self.processed_encoder_data
        }

    def filter_imu_data(self, raw_data):
        # Implement IMU filtering (e.g., complementary filter)
        return raw_data  # Simplified for example

    def filter_encoder_data(self, raw_data):
        # Implement encoder filtering
        return raw_data  # Simplified for example

class BalanceController(Node):
    def __init__(self):
        super().__init__('balance_controller')

        # Publishers for control commands
        self.left_leg_pub = self.create_publisher(Float64, 'left_leg_command', 10)
        self.right_leg_pub = self.create_publisher(Float64, 'right_leg_command', 10)

        # Subscribers for sensor data
        self.sensor_sub = self.create_subscription(Float64, 'sensor_fusion', self.sensor_callback, 10)

        # Controller parameters
        self.kp = 2.0
        self.kd = 0.5
        self.target_angle = 0.0

        # Control timer
        self.control_timer = self.create_timer(0.005, self.control_loop)

        # Internal state
        self.current_angle = 0.0
        self.current_angular_velocity = 0.0
        self.command = 0.0

        # For internal communication
        self.sensor_callback_func = None

    def set_sensor_callback(self, callback_func):
        """Set callback for internal sensor data"""
        self.sensor_callback_func = callback_func

    def sensor_callback(self, msg):
        """Process sensor data from ROS topic"""
        # This would be used when not using internal communication
        pass

    def control_loop(self):
        """Main balance control loop"""
        if self.sensor_callback_func:
            # Use internal communication
            sensor_data = self.sensor_callback_func()
            self.current_angle = sensor_data['imu']
            self.current_angular_velocity = sensor_data.get('angular_velocity', 0.0)

        # Simple PD controller
        error = self.target_angle - self.current_angle
        error_derivative = self.current_angular_velocity  # Approximate derivative

        self.command = self.kp * error - self.kd * error_derivative

        # Apply limits
        self.command = max(-1.0, min(1.0, self.command))

        # Publish commands
        left_msg = Float64()
        left_msg.data = self.command
        right_msg = Float64()
        right_msg.data = self.command

        self.left_leg_pub.publish(left_msg)
        self.right_leg_pub.publish(right_msg)

class TrajectoryPlanner(Node):
    def __init__(self):
        super().__init__('trajectory_planner')

        # Publishers for trajectory commands
        self.trajectory_pub = self.create_publisher(Float64, 'trajectory', 10)

        # Subscribers for high-level commands
        self.command_sub = self.create_subscription(Float64, 'high_level_command', self.command_callback, 10)

        # Trajectory generation timer
        self.trajectory_timer = self.create_timer(0.1, self.generate_trajectory)

        # Internal state
        self.current_trajectory = 0.0
        self.target_position = 0.0
        self.controller_callback = None

    def set_controller_callback(self, callback_func):
        """Set callback for sending commands to controller"""
        self.controller_callback = callback_func

    def command_callback(self, msg):
        """Receive high-level movement commands"""
        self.target_position = msg.data

    def generate_trajectory(self):
        """Generate smooth trajectory to target position"""
        # Simple linear interpolation for example
        if abs(self.current_trajectory - self.target_position) > 0.01:
            # Move toward target
            direction = 1 if self.target_position > self.current_trajectory else -1
            self.current_trajectory += 0.01 * direction

            # Publish trajectory
            traj_msg = Float64()
            traj_msg.data = self.current_trajectory
            self.trajectory_pub.publish(traj_msg)

            # Send to controller if available
            if self.controller_callback:
                self.controller_callback(self.current_trajectory)

def main():
    composite_node = CompositeHumanoidNode()
    composite_node.spin()

if __name__ == '__main__':
    main()
```

## Real-Time Considerations

For humanoid robots, real-time performance is often critical for safety and stability. ROS 2 provides several features to support real-time operation:

### Real-Time Scheduling

```python
import rclpy
from rclpy.node import Node
import os
import ctypes
from ctypes import wintypes  # On Windows, or import from appropriate module

class RealTimeController(Node):
    def __init__(self):
        super().__init__('real_time_controller')

        # Set up real-time parameters
        self.setup_real_time()

        # Create high-priority timer for critical control
        self.critical_timer = self.create_timer(
            0.001,  # 1ms period for critical control
            self.critical_control_loop,
            clock=self.get_clock()
        )

        # Create lower-priority timer for non-critical tasks
        self.non_critical_timer = self.create_timer(
            0.1,  # 100ms period for monitoring
            self.non_critical_loop
        )

    def setup_real_time(self):
        """Configure real-time settings"""
        # On Linux systems, you might configure CPU affinity and scheduling
        try:
            # Set CPU affinity to dedicate CPU core for critical tasks
            # This is system-specific and may require root privileges
            import os
            import sched
            # os.sched_setaffinity(0, {0})  # Pin to CPU 0 (example)

            # Set real-time scheduling policy (requires appropriate privileges)
            # sched.scheduler.RR or SCHED_FIFO

        except Exception as e:
            self.get_logger().warn(f'Could not set real-time parameters: {e}')

    def critical_control_loop(self):
        """Time-critical control loop - must execute within deadline"""
        # This loop must execute with predictable timing
        # Avoid dynamic memory allocation, locks, or blocking operations

        start_time = self.get_clock().now()

        # Critical control logic here
        self.execute_balance_control()

        end_time = self.get_clock().now()
        execution_time = (end_time.nanoseconds - start_time.nanoseconds) / 1e9

        # Log timing for monitoring
        if execution_time > 0.0009:  # 90% of 1ms deadline
            self.get_logger().warn(f'Critical loop exceeded timing: {execution_time:.6f}s')

    def non_critical_loop(self):
        """Non-critical monitoring and logging loop"""
        # This loop can have variable timing
        # Safe to do logging, diagnostics, and non-critical tasks

        # Publish diagnostic information
        self.publish_diagnostics()

    def execute_balance_control(self):
        """Execute critical balance control algorithm"""
        # Implement your balance control here
        # This should be a deterministic, bounded-time algorithm
        pass

    def publish_diagnostics(self):
        """Publish diagnostic information"""
        # This is non-critical and can take variable time
        pass
```

## Quality of Service (QoS) Optimization

Different types of data in humanoid robots have different QoS requirements:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class QoSConfiguredNode(Node):
    def __init__(self):
        super().__init__('qos_configured_node')

        # Critical control commands - RELIABLE, TRANSIENT_LOCAL
        control_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST
        )

        self.control_pub = self.create_publisher(Float64, 'critical_control', control_qos)

        # High-frequency sensor data - BEST_EFFORT, VOLATILE
        sensor_qos = QoSProfile(
            depth=1,  # Only keep most recent
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST
        )

        self.sensor_sub = self.create_subscription(
            JointState, 'joint_states', self.sensor_callback, sensor_qos
        )

        # Configuration parameters - RELIABLE, TRANSIENT_LOCAL
        param_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_ALL
        )

        self.param_pub = self.create_publisher(Float64, 'robot_parameters', param_qos)

    def sensor_callback(self, msg):
        """Handle sensor data with appropriate QoS"""
        # Process sensor data
        pass
```

## Advanced Launch File Concepts

Launch files can include complex conditional logic and parameter management:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

def generate_launch_description():
    # Declare launch arguments
    sim_mode_arg = DeclareLaunchArgument(
        'simulation_mode',
        default_value='false',
        description='Run in simulation mode'
    )

    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='humanoid_robot',
        description='Name of the robot'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='/path/to/default/config.yaml',
        description='Path to configuration file'
    )

    def launch_setup(context, *args, **kwargs):
        """Function to set up launch configuration based on arguments"""
        sim_mode = LaunchConfiguration('simulation_mode').perform(context)
        robot_name = LaunchConfiguration('robot_name').perform(context)
        config_file = LaunchConfiguration('config_file').perform(context)

        # Load configuration from file
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)

        nodes_to_launch = []

        # Add nodes based on configuration
        if sim_mode.lower() == 'true':
            # Add simulation-specific nodes
            gazebo_node = Node(
                package='gazebo_ros',
                executable='spawn_entity.py',
                arguments=[
                    '-entity', robot_name,
                    '-file', config.get('robot_model_path', ''),
                    '-x', str(config.get('spawn_x', 0.0)),
                    '-y', str(config.get('spawn_y', 0.0)),
                    '-z', str(config.get('spawn_z', 0.0))
                ],
                output='screen'
            )
            nodes_to_launch.append(gazebo_node)

        # Add robot controller node
        controller_node = Node(
            package='humanoid_control',
            executable='controller_node',
            name='controller_node',
            parameters=[
                config.get('controller_config', {}),
                {'robot_name': robot_name}
            ],
            output='screen'
        )
        nodes_to_launch.append(controller_node)

        # Add sensor processing node
        sensor_node = Node(
            package='humanoid_sensors',
            executable='sensor_processor',
            name='sensor_processor',
            parameters=[
                config.get('sensor_config', {}),
                {'simulation_mode': sim_mode}
            ],
            output='screen'
        )
        nodes_to_launch.append(sensor_node)

        # Conditionally add debugging tools
        debug_mode = config.get('debug_mode', False)
        if debug_mode:
            debug_node = Node(
                package='humanoid_debug',
                executable='debug_visualizer',
                name='debug_visualizer',
                parameters=[{'robot_name': robot_name}],
                output='screen'
            )
            nodes_to_launch.append(debug_node)

        return nodes_to_launch

    return LaunchDescription([
        sim_mode_arg,
        robot_name_arg,
        config_file_arg,
        OpaqueFunction(function=launch_setup)
    ])
```

## Performance Profiling and Optimization

Understanding and optimizing performance is crucial for humanoid robots:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time
from collections import deque
import statistics

class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')

        # Publishers for performance metrics
        self.latency_pub = self.create_publisher(Float64, 'performance/latency', 10)
        self.cpu_usage_pub = self.create_publisher(Float64, 'performance/cpu', 10)

        # Performance tracking
        self.processing_times = deque(maxlen=100)  # Keep last 100 measurements
        self.message_timestamps = {}

        # Performance monitoring timer
        self.monitor_timer = self.create_timer(1.0, self.publish_performance_metrics)

        # Message processing timer
        self.process_timer = self.create_timer(0.01, self.process_simulation_step)

    def process_simulation_step(self):
        """Simulate a processing step and measure performance"""
        start_time = time.perf_counter()

        # Simulate some processing work
        self.simulate_computation()

        end_time = time.perf_counter()
        processing_time = end_time - start_time

        # Store processing time
        self.processing_times.append(processing_time)

        # Log if processing time is too high
        if processing_time > 0.005:  # 5ms threshold
            self.get_logger().warn(f'High processing time: {processing_time:.6f}s')

    def simulate_computation(self):
        """Simulate computational work"""
        # Simulate some realistic computation
        result = 0
        for i in range(1000):
            result += i * 0.001
        return result

    def publish_performance_metrics(self):
        """Publish performance metrics"""
        if self.processing_times:
            avg_processing_time = statistics.mean(self.processing_times)
            max_processing_time = max(self.processing_times)
            min_processing_time = min(self.processing_times)

            # Publish average latency
            latency_msg = Float64()
            latency_msg.data = avg_processing_time
            self.latency_pub.publish(latency_msg)

            self.get_logger().info(
                f'Performance - Avg: {avg_processing_time:.6f}s, '
                f'Max: {max_processing_time:.6f}s, '
                f'Min: {min_processing_time:.6f}s'
            )

def main(args=None):
    rclpy.init(args=args)
    node = PerformanceMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Integration with Humanoid-Specific Systems

Humanoid robots have unique requirements that require specialized ROS 2 configurations:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Pose, Twist
from humanoid_msgs.msg import BalanceState, StepPlan  # Custom message
import numpy as np

class HumanoidIntegrationNode(Node):
    def __init__(self):
        super().__init__('humanoid_integration')

        # Subscribe to various sensor inputs
        self.joint_state_sub = self.create_subscription(
            JointState, 'joint_states', self.joint_state_callback, 10
        )

        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.imu_callback, 10
        )

        # Subscribe to force/torque sensors if available
        self.wrench_sub = self.create_subscription(
            # Custom wrench message for feet
            'left_foot/wrench', self.left_foot_wrench_callback, 10
        )

        # Publishers for humanoid-specific control
        self.balance_cmd_pub = self.create_publisher(BalanceState, 'balance_command', 10)
        self.step_plan_pub = self.create_publisher(StepPlan, 'step_plan', 10)
        self.walk_cmd_pub = self.create_publisher(Twist, 'walking_command', 10)

        # Internal state for humanoid-specific processing
        self.current_joint_positions = {}
        self.current_imu_orientation = np.array([0.0, 0.0, 0.0, 1.0])  # Quaternion
        self.left_foot_force = np.array([0.0, 0.0, 0.0])
        self.right_foot_force = np.array([0.0, 0.0, 0.0])

        # Humanoid-specific processing timer
        self.humanoid_timer = self.create_timer(0.01, self.humanoid_processing_loop)

        self.get_logger().info('Humanoid Integration Node initialized')

    def joint_state_callback(self, msg):
        """Process joint state messages"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joint_positions[name] = msg.position[i]

    def imu_callback(self, msg):
        """Process IMU data for balance control"""
        self.current_imu_orientation = np.array([
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ])

    def left_foot_wrench_callback(self, msg):
        """Process left foot force/torque data"""
        self.left_foot_force = np.array([
            msg.wrench.force.x,
            msg.wrench.force.y,
            msg.wrench.force.z
        ])

    def humanoid_processing_loop(self):
        """Main humanoid-specific processing loop"""
        # Calculate center of mass
        com = self.calculate_center_of_mass()

        # Calculate zero moment point
        zmp = self.calculate_zmp(com)

        # Determine balance state
        balance_state = self.evaluate_balance_state(zmp)

        # Generate balance commands if needed
        if not balance_state['stable']:
            self.generate_balance_correction(balance_state)

        # Check for step planning opportunities
        self.evaluate_step_planning()

    def calculate_center_of_mass(self):
        """Calculate center of mass based on joint positions"""
        # Simplified calculation - in reality, this would use full kinematic model
        # and mass distribution data
        return np.array([0.0, 0.0, 0.8])  # Approximate CoM height for humanoid

    def calculate_zmp(self, com):
        """Calculate Zero Moment Point"""
        # Simplified ZMP calculation
        # In reality, this would use full dynamic model
        zmp_x = com[0] - (com[2] * 9.81) / (10.0**2)  # Simplified formula
        zmp_y = com[1] - (com[2] * 9.81) / (10.0**2)
        return np.array([zmp_x, zmp_y, 0.0])

    def evaluate_balance_state(self, zmp):
        """Evaluate current balance state"""
        # Define support polygon (simplified as rectangle around feet)
        support_margin = 0.1  # 10cm margin

        # Simplified support polygon based on foot positions
        left_foot_x = self.current_joint_positions.get('left_foot_x', 0.0)
        left_foot_y = self.current_joint_positions.get('left_foot_y', 0.1)
        right_foot_x = self.current_joint_positions.get('right_foot_x', 0.0)
        right_foot_y = self.current_joint_positions.get('right_foot_y', -0.1)

        # Calculate support polygon bounds
        min_x = min(left_foot_x, right_foot_x) - support_margin
        max_x = max(left_foot_x, right_foot_x) + support_margin
        min_y = min(left_foot_y, right_foot_y) - support_margin
        max_y = max(left_foot_y, right_foot_y) + support_margin

        # Check if ZMP is within support polygon
        stable = (min_x <= zmp[0] <= max_x) and (min_y <= zmp[1] <= max_y)

        return {
            'stable': stable,
            'zmp': zmp,
            'support_polygon': (min_x, max_x, min_y, max_y),
            'margin': support_margin
        }

    def generate_balance_correction(self, balance_state):
        """Generate balance correction commands"""
        balance_cmd = BalanceState()
        balance_cmd.header.stamp = self.get_clock().now().to_msg()
        balance_cmd.header.frame_id = 'base_link'

        # Calculate correction needed
        zmp_error = balance_state['zmp'][:2]  # X, Y components

        # Simple proportional control for balance correction
        kp_balance = 1.0
        balance_cmd.correction_x = -kp_balance * zmp_error[0]
        balance_cmd.correction_y = -kp_balance * zmp_error[1]

        # Set balance state
        balance_cmd.stable = balance_state['stable']

        self.balance_cmd_pub.publish(balance_cmd)

    def evaluate_step_planning(self):
        """Evaluate if step planning is needed"""
        # Check if current support polygon is insufficient
        # This would involve more complex logic in a real system
        pass

def main(args=None):
    rclpy.init(args=args)
    node = HumanoidIntegrationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Week 3 Summary and Exercises

### Summary

In Week 3, you've learned advanced ROS 2 concepts that are essential for developing sophisticated humanoid robotic systems:

1. **Custom Message Types**: How to define and use custom message types for specialized humanoid data
2. **Lifecycle Nodes**: Managing node states for reliable humanoid robot operation
3. **Node Composition**: Reducing communication overhead for performance-critical applications
4. **Real-Time Considerations**: Ensuring predictable timing for safety-critical control
5. **QoS Optimization**: Configuring communication for different types of data
6. **Advanced Launch Files**: Managing complex system configurations
7. **Performance Monitoring**: Tracking and optimizing system performance
8. **Humanoid Integration**: Specialized techniques for humanoid robot systems

### Exercises for Week 3

1. Create custom message types for your humanoid robot's specific sensor and control data.

2. Implement a lifecycle node that manages the startup and shutdown of a humanoid robot's systems.

3. Create a composite node that combines multiple processing units for reduced latency.

4. Implement a real-time control loop with proper timing constraints and monitoring.

5. Configure QoS settings appropriately for different types of data in your humanoid system.

6. Create a complex launch file that handles both simulation and real robot deployment.

7. Implement performance monitoring for your humanoid robot's control systems.

8. Design a humanoid-specific integration node that processes multiple sensor modalities.

### Looking Ahead to Week 4

Week 4 will begin our exploration of simulation environments, starting with Gazebo fundamentals and how to create realistic humanoid robot simulations. You'll learn to build complex environments, configure physics parameters, and integrate simulation with your ROS 2 control systems.

[Continue to Module 2: Simulation with Gazebo and Unity](../module-2-simulation/index.md)

## References and Resources

- ROS 2 Lifecycle Nodes: https://design.ros2.org/articles/node_lifecycle.html
- ROS 2 Quality of Service: https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html
- Real-Time ROS 2: https://docs.ros.org/en/rolling/Tutorials/Real-Time-Linux.html
- Launch System: https://github.com/ros2/launch