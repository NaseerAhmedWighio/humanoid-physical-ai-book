---
sidebar_position: 5
title: "Appendix E: ROS 2 Message Types"
---

# Appendix E: ROS 2 Message Types

This appendix provides a comprehensive reference for ROS 2 message types commonly used in humanoid robotics applications. Understanding these message types is crucial for developing interoperable robotic systems and effective communication between nodes.

## Standard Message Packages

### std_msgs
Basic message types used across all ROS 2 packages.

#### Common Types
- `Bool` - Boolean value
- `Byte` - Single byte value
- `Char` - Single character
- `ColorRGBA` - RGBA color with alpha
- `Empty` - Empty message
- `Float32`, `Float64` - Floating point numbers
- `Int8`, `Int16`, `Int32`, `Int64` - Signed integers
- `UInt8`, `UInt16`, `UInt32`, `UInt64` - Unsigned integers
- `String` - Variable length string
- `Time` - Time value
- `Duration` - Duration value

#### Usage Examples
```python
from std_msgs.msg import Bool, Float64, String

# Boolean status message
status_msg = Bool()
status_msg.data = True

# Numeric data message
value_msg = Float64()
value_msg.data = 3.14159

# Text message
text_msg = String()
text_msg.data = "Hello, robot!"
```

### std_srvs
Standard service types.

#### Common Services
- `Empty` - Service with no request or response
- `SetBool` - Set boolean value
- `Trigger` - Simple trigger service

#### Usage Example
```python
from std_srvs.srv import SetBool

def set_enabled_callback(request, response):
    if request.data:
        # Enable system
        response.success = True
        response.message = "System enabled"
    else:
        # Disable system
        response.success = True
        response.message = "System disabled"
    return response
```

## Sensor Messages

### sensor_msgs
Messages for sensor data and processing.

#### Image Messages
```python
from sensor_msgs.msg import Image, CompressedImage

# Raw image message
image_msg = Image()
image_msg.header.stamp = self.get_clock().now().to_msg()
image_msg.header.frame_id = "camera_frame"
image_msg.height = 480
image_msg.width = 640
image_msg.encoding = "rgb8"
image_msg.is_bigendian = False
image_msg.step = 640 * 3  # Width * bytes per pixel
image_msg.data = image_bytes  # Flattened pixel data
```

#### Camera Info
```python
from sensor_msgs.msg import CameraInfo

# Camera calibration information
camera_info = CameraInfo()
camera_info.header.frame_id = "camera_optical_frame"
camera_info.height = 480
camera_info.width = 640
camera_info.distortion_model = "plumb_bob"
camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # Distortion coefficients
camera_info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]  # Intrinsic matrix
camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]  # Rectification matrix
camera_info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]  # Projection matrix
```

#### LaserScan
```python
from sensor_msgs.msg import LaserScan

# LIDAR scan data
scan_msg = LaserScan()
scan_msg.header.stamp = self.get_clock().now().to_msg()
scan_msg.header.frame_id = "laser_frame"
scan_msg.angle_min = -1.57  # -90 degrees in radians
scan_msg.angle_max = 1.57   # 90 degrees in radians
scan_msg.angle_increment = 0.01745  # 1 degree in radians
scan_msg.time_increment = 0.0  # Time between measurements
scan_msg.scan_time = 0.1  # Time between scans
scan_msg.range_min = 0.1  # Minimum range
scan_msg.range_max = 10.0  # Maximum range
scan_msg.ranges = [1.5, 1.6, 1.4, ...]  # Range measurements
scan_msg.intensities = [200, 190, 210, ...]  # Intensity values
```

#### PointCloud2
```python
from sensor_msgs.msg import PointCloud2, PointField
import struct

# Point cloud data
cloud_msg = PointCloud2()
cloud_msg.header.stamp = self.get_clock().now().to_msg()
cloud_msg.header.frame_id = "sensor_frame"
cloud_msg.height = 1  # Unordered point cloud
cloud_msg.width = num_points
cloud_msg.is_dense = False
cloud_msg.is_bigendian = False

# Define point structure
fields = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='rgb', offset=12, datatype=PointField.UINT32, count=1)
]
cloud_msg.fields = fields
cloud_msg.point_step = 16  # Size of each point in bytes
cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width

# Pack point data
point_data = []
for point in points:
    # Pack x, y, z, rgba
    packed = struct.pack('fffI', point.x, point.y, point.z, point.rgba)
    point_data.append(packed)

cloud_msg.data = b''.join(point_data)
```

#### Imu (Inertial Measurement Unit)
```python
from sensor_msgs.msg import Imu

# IMU data
imu_msg = Imu()
imu_msg.header.stamp = self.get_clock().now().to_msg()
imu_msg.header.frame_id = "imu_link"

# Orientation (quaternion)
imu_msg.orientation.x = 0.0
imu_msg.orientation.y = 0.0
imu_msg.orientation.z = 0.0
imu_msg.orientation.w = 1.0

# Orientation covariance (set to 0 if unknown)
imu_msg.orientation_covariance = [0.0] * 9

# Angular velocity
imu_msg.angular_velocity.x = 0.1  # rad/s
imu_msg.angular_velocity.y = 0.05
imu_msg.angular_velocity.z = 0.02

# Angular velocity covariance
imu_msg.angular_velocity_covariance = [0.0] * 9

# Linear acceleration
imu_msg.linear_acceleration.x = 0.0  # m/s^2
imu_msg.linear_acceleration.y = 0.0
imu_msg.linear_acceleration.z = 9.81

# Linear acceleration covariance
imu_msg.linear_acceleration_covariance = [0.0] * 9
```

#### JointState
```python
from sensor_msgs.msg import JointState

# Robot joint states
joint_msg = JointState()
joint_msg.header.stamp = self.get_clock().now().to_msg()
joint_msg.header.frame_id = "base_link"

# Joint names
joint_msg.name = ["joint_1", "joint_2", "joint_3"]

# Joint positions (radians)
joint_msg.position = [0.1, -0.5, 1.2]

# Joint velocities (rad/s)
joint_msg.velocity = [0.0, 0.1, -0.05]

# Joint efforts (torque in N*m)
joint_msg.effort = [1.5, 2.0, 0.8]
```

## Geometry Messages

### geometry_msgs
Messages for geometric data and transformations.

#### Point
```python
from geometry_msgs.msg import Point

# 3D point
point = Point()
point.x = 1.0
point.y = 2.0
point.z = 3.0
```

#### Vector3
```python
from geometry_msgs.msg import Vector3

# 3D vector
vector = Vector3()
vector.x = 1.0
vector.y = 0.0
vector.z = 0.0
```

#### Pose
```python
from geometry_msgs.msg import Pose

# Position and orientation
pose = Pose()
pose.position.x = 1.0
pose.position.y = 2.0
pose.position.z = 0.0

pose.orientation.x = 0.0
pose.orientation.y = 0.0
pose.orientation.z = 0.0
pose.orientation.w = 1.0
```

#### Twist
```python
from geometry_msgs.msg import Twist

# Linear and angular velocities
twist = Twist()
twist.linear.x = 0.5  # Forward velocity (m/s)
twist.linear.y = 0.0
twist.linear.z = 0.0

twist.angular.x = 0.0
twist.angular.y = 0.0
twist.angular.z = 0.2  # Angular velocity (rad/s)
```

#### TransformStamped
```python
from geometry_msgs.msg import TransformStamped

# Transformation between frames
transform = TransformStamped()
transform.header.stamp = self.get_clock().now().to_msg()
transform.header.frame_id = "parent_frame"
transform.child_frame_id = "child_frame"

# Translation
transform.transform.translation.x = 1.0
transform.transform.translation.y = 0.0
transform.transform.translation.z = 0.0

# Rotation (quaternion)
transform.transform.rotation.x = 0.0
transform.transform.rotation.y = 0.0
transform.transform.rotation.z = 0.0
transform.transform.rotation.w = 1.0
```

## Navigation Messages

### nav_msgs
Messages for navigation and path planning.

#### Odometry
```python
from nav_msgs.msg import Odometry

# Robot odometry
odom_msg = Odometry()
odom_msg.header.stamp = self.get_clock().now().to_msg()
odom_msg.header.frame_id = "odom"
odom_msg.child_frame_id = "base_link"

# Position and orientation
odom_msg.pose.pose.position.x = 1.0
odom_msg.pose.pose.position.y = 2.0
odom_msg.pose.pose.position.z = 0.0

odom_msg.pose.pose.orientation.w = 1.0

# Covariance (uncertainty)
odom_msg.pose.covariance = [0.0] * 36  # 6x6 covariance matrix

# Velocity
odom_msg.twist.twist.linear.x = 0.1
odom_msg.twist.twist.angular.z = 0.05

# Velocity covariance
odom_msg.twist.covariance = [0.0] * 36
```

#### Path
```python
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# Planned path
path_msg = Path()
path_msg.header.stamp = self.get_clock().now().to_msg()
path_msg.header.frame_id = "map"

# Waypoints
poses = []
for i in range(10):
    pose_stamped = PoseStamped()
    pose_stamped.header.stamp = self.get_clock().now().to_msg()
    pose_stamped.header.frame_id = "map"

    pose_stamped.pose.position.x = i * 0.5
    pose_stamped.pose.position.y = 0.0
    pose_stamped.pose.position.z = 0.0

    pose_stamped.pose.orientation.w = 1.0

    poses.append(pose_stamped)

path_msg.poses = poses
```

#### OccupancyGrid
```python
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header

# Map representation
map_msg = OccupancyGrid()
map_msg.header = Header()
map_msg.header.stamp = self.get_clock().now().to_msg()
map_msg.header.frame_id = "map"

# Map metadata
map_msg.info.resolution = 0.05  # meters per cell
map_msg.info.width = 1000  # cells
map_msg.info.height = 1000  # cells
map_msg.info.origin.position.x = -25.0  # origin x
map_msg.info.origin.position.y = -25.0  # origin y
map_msg.info.origin.position.z = 0.0
map_msg.info.origin.orientation.w = 1.0

# Map data (0-100 for occupancy, -1 for unknown)
map_data = [0] * (1000 * 1000)  # Flat array of occupancy values
map_msg.data = map_data
```

## Action Messages

### action_msgs
Messages for action communication.

#### GoalStatusArray
```python
from action_msgs.msg import GoalStatusArray, GoalStatus

# Array of goal statuses
status_array = GoalStatusArray()
status_array.header.stamp = self.get_clock().now().to_msg()
status_array.header.frame_id = ""

# Individual goal status
goal_status = GoalStatus()
goal_status.goal_info.goal_id.uuid = b'...'  # Unique identifier
goal_status.goal_info.stamp = self.get_clock().now().to_msg()
goal_status.status = GoalStatus.STATUS_SUCCEEDED  # or other status constants

status_array.status_list = [goal_status]
```

## Custom Message Types for Humanoid Robotics

### Humanoid-Specific Messages
For humanoid robotics applications, you may need custom message types:

#### BalanceState
```python
# In your custom message file: BalanceState.msg
# std_msgs/Header header
# geometry_msgs/Vector3 com_position
# geometry_msgs/Vector3 zmp_position
# float64[] joint_targets
# bool stable
# float64 balance_score

# Usage in code:
from humanoid_msgs.msg import BalanceState

balance_msg = BalanceState()
balance_msg.header.stamp = self.get_clock().now().to_msg()
balance_msg.header.frame_id = "base_link"

# Center of mass
balance_msg.com_position.x = 0.0
balance_msg.com_position.y = 0.0
balance_msg.com_position.z = 0.8

# Zero Moment Point
balance_msg.zmp_position.x = 0.1
balance_msg.zmp_position.y = 0.0
balance_msg.zmp_position.z = 0.0

# Joint targets for balance
balance_msg.joint_targets = [0.0, 0.0, 0.1, -0.1, 0.05, -0.05]

# Stability status
balance_msg.stable = True
balance_msg.balance_score = 0.95
```

#### StepPlan
```python
# In your custom message file: StepPlan.msg
# std_msgs/Header header
# geometry_msgs/PoseArray footsteps
# float64[] timing
# string[] step_types

# Usage in code:
from humanoid_msgs.msg import StepPlan
from geometry_msgs.msg import PoseArray, Pose

step_plan_msg = StepPlan()
step_plan_msg.header.stamp = self.get_clock().now().to_msg()
step_plan_msg.header.frame_id = "map"

# Footstep poses
poses = PoseArray()
poses.header.stamp = self.get_clock().now().to_msg()
poses.header.frame_id = "map"

# Example footsteps
for i in range(5):
    pose = Pose()
    pose.position.x = i * 0.3  # 30cm steps
    pose.position.y = (-1) ** i * 0.1  # Alternating feet
    pose.position.z = 0.0
    pose.orientation.w = 1.0  # Facing forward
    poses.poses.append(pose)

step_plan_msg.footsteps = poses
step_plan_msg.timing = [0.0, 1.0, 2.0, 3.0, 4.0]  # Timing for each step
step_plan_msg.step_types = ["walk", "walk", "turn", "walk", "stop"]
```

## Creating Custom Messages

### Message Definition File
Create a `.msg` file in your package's `msg/` directory:

```bash
# Example: HumanoidJointState.msg
std_msgs/Header header
string[] joint_names
float64[] positions
float64[] velocities
float64[] efforts
float64[] commanded_positions
float64[] commanded_velocities
bool[] joint_limits_reached
```

### Package Configuration
Update your `package.xml`:

```xml
<build_depend>rosidl_default_generators</build_depend>
<exec_depend>rosidl_default_runtime</exec_depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

Update your `CMakeLists.txt`:

```cmake
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HumanoidJointState.msg"
  "srv/SetJointImpedance.srv"
  DEPENDENCIES std_msgs geometry_msgs
)
```

## Quality of Service (QoS) Considerations

### Appropriate QoS for Different Message Types

#### Critical Control Commands
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# For critical commands that must be delivered
critical_qos = QoSProfile(
    depth=1,  # Only keep most recent
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST
)
```

#### High-Frequency Sensor Data
```python
# For high-frequency sensor data where some loss is acceptable
sensor_qos = QoSProfile(
    depth=1,  # Only keep most recent
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST
)
```

#### Configuration Parameters
```python
# For configuration parameters that should persist
param_qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_ALL
)
```

## Message Serialization and Performance

### Efficient Message Handling
```python
import sys

def analyze_message_size(message):
    """Analyze the size of a ROS message"""
    import pickle
    serialized = pickle.dumps(message)
    size_bytes = len(serialized)
    size_mb = size_bytes / (1024 * 1024)
    print(f"Message size: {size_bytes} bytes ({size_mb:.2f} MB)")

def optimize_large_arrays(message, max_elements=1000):
    """Optimize large array messages"""
    # Downsample large arrays if necessary
    if hasattr(message, 'data') and hasattr(message.data, '__len__'):
        if len(message.data) > max_elements:
            # Take every nth element to reduce size
            step = len(message.data) // max_elements
            message.data = message.data[::step]
```

## Common Patterns and Best Practices

### Message Construction
```python
def create_timestamped_message(base_msg_type, frame_id="base_link"):
    """Create a message with timestamp and frame ID"""
    msg = base_msg_type()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.header.frame_id = frame_id
    return msg

# Usage
imu_msg = create_timestamped_message(Imu, "imu_link")
```

### Message Validation
```python
def validate_message(msg):
    """Validate message contents"""
    # Check for NaN values
    if hasattr(msg, 'data') and isinstance(msg.data, float):
        if math.isnan(msg.data):
            raise ValueError("Message contains NaN value")

    # Check for infinity values
    if hasattr(msg, 'data') and isinstance(msg.data, float):
        if math.isinf(msg.data):
            raise ValueError("Message contains infinity value")

    # Validate arrays
    if hasattr(msg, 'data') and isinstance(msg.data, (list, tuple)):
        for item in msg.data:
            if isinstance(item, float):
                if math.isnan(item) or math.isinf(item):
                    raise ValueError("Array contains invalid numeric value")
```

### Error Handling
```python
def safe_publish(self, publisher, msg):
    """Safely publish message with error handling"""
    try:
        publisher.publish(msg)
    except Exception as e:
        self.get_logger().error(f"Failed to publish message: {e}")
        # Optionally implement fallback behavior
```

## Message Type Compatibility

### Version Compatibility
When creating custom messages, consider:
- Use standard types when possible
- Document breaking changes clearly
- Provide conversion functions between versions
- Use semantic versioning for message packages

### Migration Strategies
```python
def migrate_old_message(old_msg):
    """Convert old message format to new format"""
    new_msg = NewMessageType()

    # Copy common fields
    new_msg.header = old_msg.header
    new_msg.data = old_msg.data

    # Handle new fields with defaults
    new_msg.new_field = 0.0  # Default value

    return new_msg
```

## Debugging Message Issues

### Common Message Problems
1. **Serialization Errors**: Check message definitions for correctness
2. **Size Issues**: Large messages may exceed transport limits
3. **Timing Issues**: Ensure timestamps are properly set
4. **Frame ID Issues**: Verify coordinate frame consistency
5. **Data Type Issues**: Check for overflow/underflow of numeric types

### Debugging Tools
```python
def debug_message_fields(msg):
    """Print message field information"""
    for field_name in dir(msg):
        if not field_name.startswith('_'):
            field_value = getattr(msg, field_name)
            print(f"{field_name}: {field_value} (type: {type(field_value)})")
```

This comprehensive reference provides the essential information needed to work effectively with ROS 2 messages in humanoid robotics applications. Understanding these message types and their proper usage is fundamental to creating robust, interoperable robotic systems.