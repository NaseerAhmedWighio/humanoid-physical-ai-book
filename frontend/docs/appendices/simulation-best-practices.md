---
sidebar_position: 6
title: "Appendix F: Simulation Best Practices"
---

# Appendix F: Simulation Best Practices

This appendix provides comprehensive guidelines and best practices for developing effective and realistic robotic simulations. Proper simulation practices are crucial for creating systems that can successfully transfer from simulation to real-world deployment, particularly for complex humanoid robots.

## General Simulation Principles

### 1. Fidelity vs. Performance Trade-offs

Balancing simulation accuracy with computational performance is a fundamental consideration in robotics simulation:

#### High-Fidelity Requirements
- **Accurate Physics**: Use realistic mass, inertia, and friction parameters
- **Precise Sensors**: Model sensor noise, latency, and field-of-view limitations
- **Complex Environments**: Include realistic lighting, textures, and environmental conditions
- **Realistic Actuators**: Model motor dynamics, backlash, and control delays

#### Performance Optimization
- **Simplified Geometries**: Use simplified collision meshes for non-visual components
- **Reduced Update Rates**: Lower non-critical update frequencies
- **Approximate Physics**: Use simpler physics models where appropriate
- **Level of Detail (LOD)**: Implement distance-based simplification

#### Balancing Strategy
```python
class SimulationFidelityManager:
    def __init__(self):
        self.fidelity_level = "balanced"  # low, balanced, high
        self.performance_threshold = 0.9  # Real-time factor threshold

    def adjust_fidelity(self, current_rtf):
        """Adjust simulation fidelity based on performance"""
        if current_rtf < self.performance_threshold * 0.8:
            # Too slow - reduce fidelity
            self.reduce_fidelity()
        elif current_rtf > self.performance_threshold * 1.2:
            # Too fast - can increase fidelity
            self.increase_fidelity()

    def reduce_fidelity(self):
        """Reduce simulation fidelity for better performance"""
        if self.fidelity_level == "high":
            self.set_physics_quality("medium")
            self.simplify_collision_meshes()
            self.reduce_render_quality()
        elif self.fidelity_level == "balanced":
            self.set_physics_quality("low")
            self.reduce_render_quality()

    def increase_fidelity(self):
        """Increase simulation fidelity if performance allows"""
        if self.fidelity_level == "low":
            self.set_physics_quality("medium")
            self.restore_render_quality()
        elif self.fidelity_level == "balanced":
            self.set_physics_quality("high")
            self.restore_render_quality()
```

### 2. Model Accuracy and Validation

#### Mass and Inertia Properties
```xml
<!-- Correct URDF mass and inertia properties -->
<link name="upper_arm">
  <inertial>
    <!-- Mass in kilograms -->
    <mass value="1.5"/>

    <!-- Origin relative to link frame -->
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>

    <!-- Inertia matrix (symmetric) -->
    <inertia
      ixx="0.01" ixy="0.0" ixz="0.0"
      iyy="0.01" iyz="0.0"
      izz="0.001"/>
  </inertial>
</link>
```

#### Validation Techniques
- **Physical Property Verification**: Compare simulation behavior with physical laws
- **Parameter Sensitivity Analysis**: Test how sensitive results are to parameter changes
- **Cross-Validation**: Compare with other simulation tools or analytical models
- **Experimental Validation**: Compare with real robot data when available

### 3. Physics Engine Selection and Configuration

#### Choosing Physics Engines
- **ODE (Open Dynamics Engine)**: Good balance of speed and accuracy
- **Bullet**: Excellent for complex contact scenarios
- **DART**: Advanced for articulated bodies and humanoid robots
- **PhysX**: High accuracy, used in Isaac Sim

#### Configuration Guidelines
```xml
<!-- Physics configuration in SDF -->
<physics type="ode">
  <!-- Time step (smaller = more accurate but slower) -->
  <max_step_size>0.001</max_step_size>

  <!-- Real-time factor (1.0 = real-time) -->
  <real_time_factor>1.0</real_time_factor>

  <!-- Update rate -->
  <real_time_update_rate>1000</real_time_update_rate>

  <ode>
    <solver>
      <type>quick</type>  <!-- or 'pgs', 'dantzig' -->
      <iters>100</iters>  <!-- Solver iterations -->
      <sor>1.3</sor>      <!-- Successive over-relaxation -->
    </solver>
    <constraints>
      <cfm>0.0</cfm>  <!-- Constraint force mixing -->
      <erp>0.2</erp>  <!-- Error reduction parameter -->
      <contact_max_correcting_vel>100.0</contact_max_correcting_vel>
      <contact_surface_layer>0.001</contact_surface_layer>
    </constraints>
  </ode>
</physics>
```

## Gazebo Simulation Best Practices

### 1. Robot Model Optimization

#### Visual vs. Collision Separation
```xml
<link name="arm_link">
  <!-- Detailed visual model -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <mesh filename="package://robot_description/meshes/arm_visual.dae"/>
    </geometry>
    <material name="gray">
      <color rgba="0.5 0.5 0.5 1.0"/>
    </material>
  </visual>

  <!-- Simplified collision model -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <!-- Use simpler primitive instead of complex mesh -->
      <cylinder length="0.3" radius="0.05"/>
    </geometry>
  </collision>
</link>
```

#### Joint Configuration
```xml
<joint name="shoulder_joint" type="revolute">
  <parent link="torso"/>
  <child link="upper_arm"/>
  <origin xyz="0.15 0 0.5" rpy="0 0 0"/>
  <axis xyz="0 1 0"/>  <!-- Rotation axis -->

  <!-- Joint limits -->
  <limit lower="-2.0" upper="2.0" effort="100" velocity="2"/>

  <!-- Dynamics -->
  <dynamics damping="0.1" friction="0.0"/>

  <!-- Safety limits (optional) -->
  <safety_controller soft_lower_limit="-1.9" soft_upper_limit="1.9"
                   k_position="100" k_velocity="1"/>
</joint>
```

### 2. Sensor Configuration Best Practices

#### Camera Sensors
```xml
<gazebo reference="camera_link">
  <sensor name="camera" type="camera">
    <!-- Update rate -->
    <update_rate>30</update_rate>

    <camera name="head_camera">
      <!-- Field of view -->
      <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->

      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>

      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
    </camera>

    <!-- Plugin for ROS integration -->
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <frame_name>camera_optical_frame</frame_name>
      <topic_name>image_raw</topic_name>
      <hack_baseline>0.07</hack_baseline>
    </plugin>
  </sensor>
</gazebo>
```

#### IMU Sensors
```xml
<gazebo reference="imu_link">
  <sensor name="imu" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>

    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </z>
      </angular_velocity>

      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
            <bias_mean>0.0</bias_mean>
            <bias_stddev>0.001</bias_stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
  </sensor>
</gazebo>
```

### 3. World and Environment Design

#### Terrain and Obstacles
```xml
<world name="humanoid_world">
  <!-- Include standard models -->
  <include>
    <uri>model://ground_plane</uri>
  </include>

  <include>
    <uri>model://sun</uri>
  </include>

  <!-- Custom terrain -->
  <model name="uneven_terrain">
    <pose>0 0 0 0 0 0</pose>
    <link name="terrain_link">
      <collision>
        <geometry>
          <heightmap>
            <uri>file://terrain.png</uri>
            <size>10 10 1</size>
            <pos>0 0 0</pos>
          </heightmap>
        </geometry>
      </collision>
      <visual>
        <geometry>
          <heightmap>
            <uri>file://terrain.png</uri>
            <size>10 10 1</size>
          </heightmap>
        </geometry>
      </visual>
    </link>
  </model>

  <!-- Obstacles -->
  <model name="obstacle_box">
    <pose>2 0 0.5 0 0 0</pose>
    <link name="link">
      <inertial>
        <mass>1.0</mass>
        <inertia>
          <ixx>0.083</ixx>
          <ixy>0.0</ixy>
          <ixz>0.0</ixz>
          <iyy>0.083</iyy>
          <iyz>0.0</iyz>
          <izz>0.083</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.5 0.5 1.0</size>
          </box>
        </geometry>
        <material>
          <ambient>0.5 0.5 0.5 1</ambient>
          <diffuse>0.7 0.7 0.7 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.5 0.5 1.0</size>
          </box>
        </geometry>
      </collision>
    </link>
  </model>
</world>
```

## Unity Simulation Best Practices

### 1. Physics Optimization

#### Rigidbody Configuration
```csharp
using UnityEngine;

public class RobotLink : MonoBehaviour
{
    [Header("Physics Properties")]
    public float mass = 1.0f;
    public bool useGravity = true;
    public bool isKinematic = false;
    public float drag = 0.05f;
    public float angularDrag = 0.05f;

    [Header("Collision Properties")]
    public bool useInterpolation = true;
    public CollisionDetectionMode collisionDetectionMode = CollisionDetectionMode.Continuous;

    private Rigidbody rb;

    void Start()
    {
        rb = GetComponent<Rigidbody>();
        if (rb == null)
        {
            rb = gameObject.AddComponent<Rigidbody>();
        }

        ConfigureRigidbody();
    }

    void ConfigureRigidbody()
    {
        rb.mass = mass;
        rb.useGravity = useGravity;
        rb.isKinematic = isKinematic;
        rb.drag = drag;
        rb.angularDrag = angularDrag;
        rb.interpolation = useInterpolation ?
            RigidbodyInterpolation.Interpolate : RigidbodyInterpolation.None;
        rb.collisionDetectionMode = collisionDetectionMode;
    }

    void FixedUpdate()
    {
        // Physics calculations in FixedUpdate
        ApplyForces();
    }

    void ApplyForces()
    {
        // Apply control forces here
    }
}
```

#### Joint Configuration
```csharp
using UnityEngine;

public class RobotJoint : MonoBehaviour
{
    [Header("Joint Configuration")]
    public JointType jointType = JointType.Revolute;
    public Vector3 axis = Vector3.up;
    public float minLimit = -45f;
    public float maxLimit = 45f;
    public float spring = 0f;
    public float damper = 0f;
    public float targetPosition = 0f;

    private Joint joint;

    void Start()
    {
        CreateJoint();
    }

    void CreateJoint()
    {
        switch (jointType)
        {
            case JointType.Revolute:
                CreateHingeJoint();
                break;
            case JointType.Prismatic:
                CreateSliderJoint();
                break;
            case JointType.Fixed:
                CreateFixedJoint();
                break;
        }
    }

    void CreateHingeJoint()
    {
        HingeJoint hinge = gameObject.AddComponent<HingeJoint>();
        hinge.axis = axis;

        JointLimits limits = hinge.limits;
        limits.min = minLimit;
        limits.max = maxLimit;
        hinge.limits = limits;

        if (spring > 0)
        {
            hinge.spring = new JointSpring
            {
                spring = spring,
                damper = damper,
                targetPosition = targetPosition
            };
        }

        joint = hinge;
    }

    public enum JointType
    {
        Revolute,
        Prismatic,
        Fixed
    }
}
```

### 2. Sensor Simulation

#### Camera Sensor with Noise
```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class CameraSensor : MonoBehaviour
{
    [Header("Camera Configuration")]
    public int width = 640;
    public int height = 480;
    public float fieldOfView = 60f;
    public string topicName = "/camera/image_raw";
    public float publishRate = 30f;

    [Header("Noise Parameters")]
    public float noiseLevel = 0.01f;
    public float blurAmount = 0.1f;

    private Camera cam;
    private RenderTexture renderTexture;
    private Texture2D texture2D;
    private RosConnection ros;
    private float publishInterval;
    private float lastPublishTime;

    void Start()
    {
        SetupCamera();
        ros = RosConnection.GetOrCreateInstance();
        publishInterval = 1f / publishRate;
        lastPublishTime = 0f;
    }

    void SetupCamera()
    {
        cam = GetComponent<Camera>();
        if (cam == null)
        {
            cam = gameObject.AddComponent<Camera>();
        }

        cam.fieldOfView = fieldOfView;
        cam.backgroundColor = Color.black;
        cam.clearFlags = CameraClearFlags.SolidColor;

        renderTexture = new RenderTexture(width, height, 24);
        cam.targetTexture = renderTexture;
        texture2D = new Texture2D(width, height, TextureFormat.RGB24, false);
    }

    void Update()
    {
        if (Time.time - lastPublishTime >= publishInterval)
        {
            CaptureAndPublishImage();
            lastPublishTime = Time.time;
        }
    }

    void CaptureAndPublishImage()
    {
        RenderTexture.active = renderTexture;
        texture2D.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        texture2D.Apply();

        // Add noise to image
        AddNoiseToTexture(texture2D);

        byte[] imageData = texture2D.EncodeToJPG();

        ImageMsg imageMsg = new ImageMsg
        {
            header = new HeaderMsg
            {
                stamp = new TimeStamp(Time.time),
                frame_id = "camera_optical_frame"
            },
            height = (uint)height,
            width = (uint)width,
            encoding = "rgb8",
            is_bigendian = 0,
            step = (uint)(width * 3),
            data = imageData
        };

        ros.Publish(topicName, imageMsg);
    }

    void AddNoiseToTexture(Texture2D texture)
    {
        Color[] pixels = texture.GetPixels();

        for (int i = 0; i < pixels.Length; i++)
        {
            // Add Gaussian noise
            float noise = Random.Range(-noiseLevel, noiseLevel);

            pixels[i].r = Mathf.Clamp01(pixels[i].r + noise);
            pixels[i].g = Mathf.Clamp01(pixels[i].g + noise);
            pixels[i].b = Mathf.Clamp01(pixels[i].b + noise);
        }

        texture.SetPixels(pixels);
        texture.Apply();
    }
}
```

## NVIDIA Isaac Sim Best Practices

### 1. USD Scene Optimization

#### Efficient USD Structure
```python
# Efficient USD scene structure for Isaac Sim
from pxr import Usd, UsdGeom, Gf, Sdf

def create_optimized_scene(stage_path):
    """Create an optimized USD scene for robotics simulation"""
    stage = Usd.Stage.CreateNew(stage_path)

    # Create basic scene hierarchy
    world_prim = stage.DefinePrim("/World", "Xform")

    # Add ground plane with efficient subdivision
    ground_prim = stage.DefinePrim("/World/GroundPlane", "Mesh")
    ground_mesh = UsdGeom.Mesh(ground_prim)

    # Use efficient geometry for physics
    ground_mesh.CreatePointsAttr([
        (-10, -10, 0), (10, -10, 0), (10, 10, 0), (-10, 10, 0)
    ])
    ground_mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    ground_mesh.CreateFaceVertexCountsAttr([4])

    # Create robot hierarchy
    robot_prim = stage.DefinePrim("/World/Robot", "Xform")

    # Add robot links with appropriate materials
    create_robot_links(robot_prim, stage)

    stage.GetRootLayer().Save()
    return stage

def create_robot_links(robot_prim, stage):
    """Create optimized robot links with proper materials"""
    # Create links with simplified collision geometry
    for i in range(5):  # Example: 5 links
        link_name = f"link_{i}"
        link_prim = stage.DefinePrim(f"/World/Robot/{link_name}", "Xform")

        # Visual geometry (detailed)
        visual_geom = stage.DefinePrim(f"/World/Robot/{link_name}/visual", "Cylinder")
        visual_cylinder = UsdGeom.Cylinder(visual_geom)
        visual_cylinder.CreateRadiusAttr(0.05)
        visual_cylinder.CreateHeightAttr(0.2)

        # Collision geometry (simplified)
        collision_geom = stage.DefinePrim(f"/World/Robot/{link_name}/collision", "Capsule")
        collision_capsule = UsdGeom.Capsule(collision_geom)
        collision_capsule.CreateRadiusAttr(0.06)
        collision_capsule.CreateHeightAttr(0.25)
```

### 2. Isaac Sim Configuration

#### Physics Settings
```python
from omni.isaac.core.utils.physics import set_physics_dt, set_gravity
from omni.isaac.core import World

def configure_physics_settings(world: World, dt=1/60.0, gravity=-9.81):
    """Configure physics settings for optimal humanoid simulation"""

    # Set physics time step
    set_physics_dt(dt)

    # Set gravity
    set_gravity(gravity)

    # Configure solver settings
    physics_ctx = world.get_physics_context()
    physics_ctx.set_solver_type("TGS")  # TGS solver for better stability
    physics_ctx.set_position_iteration_count(8)
    physics_ctx.set_velocity_iteration_count(4)
    physics_ctx.set_gpu_max_rigid_contact_count(1024000)
    physics_ctx.set_gpu_max_rigid_patch_count(80000)

def setup_robot_for_simulation(robot_path, position, orientation):
    """Setup robot in Isaac Sim with proper initial conditions"""
    from omni.isaac.core import Robot

    robot = Robot(
        prim_path=robot_path,
        name="humanoid_robot",
        position=position,
        orientation=orientation
    )

    # Configure robot properties
    robot.initialize()

    # Set initial joint positions
    # This should match your robot's home position
    initial_positions = [0.0] * robot.num_dof
    robot.set_joint_positions(initial_positions)

    return robot
```

## Domain Randomization Techniques

### 1. Physics Parameter Randomization

```python
import random
import numpy as np

class DomainRandomizer:
    """Randomize physics parameters for improved sim-to-real transfer"""

    def __init__(self):
        self.param_ranges = {
            'mass_multiplier': (0.8, 1.2),
            'friction_range': (0.1, 1.0),
            'restitution_range': (0.0, 0.5),
            'gravity_range': (-10.0, -9.0),
            'drag_range': (0.01, 0.1),
            'angular_drag_range': (0.01, 0.1)
        }

    def randomize_robot_params(self, robot_model):
        """Randomize robot physical parameters"""
        # Randomize link masses
        for link in robot_model.links:
            mass_multiplier = random.uniform(*self.param_ranges['mass_multiplier'])
            link.mass *= mass_multiplier

        # Randomize joint friction and damping
        for joint in robot_model.joints:
            friction = random.uniform(*self.param_ranges['friction_range'])
            damping = random.uniform(*self.param_ranges['angular_drag_range'])

            joint.friction = friction
            joint.damping = damping

    def randomize_environment(self, world_model):
        """Randomize environment physics parameters"""
        # Randomize gravity
        gravity_z = random.uniform(*self.param_ranges['gravity_range'])
        world_model.gravity = [0, 0, gravity_z]

        # Randomize floor friction
        floor_friction = random.uniform(*self.param_ranges['friction_range'])
        world_model.floor_friction = floor_friction

    def randomize_sensor_params(self, sensor_models):
        """Randomize sensor noise parameters"""
        for sensor in sensor_models:
            if hasattr(sensor, 'noise_level'):
                noise_multiplier = random.uniform(0.5, 2.0)
                sensor.noise_level *= noise_multiplier

            if hasattr(sensor, 'bias_range'):
                bias_offset = random.uniform(-0.1, 0.1)
                sensor.bias_range = [b + bias_offset for b in sensor.bias_range]
```

### 2. Visual Domain Randomization

```python
import random
import numpy as np
from PIL import Image, ImageEnhance, ImageFilter

class VisualDomainRandomizer:
    """Randomize visual appearance for improved sim-to-real transfer"""

    def __init__(self):
        self.lighting_params = {
            'intensity_range': (0.5, 2.0),
            'color_temperature_range': (3000, 8000),
            'shadow_intensity_range': (0.1, 1.0)
        }

        self.material_params = {
            'roughness_range': (0.05, 0.95),
            'metallic_range': (0.0, 1.0),
            'specular_range': (0.0, 1.0)
        }

    def randomize_lighting(self, lights):
        """Randomize lighting conditions"""
        for light in lights:
            # Randomize intensity
            intensity_mult = random.uniform(*self.lighting_params['intensity_range'])
            light.intensity *= intensity_mult

            # Randomize color temperature
            color_temp = random.uniform(*self.lighting_params['color_temperature_range'])
            light.color = self.color_temperature_to_rgb(color_temp)

    def color_temperature_to_rgb(self, kelvin):
        """Convert color temperature in Kelvin to RGB values"""
        temp = kelvin / 100
        red, green, blue = 0, 0, 0

        # Red
        if temp <= 66:
            red = 255
        else:
            red = temp - 60
            red = 329.698727446 * (red ** -0.1332047592)
            red = max(0, min(255, red))

        # Green
        if temp <= 66:
            green = temp
            green = 99.4708025861 * np.log(green) - 161.1195681661
        else:
            green = temp - 60
            green = 288.1221695283 * (green ** -0.0755148492)
        green = max(0, min(255, green))

        # Blue
        if temp >= 66:
            blue = 255
        elif temp <= 19:
            blue = 0
        else:
            blue = temp - 10
            blue = 138.5177312231 * np.log(blue) - 305.0447927307
            blue = max(0, min(255, blue))

        return [red/255, green/255, blue/255]

    def randomize_materials(self, materials):
        """Randomize material properties"""
        for material in materials:
            # Randomize roughness
            roughness = random.uniform(*self.material_params['roughness_range'])
            material.roughness = roughness

            # Randomize metallic
            metallic = random.uniform(*self.material_params['metallic_range'])
            material.metallic = metallic

    def apply_image_augmentation(self, image):
        """Apply random image augmentations to simulate real-world conditions"""
        img = Image.fromarray(image)

        # Random brightness
        brightness_factor = random.uniform(0.7, 1.3)
        enhancer = ImageEnhance.Brightness(img)
        img = enhancer.enhance(brightness_factor)

        # Random contrast
        contrast_factor = random.uniform(0.8, 1.2)
        enhancer = ImageEnhance.Contrast(img)
        img = enhancer.enhance(contrast_factor)

        # Random saturation
        saturation_factor = random.uniform(0.8, 1.2)
        enhancer = ImageEnhance.Color(img)
        img = enhancer.enhance(saturation_factor)

        # Add noise
        if random.random() < 0.3:  # 30% chance to add noise
            img_array = np.array(img)
            noise = np.random.normal(0, random.uniform(0, 25), img_array.shape)
            img_array = np.clip(img_array + noise, 0, 255).astype(np.uint8)
            img = Image.fromarray(img_array)

        return np.array(img)
```

## Performance Optimization Strategies

### 1. Simulation Optimization

```python
class SimulationOptimizer:
    """Optimize simulation performance for large-scale training"""

    def __init__(self):
        self.optimization_settings = {
            'max_substeps': 1,
            'solver_iterations': 4,
            'contact_approximation': 'TGS',
            'enable_ccd': False,
            'broadphase_type': 'MBP'
        }

    def optimize_for_training(self):
        """Optimize settings for fast training"""
        # Reduce solver iterations for speed
        self.optimization_settings['solver_iterations'] = 2

        # Use simpler contact approximation
        self.optimization_settings['contact_approximation'] = 'SPLIT_IMPULSE'

        # Disable expensive features
        self.optimization_settings['enable_ccd'] = False

        # Increase time step (if stability allows)
        self.optimization_settings['max_step_size'] = 0.01

    def optimize_for_accuracy(self):
        """Optimize settings for high-fidelity simulation"""
        # Increase solver iterations
        self.optimization_settings['solver_iterations'] = 16

        # Use more accurate contact approximation
        self.optimization_settings['contact_approximation'] = 'TGS'

        # Enable CCD for fast-moving objects
        self.optimization_settings['enable_ccd'] = True

        # Decrease time step for accuracy
        self.optimization_settings['max_step_size'] = 0.001

    def batch_simulation_setup(self, num_envs, env_spacing=2.0):
        """Setup multiple environments efficiently"""
        # Calculate grid layout for environments
        grid_size = int(np.ceil(np.sqrt(num_envs)))

        environments = []
        for i in range(num_envs):
            row = i // grid_size
            col = i % grid_size

            # Position environment in grid
            x_pos = col * env_spacing
            y_pos = row * env_spacing

            env = self.create_environment(position=[x_pos, y_pos, 0])
            environments.append(env)

        return environments
```

### 2. Memory Management

```python
import gc
import psutil
import torch

class MemoryManager:
    """Manage memory usage in simulation environments"""

    def __init__(self, max_memory_fraction=0.8):
        self.max_memory_fraction = max_memory_fraction
        self.current_memory_usage = 0

    def monitor_memory(self):
        """Monitor current memory usage"""
        # System memory
        system_memory = psutil.virtual_memory()
        self.current_memory_usage = system_memory.percent

        # GPU memory (if available)
        if torch.cuda.is_available():
            gpu_memory = torch.cuda.memory_allocated() / torch.cuda.get_device_properties(0).total_memory
            gpu_usage_percent = gpu_memory * 100
            print(f"GPU Memory Usage: {gpu_usage_percent:.2f}%")

        return self.current_memory_usage

    def optimize_memory(self):
        """Optimize memory usage when approaching limits"""
        if self.current_memory_usage > self.max_memory_fraction * 100:
            # Clear PyTorch cache
            if torch.cuda.is_available():
                torch.cuda.empty_cache()

            # Run garbage collection
            gc.collect()

            # Reduce simulation complexity temporarily
            self.reduce_simulation_complexity()

    def reduce_simulation_complexity(self):
        """Reduce simulation complexity to save memory"""
        # Reduce number of parallel environments
        # Simplify collision meshes
        # Lower texture resolutions
        # Reduce sensor update rates
        pass
```

## Validation and Testing

### 1. Simulation Validation

```python
class SimulationValidator:
    """Validate simulation behavior against expected physics"""

    def __init__(self, robot_model):
        self.robot_model = robot_model
        self.validation_checks = []

    def add_validation_check(self, check_function, tolerance=0.01):
        """Add a validation check function"""
        self.validation_checks.append((check_function, tolerance))

    def validate_free_fall(self, duration=1.0):
        """Validate free fall acceleration (should be ~9.81 m/s²)"""
        initial_pos = self.robot_model.get_position()
        start_time = time.time()

        # Let object fall freely
        while time.time() - start_time < duration:
            self.robot_model.apply_force([0, 0, 0])  # No external forces
            self.simulation_step()

        final_pos = self.robot_model.get_position()
        final_time = time.time()

        # Calculate actual acceleration
        time_elapsed = final_time - start_time
        displacement = final_pos[2] - initial_pos[2]
        actual_acceleration = 2 * displacement / (time_elapsed ** 2)

        expected_acceleration = -9.81
        error = abs(actual_acceleration - expected_acceleration)

        return error < 0.1  # Allow 10% error

    def validate_conservation_of_energy(self):
        """Validate conservation of energy in closed system"""
        # Calculate total energy (kinetic + potential)
        # Should remain approximately constant
        pass

    def validate_joint_limits(self):
        """Validate that joints respect their limits"""
        joint_positions = self.robot_model.get_joint_positions()
        joint_limits = self.robot_model.get_joint_limits()

        for pos, limits in zip(joint_positions, joint_limits):
            if pos < limits[0] or pos > limits[1]:
                return False
        return True

    def run_all_validations(self):
        """Run all registered validation checks"""
        results = []
        for check_func, tolerance in self.validation_checks:
            try:
                result = check_func()
                results.append((check_func.__name__, result, tolerance))
            except Exception as e:
                results.append((check_func.__name__, False, tolerance, str(e)))

        return results
```

### 2. Transfer Validation

```python
class TransferValidator:
    """Validate sim-to-real transfer capabilities"""

    def __init__(self, sim_robot, real_robot):
        self.sim_robot = sim_robot
        self.real_robot = real_robot
        self.transfer_metrics = {}

    def compare_behavior(self, task_description, trials=10):
        """Compare robot behavior between simulation and reality"""
        sim_results = []
        real_results = []

        for trial in range(trials):
            # Reset both robots to same initial state
            self.sync_initial_state()

            # Execute task in simulation
            sim_result = self.execute_task_in_simulation(task_description)
            sim_results.append(sim_result)

            # Execute task in reality
            real_result = self.execute_task_in_reality(task_description)
            real_results.append(real_result)

        # Calculate similarity metrics
        similarity_score = self.calculate_similarity(sim_results, real_results)

        return {
            'similarity_score': similarity_score,
            'sim_results': sim_results,
            'real_results': real_results,
            'trials': trials
        }

    def calculate_similarity(self, sim_data, real_data):
        """Calculate similarity between simulation and real data"""
        # Use correlation, RMSE, or other appropriate metrics
        sim_array = np.array(sim_data)
        real_array = np.array(real_data)

        # Normalize data
        sim_norm = (sim_array - np.mean(sim_array)) / np.std(sim_array)
        real_norm = (real_array - np.mean(real_array)) / np.std(real_array)

        # Calculate correlation
        correlation = np.corrcoef(sim_norm, real_norm)[0, 1]

        return correlation

    def measure_domain_gap(self):
        """Measure the domain gap between simulation and reality"""
        # Various metrics to quantify differences
        metrics = {
            'behavior_correlation': self.calculate_behavior_correlation(),
            'sensor_fidelity': self.compare_sensor_readings(),
            'dynamics_fidelity': self.compare_dynamic_responses(),
            'control_stability': self.compare_control_performance()
        }

        return metrics
```

## Best Practices Summary

### 1. Development Workflow
- Start with simple models and gradually increase complexity
- Validate physics properties against real-world measurements
- Use version control for simulation assets and configurations
- Document all assumptions and simplifications
- Test with multiple scenarios and edge cases

### 2. Performance Considerations
- Profile simulation performance regularly
- Use appropriate level of detail based on requirements
- Consider parallel simulation for training
- Optimize collision geometries for performance
- Balance visual quality with physics accuracy

### 3. Validation Requirements
- Validate against analytical solutions where possible
- Compare with other simulation tools
- Test on real hardware when available
- Document simulation-to-reality gaps
- Implement automated validation tests

### 4. Documentation Standards
- Document all model parameters and assumptions
- Record simulation configurations used
- Track validation results and metrics
- Maintain changelog for model updates
- Include performance benchmarks

Following these best practices will help ensure your robotic simulations are both effective for development and reliable for sim-to-real transfer. The key is to maintain a balance between computational efficiency and physical accuracy while validating your models against real-world behavior.