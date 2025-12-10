---
sidebar_position: 8
title: "Week 6: Unity Robotics Integration"
---

# Week 6: Unity Robotics Integration

## Introduction to Unity for Robotics

Welcome to Week 6 of our comprehensive course on Physical AI and Humanoid Robotics. This week focuses on Unity, a powerful game engine that has emerged as a valuable tool for robotics simulation and development. Unity's high-fidelity graphics, realistic physics, and flexible development environment make it particularly valuable for humanoid robotics applications, especially in areas requiring photorealistic rendering and complex environment simulation.

Unity's appeal in robotics stems from several key capabilities:
- **Photorealistic Rendering**: Essential for training computer vision systems
- **Complex Environment Creation**: Detailed indoor and outdoor scenes
- **Physics Simulation**: Realistic physics for manipulation tasks
- **Human Character Simulation**: Natural human-robot interaction scenarios
- **Cross-Platform Deployment**: From development to embedded systems
- **Extensive Asset Library**: Pre-built environments and objects

For humanoid robotics, Unity provides unique advantages in simulating human environments, training perception systems with realistic imagery, and creating human-robot interaction scenarios that would be difficult or impossible to replicate in other simulation environments.

## Unity Robotics Ecosystem

### Unity Robotics Hub

Unity Robotics Hub is the central platform for robotics development in Unity:
- **Package Manager**: Access to robotics-specific packages
- **ROS-TCP-Connector**: Communication bridge between Unity and ROS/ROS 2
- **ML-Agents**: Reinforcement learning framework for robotics
- **Oculus XR**: Virtual reality integration for teleoperation
- **Simulation Tools**: Specialized tools for robotics simulation

### Core Robotics Packages

#### ROS-TCP-Connector
The ROS-TCP-Connector enables communication between Unity and ROS/ROS 2 systems:
- **TCP/IP Communication**: Standard protocol for cross-platform communication
- **Message Translation**: Converting between Unity and ROS message formats
- **Performance Optimization**: Efficient data transfer between systems
- **Multiple Connections**: Supporting multiple ROS nodes simultaneously

#### Unity Robotics Package
Provides robotics-specific utilities:
- **Robotics Components**: Specialized Unity components for robotics
- **Sensor Simulation**: High-fidelity sensor simulation
- **Control Interfaces**: Standardized control interfaces
- **Simulation Utilities**: Tools for robotics simulation

#### ML-Agents
Unity's machine learning framework:
- **Reinforcement Learning**: Training AI agents through simulation
- **Imitation Learning**: Learning from demonstrations
- **Curriculum Learning**: Progressive difficulty training
- **Multi-Agent Training**: Coordinated multi-robot learning

## Setting Up Unity for Robotics

### Installation Requirements

To set up Unity for robotics development:

1. **Unity Hub**: Download and install Unity Hub from Unity's website
2. **Unity Editor**: Install Unity 2021.3 LTS or later (recommended for robotics)
3. **Robotics Packages**: Install Unity Robotics packages through Package Manager
4. **ROS/ROS 2**: Install ROS/ROS 2 on the same machine or for network communication

### Unity Project Setup

Creating a robotics-focused Unity project:

1. **Create New Project**: Use 3D Core template
2. **Install Packages**: Through Window > Package Manager
   - ROS-TCP-Connector
   - ML-Agents (if needed)
   - XR packages (if needed)
3. **Configure Build Settings**: For deployment targets
4. **Set Up Scene**: Configure for robotics simulation

### Project Structure for Robotics

A well-organized robotics project typically includes:

```
Assets/
├── Scenes/                 # Simulation scenes
├── Scripts/                # C# scripts for robotics
│   ├── ROS/
│   ├── Sensors/
│   ├── Controllers/
│   └── Utilities/
├── Models/                 # 3D robot models
├── Materials/              # Material definitions
├── Prefabs/                # Reusable robot components
├── Plugins/                # Native plugins
└── StreamingAssets/        # Runtime data
```

## Creating Robot Models in Unity

### Importing Robot Models

Unity supports various 3D model formats commonly used in robotics:
- **FBX**: Most common format, good for complex models
- **OBJ**: Simple format, good for basic shapes
- **STL**: Often used for 3D printing, can be imported
- **URDF Importer**: Specialized tool for importing URDF files

### Robot Components in Unity

#### Rigidbodies
Unity's physics objects that correspond to URDF links:
- **Mass**: Physical mass of the component
- **Drag**: Linear motion resistance
- **Angular Drag**: Rotational motion resistance
- **Use Gravity**: Whether gravity affects the object
- **Is Kinematic**: Whether physics affects the object

```csharp
using UnityEngine;

public class RobotLink : MonoBehaviour
{
    [Header("Robot Link Properties")]
    public float mass = 1.0f;
    public bool useGravity = true;
    public bool isKinematic = false;
    public float linearDamping = 0.05f;
    public float angularDamping = 0.05f;

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
        rb.drag = linearDamping;
        rb.angularDrag = angularDamping;
    }
}
```

#### Joints
Unity's joint components that correspond to URDF joints:
- **Hinge Joint**: For revolute joints (rotation around one axis)
- **Fixed Joint**: For fixed joints (no relative movement)
- **Slider Joint**: For prismatic joints (linear movement)
- **Configurable Joint**: For complex joint constraints

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

    void CreateSliderJoint()
    {
        // Implementation for slider joint
    }

    void CreateFixedJoint()
    {
        FixedJoint fixedJoint = gameObject.AddComponent<FixedJoint>();
        joint = fixedJoint;
    }

    public enum JointType
    {
        Revolute,
        Prismatic,
        Fixed
    }
}
```

#### Colliders
Objects that define collision properties:
- **Mesh Collider**: Complex collision based on mesh
- **Box Collider**: Simple box-shaped collision
- **Sphere Collider**: Spherical collision
- **Capsule Collider**: Cylindrical with spherical ends

### Robot Control in Unity

#### Joint Control System

```csharp
using System.Collections.Generic;
using UnityEngine;

public class RobotController : MonoBehaviour
{
    [Header("Joint Configuration")]
    public List<JointController> joints = new List<JointController>();

    [Header("Control Parameters")]
    public float positionKp = 10f;
    public float velocityKp = 1f;
    public float maxTorque = 100f;

    void Start()
    {
        InitializeJoints();
    }

    void InitializeJoints()
    {
        // Find all joint controllers in children
        JointController[] foundJoints = GetComponentsInChildren<JointController>();
        joints.AddRange(foundJoints);
    }

    void Update()
    {
        ApplyJointControl();
    }

    void ApplyJointControl()
    {
        foreach (var joint in joints)
        {
            // Apply position control
            float targetPosition = joint.targetPosition;
            float currentPosition = joint.GetCurrentPosition();
            float positionError = targetPosition - currentPosition;

            float torque = positionKp * positionError;
            torque = Mathf.Clamp(torque, -maxTorque, maxTorque);

            joint.ApplyTorque(torque);
        }
    }

    public void SetJointPositions(float[] positions)
    {
        if (positions.Length != joints.Count)
        {
            Debug.LogError("Joint position array length mismatch");
            return;
        }

        for (int i = 0; i < joints.Count; i++)
        {
            joints[i].targetPosition = positions[i];
        }
    }

    public float[] GetJointPositions()
    {
        float[] positions = new float[joints.Count];
        for (int i = 0; i < joints.Count; i++)
        {
            positions[i] = joints[i].GetCurrentPosition();
        }
        return positions;
    }
}

[System.Serializable]
public class JointController
{
    public string jointName;
    public Transform jointTransform;
    public Joint joint;
    public float targetPosition;
    public JointType jointType;

    public float GetCurrentPosition()
    {
        switch (jointType)
        {
            case JointType.Revolute:
                HingeJoint hinge = joint as HingeJoint;
                if (hinge != null)
                {
                    return Mathf.Rad2Deg * hinge.angle * Mathf.Deg2Rad;
                }
                break;
            case JointType.Prismatic:
                ConfigurableJoint configJoint = joint as ConfigurableJoint;
                if (configJoint != null)
                {
                    return configJoint.transform.localPosition.magnitude;
                }
                break;
        }
        return 0f;
    }

    public void ApplyTorque(float torque)
    {
        Rigidbody connectedBody = joint.connectedBody;
        if (connectedBody != null)
        {
            Vector3 torqueVector = Vector3.zero;
            switch (jointType)
            {
                case JointType.Revolute:
                    torqueVector = joint.axis * torque;
                    connectedBody.AddTorque(torqueVector);
                    break;
            }
        }
    }

    public enum JointType
    {
        Revolute,
        Prismatic,
        Fixed
    }
}
```

## Unity-ROS Communication

### ROS-TCP-Connector Setup

The ROS-TCP-Connector enables communication between Unity and ROS systems:

#### Unity Side Setup

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using RosMessageTypes.Std;

public class UnityRosBridge : MonoBehaviour
{
    [Header("ROS Connection")]
    public string rosIPAddress = "127.0.0.1";
    public int rosPort = 10000;

    private RosConnection ros;

    void Start()
    {
        // Initialize ROS connection
        ros = RosConnection.GetOrCreateInstance();
        ros.Registered = true;

        // Connect to ROS
        ros.Connect(rosIPAddress, rosPort);

        Debug.Log($"Connected to ROS at {rosIPAddress}:{rosPort}");
    }

    // Example: Publish a message
    public void PublishMessage(string topic, Message message)
    {
        ros.Publish(topic, message);
    }

    // Example: Subscribe to a topic
    public void SubscribeToTopic<T>(string topic, System.Action<T> callback) where T : Message
    {
        ros.Subscribe<T>(topic, callback);
    }
}
```

#### Joint State Publisher

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

public class JointStatePublisher : MonoBehaviour
{
    [Header("Joint State Configuration")]
    public RobotController robotController;
    public string topicName = "/joint_states";
    public float publishRate = 50f; // Hz

    private RosConnection ros;
    private float publishInterval;
    private float lastPublishTime;

    void Start()
    {
        ros = RosConnection.GetOrCreateInstance();
        publishInterval = 1f / publishRate;
        lastPublishTime = 0f;
    }

    void Update()
    {
        if (Time.time - lastPublishTime >= publishInterval)
        {
            PublishJointStates();
            lastPublishTime = Time.time;
        }
    }

    void PublishJointStates()
    {
        if (robotController == null) return;

        float[] positions = robotController.GetJointPositions();
        string[] jointNames = GetJointNames();

        JointStateMsg jointState = new JointStateMsg
        {
            header = new HeaderMsg
            {
                stamp = new TimeStamp(Time.time),
                frame_id = "base_link"
            },
            name = jointNames,
            position = positions,
            velocity = new float[positions.Length],
            effort = new float[positions.Length]
        };

        ros.Publish(topicName, jointState);
    }

    string[] GetJointNames()
    {
        // Return joint names based on your robot configuration
        if (robotController != null && robotController.joints != null)
        {
            string[] names = new string[robotController.joints.Count];
            for (int i = 0; i < robotController.joints.Count; i++)
            {
                names[i] = robotController.joints[i].jointName;
            }
            return names;
        }
        return new string[0];
    }
}
```

#### Joint Command Subscriber

```csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;

public class JointCommandSubscriber : MonoBehaviour
{
    [Header("Command Configuration")]
    public RobotController robotController;
    public string topicName = "/joint_group_position_controller/command";

    private RosConnection ros;

    void Start()
    {
        ros = RosConnection.GetOrCreateInstance();

        // Subscribe to joint commands
        ros.Subscribe<JointStateMsg>(topicName, OnJointCommandReceived);
    }

    void OnJointCommandReceived(JointStateMsg command)
    {
        if (robotController != null)
        {
            // Set joint positions from command
            robotController.SetJointPositions(command.position);
        }
    }
}
```

## Sensor Simulation in Unity

### Camera Sensor Simulation

Unity's camera system can simulate various robotic cameras:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;
using Unity.Robotics.ROSConnection;

public class CameraSensor : MonoBehaviour
{
    [Header("Camera Configuration")]
    public int imageWidth = 640;
    public int imageHeight = 480;
    public float fieldOfView = 60f;
    public string topicName = "/camera/image_raw";
    public float publishRate = 30f; // Hz

    private Camera cam;
    private RenderTexture renderTexture;
    private Texture2D texture2D;
    private RosConnection ros;
    private float publishInterval;
    private float lastPublishTime;

    void Start()
    {
        // Setup camera
        cam = GetComponent<Camera>();
        if (cam == null)
        {
            cam = gameObject.AddComponent<Camera>();
        }

        cam.fieldOfView = fieldOfView;
        cam.backgroundColor = Color.black;
        cam.clearFlags = CameraClearFlags.SolidColor;

        // Create render texture for camera
        renderTexture = new RenderTexture(imageWidth, imageHeight, 24);
        cam.targetTexture = renderTexture;

        // Create texture for reading
        texture2D = new Texture2D(imageWidth, imageHeight, TextureFormat.RGB24, false);

        ros = RosConnection.GetOrCreateInstance();
        publishInterval = 1f / publishRate;
        lastPublishTime = 0f;
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
        // Set active render texture
        RenderTexture.active = renderTexture;

        // Read pixels from render texture
        texture2D.ReadPixels(new Rect(0, 0, imageWidth, imageHeight), 0, 0);
        texture2D.Apply();

        // Convert to byte array (ROS image format)
        byte[] imageData = texture2D.EncodeToJPG();

        // Create ROS Image message
        ImageMsg imageMsg = new ImageMsg
        {
            header = new HeaderMsg
            {
                stamp = new TimeStamp(Time.time),
                frame_id = "camera_optical_frame"
            },
            height = (uint)imageHeight,
            width = (uint)imageWidth,
            encoding = "rgb8",
            is_bigendian = 0,
            step = (uint)(imageWidth * 3), // 3 bytes per pixel (RGB)
            data = imageData
        };

        ros.Publish(topicName, imageMsg);
    }

    void OnDestroy()
    {
        if (renderTexture != null)
        {
            renderTexture.Release();
        }
    }
}
```

### Depth Sensor Simulation

Simulating depth cameras using Unity's depth rendering:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class DepthSensor : MonoBehaviour
{
    [Header("Depth Configuration")]
    public int width = 320;
    public int height = 240;
    public float fieldOfView = 60f;
    public float minDepth = 0.1f;
    public float maxDepth = 10.0f;
    public string topicName = "/camera/depth/image_raw";

    private Camera cam;
    private RenderTexture depthTexture;
    private Texture2D depthTexture2D;
    private RosConnection ros;

    void Start()
    {
        cam = GetComponent<Camera>();
        if (cam == null)
        {
            cam = gameObject.AddComponent<Camera>();
        }

        cam.fieldOfView = fieldOfView;
        cam.depthTextureMode = DepthTextureMode.Depth;

        // Create depth texture
        depthTexture = new RenderTexture(width, height, 24, RenderTextureFormat.RFloat);
        depthTexture.Create();
        cam.targetTexture = depthTexture;

        depthTexture2D = new Texture2D(width, height, TextureFormat.RFloat, false);
        ros = RosConnection.GetOrCreateInstance();
    }

    void Update()
    {
        CaptureAndPublishDepth();
    }

    void CaptureAndPublishDepth()
    {
        RenderTexture.active = depthTexture;
        depthTexture2D.ReadPixels(new Rect(0, 0, width, height), 0, 0);
        depthTexture2D.Apply();

        // Convert depth values to appropriate format
        Color32[] colors = depthTexture2D.GetPixels32();
        float[] depthValues = new float[colors.Length];

        for (int i = 0; i < colors.Length; i++)
        {
            // Convert from color format back to depth (this is a simplified example)
            depthValues[i] = colors[i].r; // In RFloat format, r component contains depth
        }

        // Create ROS Image message for depth
        ImageMsg depthMsg = new ImageMsg
        {
            header = new HeaderMsg
            {
                stamp = new TimeStamp(Time.time),
                frame_id = "camera_depth_frame"
            },
            height = (uint)height,
            width = (uint)width,
            encoding = "32FC1", // 32-bit float, single channel
            is_bigendian = 0,
            step = (uint)(width * 4), // 4 bytes per float
            data = new byte[depthValues.Length * 4] // Convert to bytes
        };

        // Convert float array to byte array
        for (int i = 0; i < depthValues.Length; i++)
        {
            byte[] floatBytes = System.BitConverter.GetBytes(depthValues[i]);
            for (int j = 0; j < 4; j++)
            {
                depthMsg.data[i * 4 + j] = floatBytes[j];
            }
        }

        ros.Publish(topicName, depthMsg);
    }

    void OnDestroy()
    {
        if (depthTexture != null)
        {
            depthTexture.Release();
        }
    }
}
```

### IMU Sensor Simulation

Simulating IMU data using Unity's physics system:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Sensor;

public class IMUSensor : MonoBehaviour
{
    [Header("IMU Configuration")]
    public string topicName = "/imu/data";
    public float publishRate = 100f; // Hz

    private RosConnection ros;
    private float publishInterval;
    private float lastPublishTime;
    private Rigidbody rb;

    void Start()
    {
        ros = RosConnection.GetOrCreateInstance();
        rb = GetComponent<Rigidbody>();
        publishInterval = 1f / publishRate;
        lastPublishTime = 0f;
    }

    void Update()
    {
        if (Time.time - lastPublishTime >= publishInterval)
        {
            PublishIMUData();
            lastPublishTime = Time.time;
        }
    }

    void PublishIMUData()
    {
        if (rb == null) return;

        // Get orientation (from transform)
        Quaternion orientation = transform.rotation;

        // Get angular velocity (from rigidbody)
        Vector3 angularVelocity = rb.angularVelocity;

        // Get linear acceleration (from rigidbody, removing gravity)
        Vector3 linearAcceleration = rb.velocity / Time.fixedDeltaTime;
        linearAcceleration -= Physics.gravity; // Remove gravity component

        // Create IMU message
        ImuMsg imuMsg = new ImuMsg
        {
            header = new HeaderMsg
            {
                stamp = new TimeStamp(Time.time),
                frame_id = "imu_link"
            },
            orientation = new GeometryMsgs.Quaternion
            {
                x = orientation.x,
                y = orientation.y,
                z = orientation.z,
                w = orientation.w
            },
            angular_velocity = new GeometryMsgs.Vector3
            {
                x = angularVelocity.x,
                y = angularVelocity.y,
                z = angularVelocity.z
            },
            linear_acceleration = new GeometryMsgs.Vector3
            {
                x = linearAcceleration.x,
                y = linearAcceleration.y,
                z = linearAcceleration.z
            }
        };

        ros.Publish(topicName, imuMsg);
    }
}
```

## Advanced Unity Robotics Features

### ML-Agents for Robot Learning

Unity's ML-Agents framework enables reinforcement learning for robotics:

```csharp
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using UnityEngine;

public class HumanoidLearningAgent : Agent
{
    [Header("Training Configuration")]
    public Transform target;
    public float moveSpeed = 2f;
    public float rotationSpeed = 100f;

    private Rigidbody rb;
    private Vector3 initialPosition;

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
        initialPosition = transform.position;
    }

    public override void OnEpisodeBegin()
    {
        // Reset agent position
        transform.position = initialPosition;
        rb.velocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;

        // Randomize target position
        if (target != null)
        {
            float randomX = Random.Range(-5f, 5f);
            float randomZ = Random.Range(-5f, 5f);
            target.position = new Vector3(randomX, 0.5f, randomZ);
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        // Agent position and rotation
        sensor.AddObservation(transform.position);
        sensor.AddObservation(transform.rotation);

        // Target position
        if (target != null)
        {
            sensor.AddObservation(target.position);
        }

        // Velocity
        sensor.AddObservation(rb.velocity);

        // Distance to target
        if (target != null)
        {
            sensor.AddObservation(Vector3.Distance(transform.position, target.position));
        }
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        // Actions: [0] move forward/back, [1] turn left/right
        float move = actions.ContinuousActions[0];
        float turn = actions.ContinuousActions[1];

        // Move the agent
        transform.Translate(Vector3.forward * move * moveSpeed * Time.deltaTime);
        transform.Rotate(Vector3.up, turn * rotationSpeed * Time.deltaTime);

        // Calculate distance to target
        float distanceToTarget = target != null ? Vector3.Distance(transform.position, target.position) : 0f;

        // Reward system
        if (distanceToTarget < 1f)
        {
            // Reached target
            SetReward(10f);
            EndEpisode();
        }
        else
        {
            // Negative reward based on distance
            SetReward(-distanceToTarget * 0.01f);
        }

        // Punish if agent falls or goes too far
        if (transform.position.y < -1f || distanceToTarget > 20f)
        {
            SetReward(-1f);
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        // Manual control for testing
        var continuousActionsOut = actionsOut.ContinuousActions;
        continuousActionsOut[0] = Input.GetAxis("Vertical"); // Forward/back
        continuousActionsOut[1] = Input.GetAxis("Horizontal"); // Turn
    }
}
```

### Physics Optimization for Robotics

Optimizing Unity's physics for robotic simulation:

```csharp
using UnityEngine;

public class PhysicsOptimizer : MonoBehaviour
{
    [Header("Physics Configuration")]
    public int fixedTimestep = 50; // Physics update rate (Hz)
    public int maxSubSteps = 10;
    public float solverIterations = 6;
    public float solverVelocityIterations = 1;

    void Start()
    {
        ConfigurePhysics();
    }

    void ConfigurePhysics()
    {
        // Set physics timestep
        Time.fixedDeltaTime = 1f / fixedTimestep;
        Time.maximumDeltaTime = 2f / fixedTimestep;
        Time.maximumParticleDeltaTime = 1f / fixedTimestep;

        // Physics settings
        Physics.defaultSolverIterations = (int)solverIterations;
        Physics.defaultSolverVelocityIterations = (int)solverVelocityIterations;
        Physics.maxSubsteps = maxSubSteps;

        // Bounce threshold (objects with relative velocity below this won't bounce)
        Physics.bounceThreshold = 2f;

        // Sleep threshold (objects with velocity below this may sleep)
        Physics.sleepThreshold = 0.005f;
    }
}
```

## Creating Realistic Environments

### Environment Design Principles

For humanoid robotics simulation, environments should:

1. **Mimic Real-World Spaces**: Offices, homes, factories
2. **Include Interactive Objects**: Furniture, tools, obstacles
3. **Support Various Scenarios**: Navigation, manipulation, interaction
4. **Provide Visual Variety**: Different textures, lighting conditions

### Example Environment Setup

```csharp
using UnityEngine;

public class EnvironmentSetup : MonoBehaviour
{
    [Header("Environment Configuration")]
    public GameObject[] furniturePrefabs;
    public GameObject[] obstaclePrefabs;
    public Material[] floorMaterials;
    public Light[] lightingRigs;

    [Header("Human Character")]
    public GameObject humanCharacterPrefab;

    void Start()
    {
        SetupEnvironment();
    }

    void SetupEnvironment()
    {
        // Apply random floor material
        if (floorMaterials.Length > 0)
        {
            Renderer floorRenderer = GetComponent<Renderer>();
            if (floorRenderer != null)
            {
                floorRenderer.material = floorMaterials[Random.Range(0, floorMaterials.Length)];
            }
        }

        // Place furniture randomly
        PlaceFurniture();

        // Add obstacles
        PlaceObstacles();

        // Add human characters for interaction
        SpawnHumanCharacters();
    }

    void PlaceFurniture()
    {
        foreach (GameObject prefab in furniturePrefabs)
        {
            if (prefab != null && Random.value > 0.7f) // 30% chance to place
            {
                Vector3 position = GetRandomValidPosition();
                GameObject furniture = Instantiate(prefab, position, Quaternion.identity);

                // Random rotation
                furniture.transform.rotation = Quaternion.Euler(0, Random.Range(0, 360), 0);
            }
        }
    }

    void PlaceObstacles()
    {
        foreach (GameObject prefab in obstaclePrefabs)
        {
            if (prefab != null && Random.value > 0.8f) // 20% chance to place
            {
                Vector3 position = GetRandomValidPosition();
                GameObject obstacle = Instantiate(prefab, position, Quaternion.identity);
            }
        }
    }

    void SpawnHumanCharacters()
    {
        if (humanCharacterPrefab != null && Random.value > 0.5f) // 50% chance
        {
            Vector3 position = GetRandomValidPosition();
            GameObject human = Instantiate(humanCharacterPrefab, position, Quaternion.identity);

            // Add some random behavior
            HumanBehavior behavior = human.AddComponent<HumanBehavior>();
            behavior.SetRandomDestination(GetRandomValidPosition());
        }
    }

    Vector3 GetRandomValidPosition()
    {
        // Define valid placement area
        float x = Random.Range(-10f, 10f);
        float z = Random.Range(-10f, 10f);
        float y = 0f; // Ground level

        // Check if position is valid (not inside other objects)
        Vector3 position = new Vector3(x, y, z);

        // Add ground offset
        position.y = GetGroundHeight(position);

        return position;
    }

    float GetGroundHeight(Vector3 position)
    {
        // Simple ground height calculation
        // In a real implementation, you might raycast or use a heightmap
        return 0f;
    }
}

public class HumanBehavior : MonoBehaviour
{
    public float walkSpeed = 1f;
    public Vector3 destination;

    void Update()
    {
        if (Vector3.Distance(transform.position, destination) > 0.5f)
        {
            transform.position = Vector3.MoveTowards(transform.position, destination, walkSpeed * Time.deltaTime);
            transform.LookAt(destination);
        }
        else
        {
            // Reached destination, set new one
            SetRandomDestination(GetRandomPosition());
        }
    }

    public void SetRandomDestination(Vector3 newDestination)
    {
        destination = newDestination;
    }

    Vector3 GetRandomPosition()
    {
        float x = Random.Range(-8f, 8f);
        float z = Random.Range(-8f, 8f);
        return new Vector3(x, 0, z);
    }
}
```

## Performance Optimization

### Rendering Optimization

For large-scale robotics simulation:

```csharp
using UnityEngine;

public class RenderingOptimizer : MonoBehaviour
{
    [Header("LOD Configuration")]
    public float lodDistance = 20f;
    public int lodCount = 3;

    [Header("Culling Configuration")]
    public float cullingDistance = 50f;

    void Start()
    {
        OptimizeRendering();
    }

    void OptimizeRendering()
    {
        // Reduce shadow resolution for performance
        QualitySettings.shadowResolution = ShadowResolution.Low;

        // Reduce shadow distance
        QualitySettings.shadowDistance = 20f;

        // Use faster shader variants
        Shader.globalMaximumLOD = 300; // Lower is faster

        // Configure occlusion culling if needed
        // This should be done in the editor for static objects
    }

    void Update()
    {
        // Dynamic LOD for robots based on distance
        ApplyDynamicLOD();
    }

    void ApplyDynamicLOD()
    {
        // Find all robot objects in the scene
        RobotController[] robots = FindObjectsOfType<RobotController>();

        foreach (RobotController robot in robots)
        {
            float distance = Vector3.Distance(robot.transform.position, Camera.main.transform.position);

            if (distance > lodDistance)
            {
                // Reduce rendering quality for distant robots
                Renderer[] renderers = robot.GetComponentsInChildren<Renderer>();
                foreach (Renderer renderer in renderers)
                {
                    renderer.enabled = false; // Or use simpler LOD
                }
            }
        }
    }
}
```

### Physics Optimization

```csharp
using UnityEngine;

public class PhysicsOptimizer : MonoBehaviour
{
    [Header("Physics Settings")]
    public float fixedTimestep = 0.02f; // 50 Hz
    public int solverIterations = 6;
    public int solverVelocityIterations = 1;
    public float sleepThreshold = 0.005f;
    public float bounceThreshold = 2f;

    void Start()
    {
        ConfigurePhysics();
    }

    void ConfigurePhysics()
    {
        Time.fixedDeltaTime = fixedTimestep;
        Time.maximumDeltaTime = fixedTimestep * 2f;

        Physics.defaultSolverIterations = solverIterations;
        Physics.defaultSolverVelocityIterations = solverVelocityIterations;
        Physics.sleepThreshold = sleepThreshold;
        Physics.bounceThreshold = bounceThreshold;

        // Optimize for robotic simulation
        Physics.queriesHitTriggers = false; // Disable trigger queries if not needed
        Physics.queriesHitBackfaces = QueryTriggerInteraction.Ignore; // Optimize raycasts
    }
}
```

## Integration with Real Hardware

### Simulation-to-Reality Transfer

Preparing Unity simulations for real-world deployment:

#### Domain Randomization
```csharp
using UnityEngine;

public class DomainRandomizer : MonoBehaviour
{
    [Header("Randomization Ranges")]
    public Vector2 massRange = new Vector2(0.8f, 1.2f);
    public Vector2 frictionRange = new Vector2(0.1f, 1.0f);
    public Vector2 lightIntensityRange = new Vector2(0.5f, 1.5f);

    void Start()
    {
        RandomizeEnvironment();
    }

    void RandomizeEnvironment()
    {
        // Randomize physics parameters
        RandomizePhysics();

        // Randomize visual parameters
        RandomizeVisuals();

        // Randomize lighting
        RandomizeLighting();
    }

    void RandomizePhysics()
    {
        // Apply random mass multipliers to rigidbodies
        Rigidbody[] rigidbodies = FindObjectsOfType<Rigidbody>();
        foreach (Rigidbody rb in rigidbodies)
        {
            float massMultiplier = Random.Range(massRange.x, massRange.y);
            rb.mass *= massMultiplier;
        }

        // Randomize friction
        PhysicMaterial[] materials = FindObjectsOfType<PhysicMaterial>();
        foreach (PhysicMaterial material in materials)
        {
            float friction = Random.Range(frictionRange.x, frictionRange.y);
            material.dynamicFriction = friction;
            material.staticFriction = friction;
        }
    }

    void RandomizeVisuals()
    {
        // Randomize textures, colors, lighting conditions
        Renderer[] renderers = FindObjectsOfType<Renderer>();
        foreach (Renderer renderer in renderers)
        {
            // Apply random color variations
            Material material = renderer.material;
            Color randomColor = material.color;
            randomColor.r = Mathf.Clamp01(randomColor.r + Random.Range(-0.1f, 0.1f));
            randomColor.g = Mathf.Clamp01(randomColor.g + Random.Range(-0.1f, 0.1f));
            randomColor.b = Mathf.Clamp01(randomColor.b + Random.Range(-0.1f, 0.1f));
            material.color = randomColor;
        }
    }

    void RandomizeLighting()
    {
        Light[] lights = FindObjectsOfType<Light>();
        foreach (Light light in lights)
        {
            float intensity = Random.Range(lightIntensityRange.x, lightIntensityRange.y);
            light.intensity = intensity;
        }
    }
}
```

## Debugging and Troubleshooting

### Common Unity Robotics Issues

#### Performance Issues
- **High CPU usage**: Reduce physics update rate, optimize collision detection
- **High GPU usage**: Use LOD, reduce shadow quality, optimize materials
- **Memory leaks**: Check for object pooling, destroy unused objects

#### Physics Issues
- **Unstable simulation**: Adjust solver iterations, reduce time step
- **Objects falling through each other**: Check collision layers, increase solver iterations
- **Joints behaving unexpectedly**: Verify joint configuration, check mass ratios

#### ROS Communication Issues
- **Connection failures**: Check IP addresses, firewall settings, port availability
- **Message delays**: Optimize publish rates, reduce message sizes
- **Synchronization issues**: Use ROS time consistently

### Debugging Tools

#### Physics Debugging
```csharp
using UnityEngine;

public class PhysicsDebugger : MonoBehaviour
{
    [Header("Debug Configuration")]
    public bool showForces = true;
    public bool showColliders = true;
    public bool showVelocities = true;

    void OnDrawGizmos()
    {
        if (showColliders)
        {
            DrawColliderGizmos();
        }

        if (showForces)
        {
            DrawForceGizmos();
        }

        if (showVelocities)
        {
            DrawVelocityGizmos();
        }
    }

    void DrawColliderGizmos()
    {
        Collider[] colliders = GetComponentsInChildren<Collider>();
        foreach (Collider col in colliders)
        {
            if (col.enabled)
            {
                Gizmos.color = Color.blue;
                Gizmos.DrawWireCube(col.bounds.center, col.bounds.size);
            }
        }
    }

    void DrawForceGizmos()
    {
        Rigidbody rb = GetComponent<Rigidbody>();
        if (rb != null)
        {
            Gizmos.color = Color.red;
            Vector3 force = rb.velocity * rb.mass; // Approximate force
            Gizmos.DrawRay(transform.position, force.normalized * 2f);
        }
    }

    void DrawVelocityGizmos()
    {
        Rigidbody rb = GetComponent<Rigidbody>();
        if (rb != null)
        {
            Gizmos.color = Color.green;
            Gizmos.DrawRay(transform.position, rb.velocity * 0.1f);
        }
    }
}
```

## Best Practices for Unity Robotics

### Project Organization
- Use clear folder structures for different robot components
- Separate simulation-specific code from general robot code
- Version control for both Unity assets and scripts
- Document all robot configurations and parameters

### Performance Considerations
- Use object pooling for frequently created/destroyed objects
- Optimize mesh complexity for physics calculations
- Use appropriate LOD for distant objects
- Consider multi-threading for sensor processing

### Safety and Validation
- Implement emergency stop mechanisms in simulation
- Validate simulation results against physical models
- Include sensor noise models for realism
- Test controllers in simulation before hardware deployment

## Practical Exercise: Complete Unity Robotics Setup

Let's create a complete Unity scene that demonstrates all concepts:

### 1. Create a Unity scene with a humanoid robot
- Import robot model (or create simple geometric representation)
- Add necessary components (Rigidbodies, Joints, Colliders)
- Configure physics properties

### 2. Set up ROS communication
- Add ROS-TCP-Connector
- Create joint state publisher/subscriber
- Implement sensor simulation (camera, IMU)

### 3. Create a simple environment
- Add ground plane
- Include some obstacles
- Set up lighting

### 4. Implement basic control
- Create simple controller
- Add keyboard input for testing
- Implement basic behaviors

## Summary and Looking Ahead

In Week 6, you've learned how to integrate Unity with robotics systems:

1. **Unity Robotics Ecosystem**: Understanding the available tools and packages
2. **Robot Modeling**: Creating robot models with proper physics and joints
3. **ROS Communication**: Setting up communication between Unity and ROS systems
4. **Sensor Simulation**: Implementing realistic sensor models in Unity
5. **Advanced Features**: ML-Agents for learning and optimization techniques
6. **Environment Creation**: Building realistic simulation environments
7. **Performance Optimization**: Techniques for efficient simulation
8. **Hardware Integration**: Preparing for real-world deployment

### Exercises for Week 6

1. Create a simple humanoid robot model in Unity with proper joints and physics
2. Set up ROS communication using ROS-TCP-Connector
3. Implement camera and IMU sensor simulation
4. Create a basic environment with obstacles
5. Implement a simple controller to move the robot
6. Add domain randomization to your simulation
7. Create a multi-agent scenario with multiple robots
8. Optimize your simulation for performance

### Looking Ahead to Week 7-9

Weeks 7-9 will focus on NVIDIA Isaac, exploring how GPU computing can accelerate robotics perception, planning, and control systems. You'll learn about Isaac Sim, Isaac ROS, and how to leverage NVIDIA's computing platforms for advanced humanoid robotics applications.

[Continue to Module 3: NVIDIA Isaac for Humanoid Control](../module-3-nvidia-isaac/index.md)

## References and Resources

- Unity Robotics: https://unity.com/solutions/robotics
- ROS-TCP-Connector: https://github.com/Unity-Technologies/ROS-TCP-Connector
- ML-Agents: https://github.com/Unity-Technologies/ml-agents
- Unity Robotics Package: https://docs.unity3d.com/Packages/com.unity.robotics@latest
- NVIDIA Isaac: https://developer.nvidia.com/isaac