---
sidebar_position: 3
title: "Appendix C: Software Installation Guide"
---

# Appendix C: Software Installation Guide

This guide provides step-by-step instructions for installing and configuring the software tools needed for this course. The setup process varies depending on your operating system and specific requirements. Follow these instructions carefully to ensure a proper development environment.

## Prerequisites

Before beginning the installation process, ensure your system meets the following requirements:

### System Requirements
- **Operating System**: Ubuntu 22.04 LTS, Windows 10/11 (with WSL2), or macOS (with Docker)
- **RAM**: Minimum 8GB, Recommended 16GB or more
- **Storage**: Minimum 50GB free space for development tools
- **Processor**: Multi-core processor with SSE4.1 support
- **Graphics**: GPU with CUDA support (for NVIDIA Isaac) is recommended

### Required Accounts
- GitHub account for accessing repositories
- NVIDIA Developer account (for Isaac Sim, optional)
- DockerHub account (for container images, optional)

## Ubuntu 22.04 LTS Installation Guide

### 1. Update System Packages
```bash
sudo apt update
sudo apt upgrade -y
```

### 2. Install ROS 2 Humble Hawksbill
```bash
# Set locale
locale  # check for UTF-8
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US.UTF-8

# Add ROS 2 apt repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe

# Add the ROS 2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add the repository to your sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update apt and install ROS 2 packages
sudo apt update
sudo apt install -y ros-humble-desktop
sudo apt install -y python3-argcomplete
sudo apt install -y ros-dev-tools
```

### 3. Install ROS 2 Colcon Build Tool
```bash
sudo apt install python3-colcon-common-extensions
```

### 4. Source ROS 2 Environment
Add the following line to your `~/.bashrc` file:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 5. Install Python Development Tools
```bash
sudo apt install python3-pip python3-dev
pip3 install -U argcomplete
```

### 6. Install Git and Version Control
```bash
sudo apt install git git-lfs
```

### 7. Install Gazebo Garden
```bash
# Add Gazebo repository
sudo curl -sSL http://get.gazebosim.org | sudo bash
sudo apt install gz-harmonic
```

### 8. Install Additional Development Tools
```bash
sudo apt install build-essential cmake pkg-config
sudo apt install libeigen3-dev libopencv-dev
sudo apt install python3-catkin-tools python3-osrf-pycommon
```

## Python Environment Setup

### 1. Install Python Virtual Environment Tools
```bash
sudo apt install python3-venv python3-pip
```

### 2. Create Project Virtual Environment
```bash
mkdir -p ~/ros2_workspace/src
cd ~/ros2_workspace
python3 -m venv venv
source venv/bin/activate
pip install --upgrade pip setuptools
```

### 3. Install Python Dependencies for Robotics
```bash
pip install numpy scipy matplotlib pandas
pip install opencv-python open3d
pip install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip install transformers datasets
pip install rclpy rospkg catkin_pkg
```

## NVIDIA Isaac Setup

### 1. Install NVIDIA Drivers
```bash
# Check if NVIDIA GPU is detected
lspci | grep -i nvidia

# Install NVIDIA drivers (if not already installed)
sudo apt install nvidia-driver-535
sudo reboot
```

### 2. Install CUDA Toolkit
```bash
# Download and install CUDA toolkit
wget https://developer.download.nvidia.com/compute/cuda/12.3.0/local_installers/cuda_12.3.0_545.23.06_linux.run
sudo sh cuda_12.3.0_545.23.06_linux.run
```

### 3. Install cuDNN
```bash
# Download cuDNN from NVIDIA Developer website (requires account)
# Follow NVIDIA's installation guide for your CUDA version
```

### 4. Install Isaac Sim (via Omniverse)
1. Download NVIDIA Omniverse Launcher from developer.nvidia.com
2. Install Omniverse Launcher
3. Add Isaac Sim extension through the launcher
4. Configure the application settings

## Unity Robotics Setup

### 1. Install Unity Hub
1. Download Unity Hub from unity.com/get-unity/download
2. Install Unity Hub
3. Use Unity Hub to install Unity 2022.3 LTS or later

### 2. Install Unity Robotics Packages
1. Open Unity Hub
2. Install ROS-TCP-Connector package
3. Install ML-Agents package (if needed)
4. Install XR packages if needed

### 3. Configure Unity for Robotics
1. Create a new 3D project
2. Import robotics-specific assets
3. Configure build settings for robotics applications

## Docker Setup (Alternative Installation Method)

### 1. Install Docker
```bash
# Remove old versions
sudo apt remove docker docker-engine docker.io containerd runc

# Install Docker using the official script
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh

# Add current user to docker group
sudo usermod -aG docker $USER
newgrp docker
```

### 2. Install Docker Compose
```bash
sudo apt install docker-compose-plugin
```

### 3. Create Docker Environment for Robotics
```bash
# Create project directory
mkdir ~/robotics-docker
cd ~/robotics-docker

# Create docker-compose.yml file
cat > docker-compose.yml << EOF
version: '3.8'
services:
  ros2-devel:
    image: osrf/ros:humble-desktop-full
    container_name: ros2_humble
    privileged: true
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ./workspace:/home/user/workspace
    network_mode: host
    stdin_open: true
    tty: true
    command: bash
EOF

# Start the container
docker compose up -d
```

## Development Environment Setup

### 1. Create ROS 2 Workspace
```bash
mkdir -p ~/robotics_ws/src
cd ~/robotics_ws
colcon build
source install/setup.bash
```

### 2. Install Code Editor
```bash
# Install VS Code
curl -fsSL https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor | sudo tee /etc/apt/trusted.gpg.d/microsoft.gpg > /dev/null
echo "deb [arch=amd64,arm64,armhf signed-by=/etc/apt/trusted.gpg.d/microsoft.gpg] https://packages.microsoft.com/repos/code stable main" | sudo tee /etc/apt/sources.list.d/vscode.list
sudo apt update
sudo apt install code

# Install useful extensions
code --install-extension ms-iot.vscode-ros
code --install-extension ms-python.python
code --install-extension ms-vscode.cpptools
```

### 3. Install Additional Tools
```bash
# Install system monitoring tools
sudo apt install htop iotop nethogs

# Install version control tools
sudo apt install gitk git-gui meld

# Install network tools
sudo apt install net-tools nmap ssh
```

## Verification Steps

### 1. Verify ROS 2 Installation
```bash
source /opt/ros/humble/setup.bash
ros2 --version
ros2 topic list
```

### 2. Verify Gazebo Installation
```bash
gz sim --version
```

### 3. Test Basic ROS 2 Commands
```bash
# Terminal 1: Start talker
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker

# Terminal 2: Start listener
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

### 4. Verify Python Environment
```bash
source ~/robotics_ws/install/setup.bash
python3 -c "import rclpy; print('rclpy imported successfully')"
python3 -c "import torch; print(f'PyTorch version: {torch.__version__}')"
python3 -c "import cv2; print(f'OpenCV version: {cv2.__version__}')"
```

## Troubleshooting Common Issues

### 1. Permission Issues
If you encounter permission errors:
```bash
# Add user to necessary groups
sudo usermod -a -G dialout $USER
sudo usermod -a -G video $USER
```

### 2. Environment Setup Issues
If ROS 2 commands are not found:
```bash
# Check if sourced properly
echo $ROS_DISTRO
# Should output 'humble'

# If not, source the environment
source /opt/ros/humble/setup.bash
```

### 3. Network Configuration
For multi-machine ROS 2 communication:
```bash
# Check network configuration
echo $ROS_DOMAIN_ID
echo $ROS_LOCALHOST_ONLY

# Set domain ID for your network
export ROS_DOMAIN_ID=42
```

### 4. GPU/CUDA Issues
Verify CUDA installation:
```bash
nvidia-smi
nvcc --version
python3 -c "import torch; print(torch.cuda.is_available())"
```

## Post-Installation Configuration

### 1. Create Startup Script
Create a script to automatically source ROS 2 environment:
```bash
cat > ~/ros2_setup.sh << EOF
#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/robotics_ws/install/setup.bash
export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:~/robotics_ws/install/share
EOF

chmod +x ~/ros2_setup.sh
```

### 2. Add to Shell Profile
Add to your `~/.bashrc`:
```bash
echo "source ~/ros2_setup.sh" >> ~/.bashrc
```

### 3. Create Workspace Aliases
Add to `~/.bash_aliases`:
```bash
echo "alias cw='cd ~/robotics_ws'" >> ~/.bash_aliases
echo "alias sb='source ~/robotics_ws/install/setup.bash'" >> ~/.bash_aliases
echo "alias cb='cd ~/robotics_ws && colcon build --symlink-install'" >> ~/.bash_aliases
```

## Optional: Development Container Setup

For a consistent development environment across different systems:

```bash
# Create .devcontainer directory
mkdir -p ~/robotics_ws/.devcontainer

# Create devcontainer.json
cat > ~/robotics_ws/.devcontainer/devcontainer.json << EOF
{
    "name": "ROS 2 Humble Development",
    "image": "osrf/ros:humble-desktop-full",
    "runArgs": ["--network=host", "--privileged"],
    "mounts": [
        "source=${localWorkspaceFolder},target=/workspace,type=bind,consistency=cached",
        "source=/tmp/.X11-unix,target=/tmp/.X11-unix,type=bind,consistency=cached"
    ],
    "workspaceFolder": "/workspace",
    "workspaceMount": "source=${localWorkspaceFolder},target=/workspace,type=bind,consistency=cached",
    "customizations": {
        "vscode": {
            "extensions": [
                "ms-iot.vscode-ros",
                "ms-python.python",
                "ms-vscode.cpptools"
            ]
        }
    },
    "postCreateCommand": "pip3 install --upgrade pip setuptools && pip3 install rclpy rospkg"
}
EOF
```

## Verification and Testing

After completing the installation, run these verification tests:

### 1. Basic ROS 2 Test
```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker &
sleep 2
ros2 run demo_nodes_py listener &
sleep 10
pkill talker listener
```

### 2. Gazebo Test
```bash
gz sim --verbose
```

### 3. Python Robotics Libraries Test
```bash
python3 -c "
import rclpy
import cv2
import numpy as np
import torch
print('All libraries imported successfully!')
print(f'PyTorch CUDA available: {torch.cuda.is_available()}')
"
```

## Common Installation Errors and Solutions

### Error: Cannot locate ROS_DISTRO
**Solution**: Ensure ROS 2 is properly sourced:
```bash
source /opt/ros/humble/setup.bash
echo $ROS_DISTRO  # Should output 'humble'
```

### Error: No module named 'rclpy'
**Solution**: Check Python path and ROS 2 installation:
```bash
source /opt/ros/humble/setup.bash
python3 -c "import sys; print(sys.path)"
python3 -c "import rclpy"
```

### Error: Gazebo crashes or doesn't start
**Solution**: Check graphics drivers and OpenGL support:
```bash
glxinfo | grep -i opengl
nvidia-smi
```

## Next Steps

Once your software environment is properly installed and verified:

1. **Create your first ROS 2 package** using the tutorial in Module 1
2. **Run your first simulation** using Gazebo as described in Module 2
3. **Experiment with Isaac Sim** if you have NVIDIA hardware
4. **Practice with the example code** provided in this course

Your development environment is now ready for the Physical AI and Humanoid Robotics course. Make sure to restart your terminal or source your environment files before starting the course modules.