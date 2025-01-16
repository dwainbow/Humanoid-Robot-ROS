# ROS Project Setup Guide

## Introduction
Welcome to the **Humanoid Robot Repo**! 

Follow the steps below to set up your environment, build the workspace, and run the project.

---

## Prerequisites
Before cloning the repository, make sure you have the following installed on your system:
- **ROS Noetic**: [Installation Guide](http://wiki.ros.org/noetic/Installation)
- **Python 3** (for any Python-based scripts)
- **Git** (to clone the repository)
- **Joystick Tools** (optional for testing the controller):
  ```bash
  sudo apt-get install joystick


## 1. Install Dependencies 
```bash
  cd ~/Humanoid-Robot-ROS
  rosdep install --from-paths src --ignore-src -r -y
```
If encounter issies with missing rosdep, intitialize it with 
```bash
  sudo rosdep init
  rosdep update
```

## 2. Build Workspace
Build the ROS workspace
```bash
  catkin_make
```

Source worskpace to setup the environment
```bash
  source devel/setup.bash
```

To ensure this is automatic every time, add the following line to your ~/.bashrc:
```bash
  echo "source ~/Humanoid-Robot-ROS/devel/setup.bash" >> ~/.bashrc
  source ~/.bashrc
```
## 4. Verify ROS Environment 
Check that your ROS environment is correctly set up:
```bash
  echo $ROS_PACKAGE_PATH
```

