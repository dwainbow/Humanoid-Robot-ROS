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
- **Joystick Tools** (for testing the controller):
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
```
./build_project.sh
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

# Test Motors
To test the Dynamixel Motors follow these steps :
## 1. Get Motor Information 
The motors have unique parameters (ID, Baud Rate). To get this information in a terminal run 
```bash
  roscore 
```
In a seperate terminal run: 
```bash
  ./scan_motors.sh 
```
This will scan for motors with different IDs and Baud Rates. Generally the baude rates are either 57600 or 3000000. If not update the file src/motor_controller/src/scan_motors.cpp and add additional baud rates to scan.
## 2. Setup Limb Cluster
Once you have identified the motor parameters, head over to src/motor_controller/motor_node.cpp. There will be functions that return a cluster of motors. Modify these methods to test them however you please.
Each cluster should listen to a specific button on the controller. Look at src/robot_controller/src/Controller.cpp for button names. 

## 3. Compile the project 
Once setup, compile the project by running the following command: 
```bash
  ./build_project.sh  
```
This will generate updated versions of the code to be used 

## 4. Launch Controller Node 
The motors need to listen to the controller inputs. So to set this up, run this command: 
```bash
  ./launch_controller_node.sh
```

## 5. Launch Motor Node 
In a seperate terminal, launch the motor node that reads information from the controller
```bash
  ./launch_motor_node.sh 
```
Now you should be able to control the motors via controller



