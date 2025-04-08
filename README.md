# Tiago_robot

This document provides the commands and file structure to install and view the `robot_description` of `tiago` by PAL robotics with Pinocchio Rigid Body Dynamics Library.

## Prerequisites

- Ubuntu 24.04 LTS
- ROS 2 Jazzy
- Gazebo Harmonic (gz-sim8)
- Tiago Hamonic model
- Pinocchio

## Prerequisites 
- Should have ROS2 Jazzy installed. You can refer [here](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html) for the setup. Follow the same steps.
- Source install gazebo Harmonic. This way if you encounter any issue you can just delete the workspace and do clean install again. You can refer [here](https://gazebosim.org/docs/harmonic/install_ubuntu_src/) for the setup.
- Install Tiago from this [repo](https://github.com/Tiago-Harmonic/tiago_harmonic) for simulation. Follow the steps given in their readme.
- Install Pinocchio
```
 pip install pin
```


## Setup the workspace
```
cd ~/tiago_ws
source /opt/ros/jazzy/setup.bash
```

## Usage

```
ros2 launch tiago_gazebo tiago_gazebo.launch.py is_public_sim:=True world_name:=house # or empty
```


## visualize the robot model in RViz

```
# Run simulation and then open another terminal
rviz2
```
Now click on add, then `RobotModel` in 'by display type' bar which will add that topic in rviz panel.
It will look like this, Also select `base_link` in `Fixed frame`. ![image](https://github.com/user-attachments/assets/0e822556-a12d-4d06-baf4-d373f396c61f)








But Why is rviz empty? Because we haven't yet defined topic. Just Extend the RobotModel, click of Description topic and choose `/robot_description`.
Voila! your robot will appear in rviz.
Like this.![image](https://github.com/user-attachments/assets/098638a8-b09d-4efa-ba88-50e3ba914259)



## Pinocchio Implementation

Perform 4 tasks mainly:
- Creates Pinocchio Model
- Perform forward kinematics and get a transform.
- Get a Jacobian at a specific frame.
- Check collisions between 2 links

```bash
# clone this repo and run pinocchio_analyzer.py
git clone https://github.com/ChinmayMundane/Tiago_robot.git
cd Tiago_robot
python3 pinocchio_analyzer.py
```
## Result
You will see Result like this
![image](https://github.com/user-attachments/assets/034839f4-2b89-4344-8457-f219b4cfb012)

## Issue

Now, If you look closely at the ouput, you will see 0 collision pairs even when we have them in our urdf. This is because when launching the world, collision geometry is hidden which makes it unable to detect any collision between pairs although you can add one manually which I did.

![image](https://github.com/user-attachments/assets/46137247-1dae-420d-940b-f19ec7cb4f79)


## Author Simulation

This is not the issue on my side only, The original author also had similar warnings when I saw their running example which means it must be some native issue in harmonic.
![image](https://github.com/user-attachments/assets/cd76656b-df1a-4700-ba9d-1897ba90dadc)



Youtube demonstration is available here.
- Tasks : https://youtu.be/PaRjhV3crAc


