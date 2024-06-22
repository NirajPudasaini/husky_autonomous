# Husky SLAM Implementation

This repository outlines the setup and instructions for implementing SLAM algorithms on a Husky robot using ROS and Catkin. Initially focusing on simulations, this setup is tested on Ubuntu 20.04.6 LTS with ROS Noetic.
## Prerequisites 
### Setup - ROS

1. **Install ROS Noetic**: Ensure you are using Ubuntu 20.04 and install ROS Noetic. You can follow the detailed installation instructions provided on the [ROS Wiki](http://wiki.ros.org/noetic/Installation/Ubuntu).

2. **Setup Your ROS Environment**: Configure your shell to source the ROS environment automatically.

```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

3. **Install Dependencies**: Install additional tools and dependencies.

```bash
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
```

   

### Setup - Catkin Workspace

1. **Create Catkin Workspace**: Create a workspace for your ROS packages. This is where you will build and manage your ROS packages.
```bash
mkdir -p ~/husky_ws/src
cd ~/husky_ws
catkin_make
```

2. **Source the Workspace**: Source your new setup script to overlay this workspace on top of your environment.

```bash
echo "source ~/husky_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
markdown
```


### Setup - Husky Packages

1. **Clone Husky Repositories**: Husky-related packages into your workspace.
rosdep install --from-paths src --ignore-src -r -yd packages into your workspace.

```bash
cd ~/husky_ws/src
git clone https://github.com/husky/husky.git
```

2. **Install Dependencies**: Install all required dependencies for the Husky packages.

```bash
cd ~/husky_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. **Build the Workspace**: Compile all packages in your workspace.

```bash
catkin build
```

sanity check (to be written)


## Husky Frontier Exploration Demo Setup

This section outlines the setup for the Husky Frontier Exploration demo. Follow these steps to simulate the exploration tasks using the Husky robot. For more details, refer to the official [Husky Frontier Exploration Guide](https://www.clearpathrobotics.com/assets/guides/melodic/husky/HuskyFrontiers.html).

### Prerequisites

Before starting, ensure ROS Noetic is installed and configured as outlined in the initial setup. Additionally, install `roslint` and `pcl` which are required for the demo:

```bash
# Install roslint
sudo apt-get install ros-noetic-roslint

# Install Point Cloud Library (PCL)
sudo apt-get install libpcl-dev
```

### Clone the Repository

Clone the `frontier_exploration` repository into your workspace:

```bash
cd ~/husky_ws/src
git clone https://github.com/paulbovbel/frontier_exploration.git
```

### Install Dependencies

Install all required dependencies using `rosdep`. This command is crucial as it ensures all necessary ROS dependencies are met:

```bash
cd ~/husky_ws
rosdep install --from-paths src --ignore-src -r -y
```

### Build the Workspace

```bash
catkin build
```

### Source the Workspace
```bash
source ~/husky_ws/devel/setup.bash
```

### Running the Simulation

#### Terminal 1: Start the Husky Simulation Environment

In the first terminal, start the Clearpath-configured Husky simulation in Gazebo, which includes a LiDAR sensor for the robot:

```bash
export HUSKY_LMS1XX_ENABLED=1; roslaunch husky_gazebo husky_playpen.launch
```

## Running the Husky Frontier Exploration Demo

Once the installation and setup are complete, you can run the Husky Frontier Exploration demo. You will need to open three separate terminal windows to launch different parts of the simulation. Here are the steps for each:

### Terminal 1: Start the Husky Simulation Environment

In the first terminal, start the Clearpath-configured Husky simulation in Gazebo, which includes a LiDAR sensor for the robot:

```bash
export HUSKY_LMS1XX_ENABLED=1; roslaunch husky_gazebo husky_playpen.launch
```
![Husky Gazebo Simulation](/images/husky_gazebo.png)

### Terminal 2: Launch the RViz Visualizer

In the second terminal, start the RViz visualizer. This tool provides a graphical interface to visualize the robot and its sensors in real-time:

```bash
roslaunch husky_viz view_robot.launch
```
![Husky RViz Simulation](/images/husky_rviz.png)

### Terminal 3: Start the Frontier Exploration Demo

In the third terminal, launch the frontier exploration demo. This will initiate the autonomous exploration behavior:

```bash
roslaunch husky_navigation exploration_demo.launch
```
![Husky Exploration Visualized](/images/exploration.png)
