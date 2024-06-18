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
catkin_make
```

sanity check (to be written)

```bash
./sanitycheck.sh
```
