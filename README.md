# imcpy_ros_bridge
This stack comprises a bridge interface between IMC (Interface Message Control), ROS (Robot Operating System), and the behaviour trees, facilitating control, communication and data exchange in the robot mission.

## Installation

### Prerequisites
- LSTS Toolchain components, for controlling the AUV:
  - Dune: for setting the comunication and send commands to the AUV. Installation and usage instructions are available [here](github.com/LSTS/dune/wiki).
  - Neptus: Dune's graphical interface. Installation and usage instructions are available [here](github.com/LSTS/neptus/wiki).
- UNavSim: for simulating realistic renderings of underwater environments, and getting sensor recordings such as camera, segmentation and IMU. Installation instructions available [here](https://github.com/open-airlab/UNav-Sim).
- ROS2. These repos have been tested under ROS2 Foxy and Humble.
- py_trees_ros. Installation instructions available [here](https://github.com/splintered-reality/py_trees_ros).


### Clone the Repository
`imcpy_ros_bridge` has been tested on Ubuntu 20 and 22, and with ROS2 Foxy and Humble. You can clone this repository in your colcon workspace to compile these ROS stacks as follows:
```bash
cd $HOME/<path-to-your-colcon-ws>/src
git clone https://github.com/olayasturias/imcpy_ros_bridge
cd ..
colcon build
```

## Usage
You first need to run Dune to connect with the robot (and optionally, Neptus for getting a graphical interface). Once Dune is running, you can get the behaviour trees to work. The procedure goes as follows: 

1. **Dune**

First, cd to your `dune/build` directory and run:

```
./dune -c lauv-simulator-1 -p Simulation
```
2. **Neptus**
Then, from the directory where you cloned Neptus, execute Neptus as follows:

```
./neptus
```
In the Neptus interface, connecto to the `lauv-simulator-1` vehicle to see its state.

3. **Move the robot** with the behaviour trees:

```
ros2 launch imcpy_trees square_trajectory_launch.py
```