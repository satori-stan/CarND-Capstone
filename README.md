# CarND-Capstone-Project

Self-Driving Car Engineer Nanodegree Program

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9).

## Goals

In this project, the goal is to integrate the knowledge of the previous terms to have a self-driving car follow a given route, following a given trajectory that includes stopping at traffic lights and staying in a given lane. This is achieved by programming several of the car's sub-systems to:
1. Identify obstacles in the route of a self-driving car
2. Calculate and send the list of target waypoints that can help the car complete its route, avoiding obstacles
3. Implement throttle, break and steering control
4. Connect everything through the ROS (Robot Operating System) platform

### Here I will consider each point individually and explain how I addressed them in my implementation

#### Identify obstacles in the route of a self-driving car

The obstacle identification in this stage consists in identifying traffic lighs and the traffic "command" (traffic light color, or state) conveyed by them along the planned route. The project covers only the identification of traffic light state without accounting for turn-lane traffic lights, which means we expect all traffic lights in the direction of travel to share the same state.

The traffic light state identification is done by reading the image feed from the car's front camera and passing it to a deep neural network to guess the state of the traffic light. The state can be one of red, yellow, green or unknown (for when no traffic light can be found or its state cannot be identified). The image is only processed if we can find in the list of traffic lights in the map, one that is close to our position.

The selected neural network's architecture is MobileNet, taken from the Keras framework using a TensorFlow backend. The model was trained over several thousand images of each of the possible states. Since the work was done in a Udacity-provided web workspace, disk-space constraints prevented from using a larger dataset.

MobileNet was selected as the architecture for being a fast neural network to train and execute with reasonable accuracy (> 70%). It was developed for use in low power devices such as mobile phones, which is relatable to the kind of devices that would be available to run the logic in a self-driving car. Other architectures were also explored such as MobileNetV2 and ShuffleNet, but the project is restricted for submission using only the libraries available in the provided system image.

An additional advantage of MobileNet is the size of the model in terms of the number of operations, and thus, parameters required. Other, more common, architectures such as VGG or ResNet have larger parameter spaces; which result in model files of several hundred megabytes in size. With MobileNet, a model file is just south of 150Mb. Although this size is still too large to be uploaded to GitHub, saving only the parameter weights results in a file 50Mb in size. For this reason, the model is recompiled every time the traffic light classification program is started.

Once the traffic light's state is identified, the stop line closest to the traffic light (in the direction of travel) is identified and published so that the trajectory can be updated to stop at the line if required. Real-life logic would take into account not just the position of the stop line, but also other vehicles.

#### Calculate and send the list of target waypoints

In a second process, the system is constantly identifying the position of the car along the route and publishing a new list of waypoints together with target velocity commands.

Without traffic lights or obstacles, the process will publish a list of 200 waypoints in front of the car along the marked route. When a traffic light indicates a stop command, the system will calculate a comfortable stopping velocity profile (using a sigmoid function). A similar function would be used to navigate other types of obstacles, such as other vehicles driving at a slower speed.

The sigmoid function was selected as a stopping profile in order to ease the vehicle in and out of a braking stage in an effor to minimize jerk, although a more thorough review of the function's derivatives are required to support this claim.

#### Implement throttle, break and steering control

Vehicle control is managed with a PID controller and a Yaw controller (the latter provided for the project).

The first step is to identify if it is actually required to control the car. A boolean variable is used for this purpose, which could be fed from a switch or button in a real car. For the simulator, a checkbox is used.

Once the autonomous control is engaged, the car state variables (current and target lineal speed and angular speed) are read and target values are calculated for throttle, break and steering. The vehicle's physical attributes are factored into the calculations as part of the initialization of the controllers. Such attributes include the vehicle's mass, wheel radius, wheel base, etc.

The steering is calculated first, mainly as a function of the three state variables. Because of some of the lag, the steering value is decreased as the speed of the vehicle increases. This is natural, as the steering angles should be softer for higher speeds.

The throttle is calculated with the PID controller, using the current and target velocities as process variable and setpoint, basically. It is relevant to say that the current velocity is applied a low pass filter in order to reduce variability from noise in the readings.

Finally, the break is only ever applied if the target velocity cannot be reached only by eliminating the throttle, although it has a limit set by the maximum deceleration allowed.

Several improvements are required in this section of the code to ensure the vehicle follows the trajectory better. The yaw controller is a prime candidate for review since the car doesn't seem to stop swaying. I followed one of the walkthrough's recommendations to modify the "Pure Pursuit" waypoint follower so that angular velocities are sent always instead of only when the car strays from the trajectory, but this only caused more lag instead.

#### Connect everything through ROS

Since all the different subsystems must communicate with one another, the "Robot Operating System" (ROS) represents a great match for a self-driving car. In the case of this project, several processes (ROS nodes) are initialized and communicate via publishers and subscribers. Each of the previous sections corresponds to at least one ROS node (tl_detector, waypoint_updater and dbw_node respectively).

Each node accesses configuration parameters and can subscribe and publish to any number of nodes. Since all the logic to do that is part of the platform, no communication, configuration or even logging code had to be written.

A bit of a downside is that hard realtime performance is not available for mainline ROS, although it is possible through some extensions. I would also like to dive deeper into the overhead of the base system. According to the information on the course material, the hardware used to run the system is quite powerful.

## Installation
Please use **one** of the two installation options, either native **or** docker installation.

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Port Forwarding
To set up port forwarding, please refer to the [instructions from term 2](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/16cf4a78-4fc7-49e1-8621-3450ca938b77)

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/traffic_light_bag_file.zip) that was recorded on the Udacity self-driving car.
2. Unzip the file
```bash
unzip traffic_light_bag_file.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_file/traffic_light_training.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
