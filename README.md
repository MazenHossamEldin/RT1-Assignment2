# RT1 Assignment 2

This repository contains the implementation of the second assignment for the Real-Time Systems course. The project involves setting up a ROS environment with various nodes, services, and actions to control a robot simulation.

## Table of Contents
- [Installation](#installation)
- [Usage](#usage)
- [Configuration](#configuration)
- [Launch Instructions](#launch-instructions)
- [Message, Service, and Action Definitions](#message-service-and-action-definitions)
- [Action Client Node](#action-client-node)

## Installation

1. Clone the repository:
    ```sh
    git clone https://github.com/MazenHossamEldin/RT1-Assignment2.git
    cd RT1-Assignment2
    ```

2. Build the package:
    ```sh
    catkin_make
    ```

3. Source the setup file:
    ```sh
    source devel/setup.bash
    ```

## Configuration

### RViz Configurations

The repository includes two RViz configuration files which can be used for visualization:

1. **sim.rviz**:
    - File path: `config/sim.rviz`
    - Panels and tools are set up for viewing robot models, grids, and camera feeds.

2. **sim2.rviz**:
    - File path: `config/sim2.rviz`
    - Similar to `sim.rviz` but with additional configurations for LaserScan data.

### Launch Files

1. **assignment1.launch**:
    - Launches the main simulation environment.
    - Includes nodes for wall following, point navigation, and bug action service.
    - Starts the action client in a separate terminal.

2. **sim_w1.launch**:
    - Sets up the Gazebo simulation environment.
    - Loads the URDF model and starts RViz with the specified configuration.

## Message, Service, and Action Definitions

### Messages

- **PositionVelocity.msg**:
    - File path: `msg/PositionVelocity.msg`
    - Fields:
        ```msg
        float64 x
        float64 y
        float64 vel_x
        float64 vel_z
        ```

### Services

- **GetLastTarget.srv**:
    - File path: `srv/GetLastTarget.srv`
    - Service definition for retrieving the last target position.

### Actions

- **Planning.action**:
    - File path: `action/Planning.action`
    - Action definition for planning with the following fields:
        ```action
        geometry_msgs/PoseStamped target_pose
        ---
        ---
        geometry_msgs/Pose actual_pose
        string stat
        ```

## Action Client Node

The `scripts/action_client.py` file contains a ROS node that acts as an action client to send goals to the `/reaching_goal` action server. It also publishes the robot's position and velocity and provides a service to get the last target.

### Key Components:

- **Node Initialization**: Initializes the ROS node, action client, publisher, and service.
- **send_goal**: Sends a goal to the action server.
- **cancel_goal**: Cancels the current goal.
- **feedback_callback**: Receives feedback from the action server.
- **odom_callback**: Subscribes to odometry data and publishes position and velocity.
- **handle_service**: Handles requests for the last target.
- **run**: Provides a command-line interface for sending and canceling goals.


## Usage

### Launching the Simulation

To start the simulation, use the following launch files:

1. **Assignment Launch**:
    ```sh
    roslaunch assignment_2_2024 assignment1.launch
    ```

2. **Simulation Launch**:
    ```sh
    roslaunch assignment_2_2024 sim_w1.launch
    ```
