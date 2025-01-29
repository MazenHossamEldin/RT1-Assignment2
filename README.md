# RT1-Assignment2 - Part 2

This is the second assignment delivered in the Research Track 1 course at the University of Genoa.

## Prerequisites

To work with this part of the assignment, you will need the following:

- ROS2 installed on your system.
- The `robot_urdf` repository with the `ros2` branch.

## Setup Instructions

1. **Clone the `robot_urdf` repository with the `ros2` branch:**

    ```bash
        git clone -b ros2 https://github.com/CarmineD8/robot_urdf.git
    ```

2. **Build the `robot_urdf` package:**

    ```bash
        cd robot_urdf
        colcon build
        source install/setup.bash
    ```
3.** Make sure to install the following dependencies:**
	```bash
 	    apt-get install ros-foxy-xacro ros-foxy-joint-state-publisher ros-foxy-gazebo*
    ```	

4. **Clone this repository and switch to the `part2` branch:**

    ```bash
        git clone https://github.com/MazenHossamEldin/RT1-Assignment2.git
        cd RT1-Assignment2
        git checkout part2
    ```

5. **Install required dependencies:**

    Ensure you have all the required dependencies installed. You may use `rosdep` to install them:

    ```bash
        rosdep install --from-paths src --ignore-src -r -y
    ```

6. **Build the package:**

    ```bash
            colcon build --packages-select robot_controller
            source install/setup.bash
    ```

## Running the Simulation

After completing the setup, you can run the simulation as follows:

1. **Source the ros2 and your workspace in the .bashrc file:**


    ```bash
    	source /opt/ros/foxy/setup.bash
	    source ~/ros2_ws/install/setup.bash
    ```

2.**Launch the robot description:**

    ```bash
        ros2 launch robot_urdf gazebo.launch.py
    ```

3. **Run the assignment code:**

    In another terminal, run the necessary nodes for the assignment:

    ```bash
       ros2 run robot_controller move_robot 
    ```
