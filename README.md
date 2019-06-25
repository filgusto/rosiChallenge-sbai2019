# sbai2019-rosiDefy

This repository contains the ROS package and V-REP files for running the **ROSI CHALLENGE** simulation. 

The final stage of the competition will occur on the **XIV SBAI**, to be held in Ouro Preto (Brazil) in october 2019.
One may find more info about SBAI in www.sbai2019.com.br.


# Package description

This repository is a ROS package developped and tested on **Ubuntu 18.04** and **ROS Melodic**.

The folders organization is as follows:

- `cenas` - Contains simulation cenes of the challenge. You may load them inside V-REP simulator.

- `config` - Contains the **.yaml** files with simulation parameters. You may change them accordingly to your needs.

- `dev` - Useful **.sh** files for testing the simulator.

- `launch` - Contains a ROS launch file example. Notice that **rosi_joy.launch** loads the parameters file from `config` folder.

- `msg` - Message files needed to interact with the Rosi simulated model.

- `resources` - General support files.

- `script` - Example node in Python to control Rosi using a joystick. Code written with a Xbox 360 wireless joystick. 


# Simulation Parameters

The simulation may run slow due to its complexity and amount of running modules. You can tweak some parameters to better adjust the simulation to your code needs.

Inside the **rosi_defy** package, you may find the parameters in `<rosi_defy>/config/simulation_parameters.yaml`. They are all automatic loaded from the launch file in `<rosi_defy>/launch/rosi_joy.launch`. If you do not load them, all parameters will be set to **true**. Naturally, you can change parameters while running **roscore** using `rosparam set <param_name> <param_desired_value>`.

You can adjust the following flags:

- `simulation_rendering` - `Boolean` - Controls the rendering of V-REP visualization. Disable it for better performance.

- `velodyne_processing` - `Boolean` - If your code do not rely on Velodyne data, you can disable this sensor here and speed up the simulation.

- `kinect_processing` - `Boolean` - If your code do not rely on Kinect data, you can disable this sensore here and speed up the simulation.

# ROSI2ROS

This section shows how to interact with the simulated ROSI in ROS framework. It shows the published/subscribed topics, along with its message type and brief comment.
The standard is:
- `/topic_namespace/topic_name` - `<topic_message/Type>` - A brief description of its content.

As a rule of thumb, all variables are mapped in the International System of Units.

## Rosi publishes to (you receive information throught them):

- `/rosi/arms_joints_position` - `<rosi_defy/RosiMovementArray>` - Rosi tracked arms position in \[radians\].

- `/rosi/kinect_joint` - `<std_msgs/Float32>` - Kinect joint position in \[radians\].

- `/sensor/gps` - `<sensor_msgs/NavSatFix>` - Emulated GPS sensor output.

- `/sensor/imu` - `<sensor_msgs/Imu>` - Emulated IMU sensor output.

- `/sensor/kinect_depth` - `<sensor_msgs/Image>` - Emulated kinect depth image output.

- `/sensor/kinect_rgb` - `<sensor_msgs/Image>` - Emulated kinect rgb image output.

- `/sensor/kinect_info` - `<sensor_msgs/CameraInfo>` - Emulated kinect information.

- `/sensor/velodyne` - `<sensor_msgs/PointCloud>` - Emulated Velodyne output.

- `/simulation/time` - `<std_msgs/Float32>` - V-REP simulation time in \[seconds\]

- `/ur5/jointsPositionCurrentState` - `<rosi_defy/ManipulatorJoints>` - UR-5 robotic manipulator joints current position. 

## Rosi subscribes to (you send commands to the robot):

- `/rosi/command_arms_speed` - `<rosi_defy/RosiMovementArray>` - Sets the tracked arms angular velocity in \[radians/s\]. Command limits in \[-0.52, 0.52\] rad/s

- `/rosi/command_traction_speed` - `<rosi_defy/RosiMovementArray>` - Sets the traction system joint angular velocity in \[radians/s\]. The traction system drives simultaneously both the wheel and the tracks. Command limits in \[-37.76, 37.76\] rad/s.

- `/rosi/command_kinect_joint` - `<std_msgs/Float32>`- Sets the kinect joint angular position set-point. Joint limits are \[-45°,45° \]. It has a built-in PID controller with maximum joint speed of |0.35| rad/s.

- `/ur5/jointsPosTargetCommand` - `<rosi_defy/ManipulatorJoints>` - Sets the UR-5 joints desired angular position. Each joint has a built-in PID controller. One may find more UR-5 info in [here](https://www.universal-robots.com/media/50588/ur5_en.pdf).
`








