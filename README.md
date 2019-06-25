# sbai2019-rosiDefy


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

- `/rosi/command_arms_speed` - `<rosi_defy/RosiMovementArray>` - Sets the tracked arms angular velocity in \[radians/s\].

- `/rosi/command_traction_speed` - `<rosi_defy/RosiMovementArray>` - Sets the traction system joint angular velocity in \[radians/s\]. The traction system drives simultaneously both the wheel and the tracks.

- `/rosi/command_kinect_joint` - `<std_msgs/Float32>`- Sets the kinect joint angular position set-point. Joint limits are \[-45°,45° \]. It has a built-in PID controller with maximum joint speed of 20 °/s.

- `/ur5/jointsPosTargetCommand` - `<rosi_defy/ManipulatorJoints>` - Sets the UR-5 joints desired angular position. Each joint has a built-in PID controller. One may find more UR-5 info in [here](https://www.universal-robots.com/media/50588/ur5_en.pdf).
`








