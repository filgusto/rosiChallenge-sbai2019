# ROSI CHALLENGE - XIV SBAI

![rosi_banner](https://raw.githubusercontent.com/filRocha/rosiChallenge-sbai2019/master/resources/banner.png)

This repository contains the ROS package and V-REP files for running the **ROSI CHALLENGE** simulation. 

The final stage of the competition will occur on the **XIV SBAI**, to be held in Ouro Preto (Brazil) in october 2019.
One may find more info about SBAI in www.sbai2019.com.br.

Additionally, the content here may be beneficial for robotics classes. There is a complete mobile robot model with divers actuators and sensor fully communicating with the ROS framework and an industrial belt conveyor scenario. 
Feel free to use it for spreading the robotics knowledge in your classes! :)

We would like to thank Marc Freese and the Coppelia Robotics team for releasing the V-REP simulator for this challenge.
Moreover, these noble gentlemen greatly contributed to this simulator: Amauri Coelho Ferraz, Raphael Pereira Figueiredo da Silva, and Wagner Ferreira Andrade.

# DISCLAIMER

**This is the beta version of the simulator. 
To those interested in competing, you can download this version to see what is available and to start basic modules implementations.
However, keep up with major code's update. The competition rules (along with the inspection task description) and inscription will be available soon on the SBAI website.**

**Remember that your solution has to be implemented in the ROS (Robot Operating System) package/nodes format. The code has to be well structured, readable and cataloged as it will be open-source available. 
On the competition stage, the simulation will run on a server computer, and each teams' package will run on another computer. Be careful not to change any simulation parameters as the official one is the only one considered for the competition.**

# Package description

This repository is structured as a ROS package. The folders organization is as follows:

- `config` - Contains the **.yaml** files with simulation parameters. You may change them accordingly to your needs.

- `dev` - Useful **.sh** files for testing the simulator.

- `launch` - Contains a ROS launch file example. Notice that **rosi_joy.launch** loads the parameters file from `config` folder.

- `msg` - Message files needed to interact with the Rosi simulated model.

- `resources` - General support files.

- `script` - Example node in Python to control Rosi using a joystick. Code written with a Xbox 360 wireless joystick. 

- `vrep_content` - Contains simulation cenes of the challenge. You may load them inside V-REP simulator.

# Installation

The simulator was conceived using **Ubuntu 18.04**, **ROS Melodic**, and **V-REP 3.6.2 (rev.0)**. Another software versions might work, but they are not recommended nor officially supported for the competition. 

This installation instructions consider that you are already running **Ubuntu Version > 16.04** and **ROS Version > Kinect**. We will download V-REP in the following steps:

## Automatic installation

Follow the steps below to configure your V-REP along with ROS:


**1.** Clone and download this repository package to your `catkin_ws/src` folder with the name `rosi_defy`:
```
$ git clone https://github.com/filRocha/sbai2019-rosiDefy rosi_defy
``` 

**2.** Navigate to `<rosi_defy>/resources` folder and run the following:
```
$ cd <catkin_ws>/src/rosi_defy/resources/
$ chmod u+x ./rosi_installer.sh 
$ ./rosi_installer.sh
```
This script **should do all needed configurations** to run V-REP alongside ROS. Pay attention to its blue messages as they may contain important information. 

If everything went fine, you are good to go!
If you have any trouble running the ROSI simulator after running this script, you can re-do the below steps manually.


## Manual Instalation
Steps for manual installation:
*This consider you have already done step 1 and (at least tried) step 2.

**3.** Download **V-REP PRO EDU V3.6.1 rev4** from the Coppelia Robotics website: http://www.coppeliarobotics.com/downloads.html


**4.** Unzip it (preferentially) to your **home** folder and rename the folder as `vrep`.


**5.** Add both the CATKIN_WS and V-REP folder location to your `.bashrc`, an alias to run it, and source the `.bashrc` again: 
```
$ echo "export ROS_CATKIN_WS='<path_to_your_catkin_ws_folder>'" >> $HOME/.bashrc
$ echo "export VREP_ROOT='<path_to_your_vrep_folder>'" >> $HOME/.bashrc
$ echo "alias vrep=$VREP_ROOT/vrep.sh" >> ~/.bashrc
$ source $HOME/.bashrc
```
Remember to insert the path to your CATKIN_WS and V-REP folder in this command.

(All instructions consider that you use `bash`. If you use `.zsh`, you know what to do ;)


**6.** Test the V-REP functionality by running:
```
$ vrep
```
(Notice that you have created this command on the last step)


**7.** Clone recursively the V-REP/ROS interface to your `catkin_ws/src`:
```
$ cd $ROS_CATKIN_WS/src/
$ git clone --recursive https://github.com/CoppeliaRobotics/v_repExtRosInterface.git vrep_ros_interface
```
(More information and credits about this interface can be found on its [Github repository](https://github.com/CoppeliaRobotics/v_repExtRosInterface.git)


**8.** Install some support packages:
```
$ sudo apt install python-catkin-tools xsltproc ros-$ROS_DISTRO-brics-actuator ros-$ROS_DISTRO-tf2-sensor-msgs ros-$ROS_DISTRO-joy
```


**9.** We have just installed a new `catkin build` tool. If you use `catkin_make`, you have to clean your workspace and perform a fresh new compilation:
```
$ cd $ROS_CATKIN_WS
$ catkin clean
$ catkin build
$ source $HOME/.bashrc
```


**10.** Some messages from our `rosi_defy` package should be referenced in the `vrep_ros_interface` package. To do that:

10.1 Insert their namespace and names in `<vrep_ros_interface>/meta/messages.txt` file:
```
$ echo -e "rosi_defy/ManipulatorJoints\nrosi_defy/RosiMovement\nrosi_defy/RosiMovementArray" >> $ROS_CATKIN_WS/src/vrep_ros_interface/meta/messages.txt
```

10.2 Tell `vrep_ros_interface`that it depends on the `rosi_defy` package by adding 
```
<depend>rosi_defy</depend>
```
to `vrep_ros_interface/package.xml`.

10.3 Add `rosi_defy` package dependence on the `vrep_ros_interface/CMakeLists.txt`:
```
set(PKG_DEPS
  ... (many many other packages)
  rosi_defy
)
```


**11.** If your compilation runs well, there is now a ros interface library to copy from your `catkin_ws` to the V-REP folder:
```
$ cp $ROS_CATKIN_WS/devel/lib/libv_repExtRosInterface.so $VREP_ROOT
```
(Notice that, for further events, every time you add new custom ROS messages to the interface, you have to re-compile this library and re-copy it to `$VREP_ROOT`.


**12.** Everything should be set up for now. To run the simulation, you should first (always!) run ROS:
```
$ roscore
$ vrep
```
Open the scene in `<rosi_defy>/cenas/` and play it. You should be able to see the simulator topics being published with `rostopic list`. Additionally, if you have a joystick, you can run the `rosi_joy.py` example node to see how the communication with the robot works.

# Hello World!

To first run your simulator, do the following:

**1.** Open a new terminal and run `$ roscore` to enable ROSMASTER. You should **always** run the ROSMASTER before V-REP.

**2.** In another terminal window, run `$ vrep`. This should work if you have created successfully V-REP alias on `.bashrc`. If the negative case, run it directly using `$ <vrep_folder>/vrep.sh`.

**3.** Go to `File > Open Scene...` and locate the V-REP scenario in `<rosi_defy>/vrep_content/challenge_scenario.ttt`. 
You have also available the ROSI model that can be directly loaded in another scenario. It can be also found in the V-REP model browser (simulator's left side column).

**4.** Run the simulator by goint to `Simulation > Start simulation`. At this point, you should be able to see all ROSI topics running in ROS framework.

**5.** You can find a ROS script example code in `<rosi_defy>/script/rosi_joy.py`. This node lets you control ROSI with a joystick (made with Xbox 360 controller). Besides, it is a good way to see the basics of the ROS<->ROSI interaction.


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

## ROSI publishes to (you receive information from the robot):

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

- `/ur5/forceTorqueSensorOutput` - `<geometry_msgs/TwistStamped>` - UR-5 Force/Torque sensor output. It gives two vector of linear and angular forces and torques, respectively. Axis order is **x**, **y**, **z**.

## ROSI subscribes to (you send commands to the robot):

- `/rosi/command_arms_speed` - `<rosi_defy/RosiMovementArray>` - Sets the tracked arms angular velocity in \[radians/s\]. Command limits in \[-0.52, 0.52\] rad/s

- `/rosi/command_traction_speed` - `<rosi_defy/RosiMovementArray>` - Sets the traction system joint angular velocity in \[radians/s\]. The traction system drives simultaneously both the wheel and the tracks. Command limits in \[-37.76, 37.76\] rad/s.

- `/rosi/command_kinect_joint` - `<std_msgs/Float32>`- Sets the kinect joint angular position set-point. Joint limits are \[-45°,45° \]. It has a built-in PID controller with maximum joint speed of |0.35| rad/s.

- `/ur5/jointsPosTargetCommand` - `<rosi_defy/ManipulatorJoints>` - Sets the UR-5 joints desired angular position. Each joint has a built-in PID controller. One may find more UR-5 info in [here](https://www.universal-robots.com/media/50588/ur5_en.pdf).


# I have found something wrong!

Great! Please, open a new **issue** directly on this repository, or send your **pull request**.

If you have any doubts, suggestions, comments or compliments, you can reach:

**Filipe Rocha**

`filipe.rocha@itv.org` or `f.rocha41@gmail.com`.



## Have FUN!

