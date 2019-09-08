# ROSI CHALLENGE CHANGELOG

This file list all official major changes on rosi challenge codes or regulation.

=================================================================================
## 2019-09-07 - INDEPENDENCE DAY's 1st major bug correction

- created 'launcher/load_parameters.launch'.

- in simulator
	-- kinect depth enconding transformed to 16UC1. 
	-- added a vision sensor to the UR5 tool.
	-- fixed kinect depth image step to width x 2.
	-- changed kinect frame id to camera_rgb_optical_frame (rgb) and camera_depth_optical_frame (depth).
	-- rosi publishes its first static transform to ROS tf server.
	-- velodyne treatment now is on an external plugin. See README.md for more information.
	-- velodyne now publishes in PointCloud2 format.
	-- fixed dynamical arms bounding box.
	-- fixed kinetic static object mounted over dynamic.
	-- removed dynamic arms local and global response.

- added flag to deactivate fire rendering in ./config/parameters.yaml


---------------------------------------------------------------------------------
## 2019-08-27 - URDF ADDITION and  

- Created Rosi URDF model and `urdf` folder

- in './vrep_content/rosi_model.ttm': 
	- changed robot default coordinates to [0,0, 2.41e-1]m.
	- changed 'arm_dynX' -> 'Scene Object Properties -> Select base of model instead' box trully checked.
	- model rotated in 3.14 rad around its z axis.

- in './README.md'
	- added `ros-$ROS_DISTRO-joint-state-publisher` package as common install ROS package.
	- removed inscription link.
	- added `urdf` folder description.

- in './resources/regulamento_rosiChallenge':
	- Regulation (V1.2): changed font colors.


---------------------------------------------------------------------------------
## 2019-08-19 - REGULATION AND REGISTRATION DATES CHANGES

- The inscription date was extended until 2019-august-26.

Regulation changes (V1.1):
-- Explicitly indicated that the teams solutions have to be uploaded to a clean github repository.
-- The teams can now indicate a roslaunch file and/or a .sh file to run its solution.
-- Added Hector Azpúrua as organizer.


---------------------------------------------------------------------------------
## 2019-06-29 - REGULATION AND REGISTRATION SYSTEM

- The regulation 'regulamento_rosiChallenge.pdf' V1.0 and changelog 'changelog.txt' documents are available on the official repository.


---------------------------------------------------------------------------------
## 2019-06-25 - HOKUYO ADDITION

- We have added a planar laser Hokuyo on UR-5 tool. It is already publishing in ROS topics.


---------------------------------------------------------------------------------


