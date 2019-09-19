# ROSI CHALLENGE CHANGELOG

This file list all official major changes on ROSI Challenge codes or regulation.

===============================================================================
## 2019-09-19 - Regulation changes

- in './resources/regulamento_rosiChallenge' - Regulation (V1.4):	
	- changed prize format.

---------------------------------------------------------------------------------
## 2019-09-16 - Minor bug 

- Fixed kinect stamp (thanks to Danilo Schneider).
- Corrected kinect's 'is_bigendian' bit.

---------------------------------------------------------------------------------
## 2019-09-10 - Correcting Velodyne bug


- Velodyne was not correctly publishing its local transform. This lead to an error while plotting its data. This is now corrected.


---------------------------------------------------------------------------------
## 2019-09-07 - INDEPENDENCE DAY's 1st major bug correction

- Created 'launcher/load_parameters.launch'.

- in simulator:
	- Kinect depth enconding transformed to 16UC1.
	- Added a vision sensor to the UR5 tool.
	- Fixed kinect depth image step to width x 2.
	- Changed kinect frame id to camera_rgb_optical_frame (rgb) and camera_depth_optical_frame (depth).
	- Rosi publishes its first static transform to ROS tf server.
	- Velodyne treatment now is on an external plugin. See README.md for more information.
	- Velodyne now publishes in PointCloud2 format.
	- Fixed dynamical arms bounding box.
	- Fixed kinetic static object mounted over dynamic.
	- Removed dynamic arms local and global response.
	
- Added flag to deactivate fire rendering in './config/parameters.yaml'.

- in './resources/regulamento_rosiChallenge' - Regulation (V1.3):
	- Added the description for the vision sensor from UR5 tool.
	- Added Wenderson G. Serrantola as support commissioner.


---------------------------------------------------------------------------------
## 2019-08-27 - URDF ADDITION and  

- Created Rosi URDF model and `urdf` folder

- in './vrep_content/rosi_model.ttm': 
	- Changed robot default coordinates to [0,0, 2.41e-1]m.
	- Changed 'arm_dynX' -> 'Scene Object Properties -> Select base of model instead' box trully checked.
	- Model rotated in 3.14 rad around its z axis.

- in './README.md'
	- Added `ros-$ROS_DISTRO-joint-state-publisher` package as common install ROS package.
	- Removed inscription link.
	- Added `urdf` folder description.

- in './resources/regulamento_rosiChallenge'- Regulation (V1.2):
	- Changed font colors.


---------------------------------------------------------------------------------
## 2019-08-19 - REGULATION AND REGISTRATION DATES CHANGES

- The inscription date was extended until 2019-august-26.

- in './resources/regulamento_rosiChallenge' - Regulation (V1.1):
	- Explicitly indicated that the teams solutions have to be uploaded to a clean github repository.
	- The teams can now indicate a roslaunch file and/or a .sh file to run its solution.
	- Added Hector Azpúrua as organizer.


---------------------------------------------------------------------------------
## 2019-06-29 - REGULATION AND REGISTRATION SYSTEM

- The regulation 'regulamento_rosiChallenge.pdf' V1.0 and changelog 'changelog.txt' documents are available on the official repository.


---------------------------------------------------------------------------------
## 2019-06-25 - HOKUYO ADDITION

- We have added a planar laser Hokuyo on UR-5 tool. It is already publishing in ROS topics.


---------------------------------------------------------------------------------
