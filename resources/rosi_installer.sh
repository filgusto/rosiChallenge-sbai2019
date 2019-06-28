#!/bin/bash

# support variables
vrep_folder=$HOME'/.vrep' #vrep folder location

#colors for echo
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
ORANGE='\033[0;33m'
NC='\033[0m' # No Color

# messages to the user
echo -e "${GREEN}=========================================${NC}"
echo -e "${GREEN}=== ROSI CHALLENGE SIMULATOR INSTALLER ==${NC}"
echo -e "${GREEN}=========================================${NC}\n"

# reads and treats the caktin_ws location
echo -e "${BLUE}First, tell me where is located your catkin_ws folder (example: /home/john/catkin_ws):${NC}"
read var_catkin_ws_folder

# checks if this directory exists
if [ ! -d $var_catkin_ws_folder ]; then
	echo -e "${RED}It appears that this path does not exists. Stopping now this script.${NC}"
	exit 1
fi

# searches for the .zsh file
if [ -f "$HOME/.zshrc" ]; then
	echo -e "${ORANGE}$HOME/.zshrc file found!${NC}"
	var_shell="y"
else
	var_shell="n"
fi

# install some support packages
echo -e "${BLUE}\nInstalling some support packages\n${ORANGE}(We might need your sudo password here)${NC}"
sudo apt update #UNCOMMENT
sudo apt install -f python-catkin-tools xsltproc ros-$ROS_DISTRO-brics-actuator ros-$ROS_DISTRO-tf2-sensor-msgs ros-$ROS_DISTRO-joy #UNCOMMENT

# downloads V-REP
echo -e "\n${BLUE}\nDownloading V-REP from Coppelia Robotics server.${NC}"
wget http://coppeliarobotics.com/files/V-REP_PRO_EDU_V3_6_2_Ubuntu18_04.tar.xz #UNCOMMENT

# extract, move and delete V-REP folder
echo -e "\n${BLUE}Extracting V-REP to $vrep_folder.${NC}"
tar -xf ./V-REP_PRO_EDU_V3_6_2_Ubuntu18_04.tar.xz 
mv ./V-REP_PRO_EDU_V3_6_2_Ubuntu18_04 $vrep_folder
rm -rf ./V-REP_PRO_EDU_V3_6_2_Ubuntu18_04.tar.xz

# insert some stuff into bashrc
echo -e "\n${BLUE}Adding some lines to $HOME/.bashrc${NC}"
echo -e "\n\n\n#ROSI SIMULATION additions " >> $HOME/.bashrc
echo "export ROS_CATKIN_WS='$var_catkin_ws_folder'" >> $HOME/.bashrc
echo "export VREP_ROOT='$vrep_folder'" >> $HOME/.bashrc
echo "alias vrep='$vrep_folder/vrep.sh'" >> $HOME/.bashrc

# adds also to zshrc, if the user have it
if [[ $var_shell == "y" ]]; then
	echo -e "\n${BLUE}Adding some lines to $HOME/.zshrc ${NC}"
	echo -e "\n\n\n#ROSI SIMULATION additions " >> $HOME/.zshrc
	echo "export ROS_CATKIN_WS='$var_catkin_ws_folder'" >> $HOME/.zshrc
	echo "export VREP_ROOT='$vrep_folder'" >> $HOME/.zshrc
	echo "alias vrep='$vrep_folder/vrep.sh'" >> $HOME/.zshrc
fi

#sourcing again bashrc
source $HOME/.bashrc

#compiles rosi_defy package into catkin_ws
echo -e "\n${BLUE}Compiling rosi_defy package to generate its messages.\nWe have to first clean your catkin_ws environment\n${ORANGE}Select 'YES' if asked to remove catkin_ws directories.${NC}"
cd $ROS_CATKIN_WS
catkin clean 
catkin build
source $HOME/.bashrc

#clones the vrep_ros_interface
echo -e "\n${BLUE}Cloning the V-REP ROS Interface into $ROS_CATKIN_WS/src/vrep_ros_interface.${NC}"
git clone --recursive https://github.com/CoppeliaRobotics/v_repExtRosInterface.git $ROS_CATKIN_WS/src/vrep_ros_interface

#adding ROSI messages to the vrep_ros_interface
echo -e "\n${BLUE}Adding custom ROSI simulator messages to vrep_ros_interface files.${NC}"
echo -e "rosi_defy/ManipulatorJoints\nrosi_defy/RosiMovement\nrosi_defy/RosiMovementArray" >> $ROS_CATKIN_WS/src/vrep_ros_interface/meta/messages.txt
sed -i '33i\  <depend>rosi_defy</depend>' $ROS_CATKIN_WS/src/vrep_ros_interface/package.xml
sed -i '27i\    rosi_defy' $ROS_CATKIN_WS/src/vrep_ros_interface/CMakeLists.txt

# cleaning and re-compiling the workspace
echo -e "\n${BLUE}Re-compiling the ROS workspace.${NC}"
cd $ROS_CATKIN_WS
catkin build
source $HOME/.bashrc

# copying the vrep library to vrep_root
echo -e "\n${BLUE}Copying vrep_ros_interface library to VREP_ROOT.${NC}"
cp $ROS_CATKIN_WS/devel/lib/libv_repExtRosInterface.so $vrep_folder

# Copy ROSI model to vrep models folder
echo -e "\n${BLUE}Copying ROSI model to V-REP robot models"
cp $ROS_CATKIN_WS/src/rosi_defy/vrep_content/rosi_model.ttm $vrep_folder/models/robots/mobile/ROSI.ttm

#sourcing again bashrc
source $HOME/.bashrc

# final messages 
echo -e "${GREEN}\n\nFinished installing and configuring V-REP and ROS interface!${NC}"
echo -e "${GREEN}\nIf you are using zsh, please source it again.${NC}"
echo -e "${GREEN}Now you can run V-REP in terminal using the command ${ORANGE}vrep${BLUE}.${NC}"
echo -e "${GREEN}Check if everything is OK, since this code does not check its actions.${NC}"
echo -e "\n${BLUE}HAVE FUN! :).${NC}"
echo -e "${GREEN}=========================================${NC}"