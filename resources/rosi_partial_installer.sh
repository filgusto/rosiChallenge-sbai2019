#!/bin/bash

echo "=================================="
echo -e "ROSI CHALLENGE partial installer*\n\n*You still need to do some manual operations after.\n\n"

echo -e "First, tell me where is located your catkin_ws folder (example: /home/john/vrep):"
read var_catkin_ws_folder

echo -e "\nDo you use bash or zsh? (answer: bash, zsh)"
read var_shell

# treats the rc file
if [[ $var_shell == "zsh" ]]; then
	editorc=".zshrc"
elif [[ $var_shell == "bash" ]]; then
	editorc=".bashrc"
fi

echo $HOME/$editorc

# downloads V-REP
# wget http://coppeliarobotics.com/files/V-REP_PLAYER_V3_6_1_Ubuntu18_04.tar.xz

# insert some stuff into bashrc or zshrc
echo "export ROS_CATKIN_WS='$var_catkin_ws_folder'" >> $HOME/editorc


