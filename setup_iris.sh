#!/bin/bash


#iris_files
cd ~/iris_files/


#ardupilot gazebo
cd ardupilot_gazebo/
mkdir build
cd build
cmake ..
make -j4
sudo make install

echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc


#ardupilot
cd ~/iris_files/
git clone https://github.com/ArduPilot/ardupilot.git

cd ~/iris_files/ardupilot/
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y #
./waf configure --board sitl
./waf copter -v
cd ~/iris_files/ardupilot/Tools/autotest
sudo pip3 install MAVProxy

#yolov7
cd ~/iris_files/
git clone https://github.com/theos-ai/easy-yolov7.git
cd easy-yolov7
pip install -r requirements.txt
cp 

#Environment
echo 'source $HOME/iris_files/ardupilot/Tools/completion/completion.bash' >> ~/.bashrc
echo 'export PATH=$PATH:$HOME/iris_files/ardupilot/Tools/autotest' >> ~/.bashrc
echo 'export PATH=/usr/lib/ccache:$PATH' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=~/iris_files/iris/src/iris_drone/models' >> ~/.bashrc
echo 'export GAZEBO_RESOURCE_PATH=~/iris_files/iris/src/iris_drone/worlds:${GAZEBO_RESOURCE_PATH}' >> ~/.bashrc


#workspace
cd ~/iris_files/iris/
grep -rl 'jagadeesh' ./src/ | xargs sed -i -e "s/jagadeesh/$(whoami)/g"

sudo apt update
rosdep update
rosdep install -y --from-paths src --ignore-src -r
colcon build
source ~/iris_files/iris/install/setup.bash

source ~/.bashrc







