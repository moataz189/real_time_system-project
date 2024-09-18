Project Description:  Our project focuses on developing an autonomous drone system capable of flying to a designated starting point, detecting objects, and returning to its origin. The drone navigates to the specified location using predefined coordinates or real-time inputs. Once it arrives, the drone employs an object detection algorithm to identify a specific target — in our case, a chair. Upon detecting the chair, the drone hovers over the point, lands, and then initiates its return flight to the starting point. This system showcases autonomous navigation, real-time decision-making, object detection, and safe landing capabilities.
Installing the required packages for the project
#Ros2 installation

locale  # check for UTF-8

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

sudo apt upgrade

sudo apt install ros-humble-desktop-full

source /opt/ros/humble/setup.bash

echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
source ~/.bashrc

#Gazebo

sudo apt install ros-humble-gazebo-ros*
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
source ~/.bashrc


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
git clone https://github.com/WongKinYiu/yolov7.git
cd yolov7
cd ~/Iris_Files/yolov7/
pip install -r requirements.txt
wget https://github.com/WongKinYiu/yolov7/releases/download/v0.1/yolov7.pt


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
