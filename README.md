# Real-Time Autonomous Drone System

## 🧠 Project Description
This project focuses on developing an autonomous drone system capable of flying to a designated starting point, detecting specific objects, and returning to its origin. The drone navigates using either predefined GPS coordinates or real-time input. Upon reaching the target area, it uses an object detection model (YOLOv7) to search for a chair. Once detected, the drone hovers, lands near the target, and initiates a return flight.

This system demonstrates:
- Autonomous navigation using ROS 2
- Real-time object detection with YOLOv7
- Safe landing and return mechanisms
- Mission-based and real-time waypoint handling

Developed by: **Moataz Odeh & Adan Sulaimani**

---

## 📦 Installation Guide

### 🔧 ROS 2 (Humble) Installation
```bash
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
locale

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
```

### 🧱 Gazebo Installation
```bash
sudo apt install ros-humble-gazebo-ros*
echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc
source ~/.bashrc
```

### 🛫 Iris Drone Files Setup
```bash
cd ~/iris_files/

# ArduPilot Gazebo
cd ardupilot_gazebo/
mkdir build
cd build
cmake ..
make -j4
sudo make install

# ArduPilot
cd ~/iris_files/
git clone https://github.com/ArduPilot/ardupilot.git
cd ardupilot/
git submodule update --init --recursive
Tools/environment_install/install-prereqs-ubuntu.sh -y
./waf configure --board sitl
./waf copter -v
cd Tools/autotest
sudo pip3 install MAVProxy
```

### 🔍 YOLOv7 Installation
```bash
git clone https://github.com/WongKinYiu/yolov7.git
cd yolov7
pip install -r requirements.txt
wget https://github.com/WongKinYiu/yolov7/releases/download/v0.1/yolov7.pt
```

### 🌍 Environment Setup
```bash
echo 'source $HOME/iris_files/ardupilot/Tools/completion/completion.bash' >> ~/.bashrc
echo 'export PATH=$PATH:$HOME/iris_files/ardupilot/Tools/autotest' >> ~/.bashrc
echo 'export PATH=/usr/lib/ccache:$PATH' >> ~/.bashrc
echo 'export GAZEBO_MODEL_PATH=~/iris_files/iris/src/iris_drone/models' >> ~/.bashrc
echo 'export GAZEBO_RESOURCE_PATH=~/iris_files/iris/src/iris_drone/worlds:${GAZEBO_RESOURCE_PATH}' >> ~/.bashrc
source ~/.bashrc
```

### 🧪 Workspace Setup
```bash
cd ~/iris_files/iris/
grep -rl 'jagadeesh' ./src/ | xargs sed -i -e "s/jagadeesh/$(whoami)/g"
sudo apt update
rosdep update
rosdep install -y --from-paths src --ignore-src -r
colcon build
source install/setup.bash
```

**Copy the `ros2_yolo.py` file to the `yolov7` directory.**

### 🌐 Download Extra Models
Download the `MODELS` folder from this repo:
https://github.com/leonhartyao/gazebo_models_worlds_collection/tree/master

Copy it to:
```bash
iris_files/iris/src/iris_drone/worlds
```

---

## ▶️ How to Run

### Terminal 1:
```bash
cd ~/iris_files/iris/
colcon build
source install/setup.bash
ros2 launch iris_drone iris.launch.py
```

### Terminal 2:
```bash
cd ~/iris_files/yolov7/
python3 ros2_yolo.py       # or: python3 ros2_yolo.py chair
```

### Terminal 3:
```bash
cd ~/iris_files/iris/
colcon build
source install/setup.bash
```

#### Run predefined mission:
```bash
ros2 run iris_drone mission
```

#### Run real-time point input mode:
```bash
ros2 run iris_drone real_time_node
```

---

## 📍 Adding Search Points
In the `mission` code:
```python
self.GotoLocation(-35.3630969, 149.1651725, alt)
```

In real-time, publish coordinates like this:
```bash
ros2 topic pub /destination_coordinates geometry_msgs/Point "{x: -35.363244, y: 149.1652153, z: 5.0}"
```

---

## 🔗 References
- https://github.com/ArduPilot/ardupilot_gazebo/blob/main/models/iris_with_ardupilot/model.sdf  
- https://github.com/leonhartyao/gazebo_models_worlds_collection/tree/master/worlds  
- https://github.com/PX4/PX4-Autopilot  
- https://github.com/WongKinYiu/yolov7

