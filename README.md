# English | [中文](README_cn.md)
# Tron1 SS

## 1. Deployment Environment Setup

**Install ROS Noetic:**  
We recommend building an algorithm development environment based on ROS Noetic on Ubuntu 20.04. ROS provides a suite of tools and libraries—such as core libraries, communication frameworks, and simulation tools (e.g., Gazebo)—which greatly facilitate the development, testing, and deployment of robotic algorithms. These resources offer users a rich and complete development environment.

To install ROS Noetic, please refer to the official documentation:  
👉 [ROS Noetic Installation on Ubuntu](https://wiki.ros.org/noetic/Installation/Ubuntu)  
Make sure to choose the **`ros-noetic-desktop-full`** version.

After installing ROS Noetic, run the following shell commands in a Bash terminal to install the required dependencies:

```bash
sudo apt-get update
sudo apt install ros-noetic-map-server \
                 ros-noetic-octomap-ros \
                 ros-noetic-octomap-msgs \
                 ros-noetic-octomap-server \
                 ros-noetic-dwa-local-planner \
                 ros-noetic-octomap-rviz-plugins\
                 ros-noetic-move-base \
                 ros-noetic-pointcloud-to-laserscan \
                 python3.8 \
                 libeigen3-dev \
                 cmake build-essential libpcl-dev libeigen3-dev libopencv-dev  \
                 python3-pip libboost-all-dev libtbb-dev -y
sudo apt-get install libboost-all-dev libwebsocketpp-dev nlohmann-json3-dev
sudo apt-get install ros-noetic-realsense2-camera ros-noetic-realsense2-camera-dbgsym ros-noetic-realsense2-description

cd ~/ && mkdir tools && cd tools
git clone https://github.com/Livox-SDK/Livox-SDK
cd Livox-SDK/build
cmake .. && make -j6  
sudo make install

git clone https://github.com/Livox-SDK/Livox-SDK2
cd Livox-SDK2 && mkdir build && cd build
cmake .. && make -j6
sudo make install

cd ../../ 
wget -O gtsam.zip [https://github.com/borglab/gtsam/archive/refs/tags/4.1.1.zip](https://github.com/borglab/gtsam/archive/refs/tags/4.1.1.zip)
unzip gtsam.zip
cd gtsam-4.1.1/ && mkdir build && cd build
cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON ..
sudo make install -j6

cd ../../

git clone https://github.com/MIT-SPARK/TEASER-plusplus.git
cd TEASER-plusplus && mkdir build && cd build
cmake .. -DENABLE_DIAGNOSTIC_PRINT=OFF
sudo make install -j6
sudo ldconfig


```

---

## 2. Compilation and Execution


### Step 1: Open a Bash terminal.

### Step 2: Clone the source code repository:

```bash
git clone https://github.com/limxdynamics/tron1-ss
```

### Step 3: Compile the project:

```bash
cd tron1-ss
catkin_make_isolated 
```

---


## 3. Run 

mapping
### Step 1: Mapping
```bash
roslaunch livox_ros_driver2  msg_MID360.launch
roslaunch fast_lio_sam_sc_qn run.launch lidar:=livox_mid360
```

localization and navigation
### Step 2: Localization & Navigation
```bash
roslaunch livox_ros_driver2  msg_MID360.launch
roslaunch fast_lio_localization_sc_qn run.launch lidar:=livox_mid360
```

## License

This repository's own deployment & integration code is licensed under [Apache License 2.0](LICENSE). However, the bundled localization & mapping stack (`src/fast-lio-mapping_and_localization-sc-qn/`) is from third parties and is licensed under **CC BY-NC-SA 4.0 (non-commercial)**; it also bundles GPL-2.0 (FAST_LIO) and MIT components.

> ⚠️ **Commercial use of the localization & mapping stack requires a license from the original authors, or replacing those components.** See [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md) for details.
