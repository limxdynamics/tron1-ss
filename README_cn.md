# 中文 | [English](README.md)
# 训练结果部署



## 1. 部署环境配置

- 安装ROS Noetic：我们推荐在Ubuntu 20.04操作系统上建立基于ROS Noetic的算法开发环境。ROS提供了一系列工具和库，如核心库、通信库和仿真工具（如Gazebo），极大地便利了机器人算法的开发、测试和部署。这些资源为用户提供了一个丰富而完整的算法开发环境。ROS Noetic 安装请参考文档：https://wiki.ros.org/noetic/Installation/Ubuntu ，选择“ros-noetic-desktop-full”进行安装。ROS Noetic 安装完成后，Bash终端输入以下Shell命令，安装开发环境所依赖的库：

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
cmake .. && make   
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

   

## 2. 编译运行

- 打开一个Bash终端。

- 下载源代码：
    ```Bash
    git clone https://github.com/limxdynamics/tron1-ss
    ```
    
- 编译工程：
    ```Bash
    cd tron1-ss
    catkin_make_isolated 
    ```
    
## 3. 运行 

建图
- 打开两个Bash终端。
    ```Bash
    roslaunch livox_ros_driver2  msg_MID360.launch
    roslaunch fast_lio_sam_sc_qn run.launch lidar:=livox_mid360
    ```


定位和导航
- 打开两个Bash终端。
    ```Bash
    roslaunch livox_ros_driver2  msg_MID360.launch
    roslaunch fast_lio_localization_sc_qn run.launch lidar:=livox_mid360
    ```

