# 阿克曼车辆导航系统构建说明

## 系统要求

- Ubuntu 18.04 或更高版本
- ROS Noetic
- Python 3
- CMake 3.0.2 或更高版本

## 安装依赖

### 1. 安装ROS Noetic
```bash
# 添加ROS源
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# 更新包列表
sudo apt update

# 安装ROS Noetic Desktop Full
sudo apt install ros-noetic-desktop-full

# 初始化rosdep
sudo rosdep init
rosdep update

# 设置环境
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2. 安装其他依赖
```bash
# 安装必要的ROS包
sudo apt install ros-noetic-tf ros-noetic-nav-msgs ros-noetic-geometry-msgs ros-noetic-visualization-msgs

# 安装Python依赖
sudo apt install python3-numpy python3-rospy
```

## 构建项目

### 方法1: 使用构建脚本
```bash
# 进入工作空间
cd catkin_ws

# 运行构建脚本
chmod +x build.sh
./build.sh
```

### 方法2: 手动构建
```bash
# 进入工作空间
cd catkin_ws

# 清理之前的构建
rm -rf build/ devel/

# 构建项目
catkin_make

# 设置环境
source devel/setup.bash
```

## 常见编译错误及解决方案

### 错误1: 找不到头文件
```
fatal error: 'ackermann_navigation/mppi_controller.h' file not found
```
**解决方案**: 确保在正确的目录中运行构建命令，并且所有头文件都存在。

### 错误2: 链接错误
```
undefined reference to 'ros::NodeHandle::subscribe'
```
**解决方案**: 确保CMakeLists.txt中正确链接了catkin库。

### 错误3: 类型转换错误
```
error: cannot convert 'unsigned int' to 'int'
```
**解决方案**: 已在代码中添加了适当的类型转换。

### 错误4: 找不到ROS包
```
Could not find a package configuration file provided by "gazebo_ros"
```
**解决方案**: 已简化依赖项，移除了Gazebo相关依赖。

## 验证构建

构建成功后，您应该看到以下可执行文件：
```bash
ls devel/lib/ackermann_navigation/
# 应该看到:
# - mppi_controller
# - path_planner
# - costmap_manager
# - fake_odom_publisher
```

## 运行系统

### 1. 启动核心系统
```bash
# 终端1: 启动仿真
roslaunch ackermann_navigation simulation.launch
```

### 2. 运行测试
```bash
# 终端2: 运行测试脚本
python3 src/ackermann_navigation/scripts/simple_test.py
```

### 3. 查看话题
```bash
# 终端3: 查看系统话题
rostopic list
```

## 故障排除

### 如果构建仍然失败

1. **检查ROS环境**
   ```bash
   echo $ROS_DISTRO
   # 应该显示: noetic
   ```

2. **检查依赖项**
   ```bash
   rosdep check ackermann_navigation
   ```

3. **清理并重新构建**
   ```bash
   cd catkin_ws
   rm -rf build/ devel/
   catkin_make clean
   catkin_make
   ```

4. **检查编译器版本**
   ```bash
   gcc --version
   # 应该支持C++11或更高版本
   ```

### 如果运行时出错

1. **检查ROS Master**
   ```bash
   roscore
   ```

2. **检查话题连接**
   ```bash
   rostopic list
   rostopic echo /costmap
   ```

3. **检查TF变换**
   ```bash
   rosrun tf tf_echo map odom
   ```

## 系统架构

简化后的系统包含以下组件：

- **costmap_manager**: 生成和管理代价地图
- **path_planner**: 基于贝塞尔曲线的路径规划
- **mppi_controller**: MPPI控制算法实现
- **fake_odom_publisher**: 模拟里程计数据

## 下一步

构建成功后，您可以：

1. 在RViz中可视化系统
2. 添加自定义障碍物
3. 设置不同的目标点
4. 调整MPPI参数
5. 扩展系统功能

## 支持

如果遇到问题，请检查：

1. ROS版本是否正确
2. 所有依赖项是否安装
3. 构建环境是否正确
4. 代码是否有语法错误

更多信息请参考项目README.md文件。 