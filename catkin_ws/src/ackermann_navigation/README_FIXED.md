# 阿克曼车辆导航系统 - 修复版本

## 概述

这是一个基于ROS的阿克曼车辆导航控制系统，使用MPPI（Model Predictive Path Integral）控制算法进行路径跟踪和障碍物避让。系统已修复编译错误，简化了依赖项，提高了兼容性。

## 主要修复

### 1. 编译错误修复
- 添加了缺失的头文件 `<limits>` 和 `<cmath>`
- 修复了类型转换问题（unsigned int 到 int）
- 添加了除零保护
- 修复了数组边界检查

### 2. 依赖项简化
- 移除了Gazebo相关依赖
- 移除了Eigen3依赖
- 移除了sensor_msgs依赖
- 保留了核心ROS依赖项

### 3. 系统组件

#### 核心组件
- **MPPI控制器** (`mppi_controller`): 实现MPPI控制算法
- **路径规划器** (`path_planner`): 基于贝塞尔曲线的路径规划
- **代价地图管理器** (`costmap_manager`): 生成和管理代价地图
- **模拟里程计发布器** (`fake_odom_publisher`): 提供模拟的里程计数据

#### 新增功能
- 简化的启动文件，不依赖Gazebo
- 模拟里程计发布器，用于测试
- 简化的测试脚本
- 改进的构建脚本

## 系统架构

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Costmap       │    │   Path          │    │   MPPI          │
│   Manager       │    │   Planner       │    │   Controller    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
   /costmap              /reference_path         /predicted_trajectory
         │                       │                       │
         └───────────────────────┼───────────────────────┘
                                 │
                                 ▼
                        /ackermann/cmd_vel
                                 │
                                 ▼
                    Fake Odom Publisher
                                 │
                                 ▼
                              /odom
```

## 快速开始

### 1. 构建系统
```bash
cd catkin_ws
chmod +x build.sh
./build.sh
```

### 2. 启动系统
```bash
# 终端1: 启动ROS Master
roscore

# 终端2: 启动导航系统
roslaunch ackermann_navigation simulation.launch
```

### 3. 运行测试
```bash
# 终端3: 运行测试脚本
python3 src/ackermann_navigation/scripts/simple_test.py
```

### 4. 可视化
```bash
# 终端4: 启动RViz（如果未自动启动）
rviz -d src/ackermann_navigation/config/navigation.rviz
```

## 话题说明

### 输入话题
- `/move_base_simple/goal` (geometry_msgs/PoseStamped): 目标点设置
- `/add_obstacle` (geometry_msgs/PoseStamped): 添加障碍物
- `/ackermann/cmd_vel` (geometry_msgs/Twist): 控制指令

### 输出话题
- `/costmap` (nav_msgs/OccupancyGrid): 代价地图
- `/reference_path` (nav_msgs/Path): 参考路径
- `/predicted_trajectory` (nav_msgs/Path): 预测轨迹
- `/odom` (nav_msgs/Odometry): 里程计数据
- `/path_markers` (visualization_msgs/Marker): 路径可视化
- `/obstacle_markers` (visualization_msgs/Marker): 障碍物可视化

## 参数配置

### MPPI控制器参数
```yaml
# 控制参数
horizon: 20              # 预测时域
sample_count: 1000       # 采样数量
dt: 0.1                  # 时间步长
lambda: 0.1              # 温度参数

# 车辆参数
wheelbase: 1.6           # 轴距
max_velocity: 5.0        # 最大速度
min_velocity: 0.5        # 最小速度
max_steering: 0.5        # 最大转向角

# 代价函数权重
w_path: 1.0              # 路径跟踪权重
w_obstacle: 10.0         # 障碍物避让权重
w_control: 0.1           # 控制平滑权重
w_velocity: 0.5          # 速度跟踪权重
```

### 路径规划器参数
```yaml
path_resolution: 0.1     # 路径分辨率
max_curvature: 0.5       # 最大曲率
vehicle_width: 1.0       # 车辆宽度
vehicle_length: 2.0      # 车辆长度
safety_margin: 0.5       # 安全边距
```

### 代价地图参数
```yaml
map_width: 50.0          # 地图宽度
map_height: 50.0         # 地图高度
map_resolution: 0.1      # 地图分辨率
map_origin_x: -25.0      # 地图原点X
map_origin_y: -25.0      # 地图原点Y
```

## 算法原理

### MPPI控制算法
MPPI（Model Predictive Path Integral）是一种基于采样的最优控制算法：

1. **轨迹采样**: 在当前控制序列基础上添加噪声生成多个轨迹
2. **代价评估**: 计算每个轨迹的总代价
3. **权重计算**: 基于代价计算每个轨迹的权重
4. **控制更新**: 使用加权平均更新控制序列

### 贝塞尔路径规划
使用三次贝塞尔曲线生成平滑路径：

1. **控制点计算**: 根据路径点计算贝塞尔曲线控制点
2. **曲线生成**: 使用贝塞尔公式生成平滑路径
3. **碰撞检测**: 检查路径是否与障碍物碰撞
4. **路径优化**: 必要时调整控制点优化路径

## 性能优化

### 编译优化
- 使用 `-O2` 优化级别
- 启用内联函数
- 使用静态链接

### 运行时优化
- 限制采样数量
- 使用简化的碰撞检测
- 优化路径分辨率

## 故障排除

### 常见问题

1. **编译错误**
   - 确保ROS环境正确设置
   - 检查所有依赖项是否安装
   - 清理并重新构建

2. **运行时错误**
   - 检查ROS Master是否运行
   - 验证话题连接
   - 检查TF变换

3. **性能问题**
   - 调整MPPI参数
   - 减少采样数量
   - 优化路径分辨率

### 调试技巧

1. **查看话题数据**
   ```bash
   rostopic echo /costmap
   rostopic echo /reference_path
   rostopic echo /predicted_trajectory
   ```

2. **检查TF变换**
   ```bash
   rosrun tf tf_echo map odom
   ```

3. **可视化调试**
   ```bash
   rviz -d config/navigation.rviz
   ```

## 扩展功能

### 可能的扩展
1. 添加SLAM功能
2. 集成真实传感器数据
3. 添加多车辆支持
4. 实现动态障碍物避让
5. 添加路径重规划功能

### 自定义开发
1. 修改代价函数
2. 调整车辆参数
3. 添加新的控制算法
4. 扩展可视化功能

## 许可证

本项目采用Apache 2.0许可证。

## 贡献

欢迎提交问题报告和功能请求。请确保在提交代码前通过所有测试。

## 联系方式

如有问题，请通过以下方式联系：
- 提交GitHub Issue
- 发送邮件至维护者

---

**注意**: 这是修复版本，移除了Gazebo依赖，专注于核心导航功能。如需完整的仿真功能，请参考原始版本。 