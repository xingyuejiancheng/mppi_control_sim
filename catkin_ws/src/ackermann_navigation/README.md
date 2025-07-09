# 阿克曼车辆导航系统

这是一个基于ROS1的阿克曼转向车辆导航仿真系统，使用MPPI（Model Predictive Path Integral）控制算法实现路径跟踪和障碍物避免。

## 系统特性

- **阿克曼转向模型**: 完整的车辆运动学模型，支持前轮转向
- **MPPI控制器**: 基于采样的模型预测控制算法
- **贝塞尔曲线路径规划**: 平滑的路径生成
- **动态障碍物避免**: 实时代价地图和碰撞检测
- **Gazebo仿真**: 3D物理仿真环境
- **RViz可视化**: 实时状态和路径可视化

## 系统架构

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Costmap       │    │   Path          │    │   MPPI          │
│   Manager       │───▶│   Planner       │───▶│   Controller    │
│                 │    │                 │    │                 │
│ - 障碍物管理     │    │ - 贝塞尔曲线     │    │ - 轨迹预测       │
│ - 代价地图生成   │    │ - 路径优化       │    │ - 控制优化       │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       │
         ▼                       ▼                       ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Gazebo        │    │   RViz          │    │   Ackermann     │
│   Simulation    │    │   Visualization │    │   Vehicle       │
│                 │    │                 │    │                 │
│ - 物理仿真       │    │ - 路径显示       │    │ - 车辆模型       │
│ - 传感器数据     │    │ - 状态监控       │    │ - 运动控制       │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

## 安装依赖

### 系统依赖
```bash
sudo apt-get update
sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
sudo apt-get install ros-noetic-robot-state-publisher ros-noetic-joint-state-publisher
sudo apt-get install ros-noetic-teleop-twist-keyboard
sudo apt-get install libeigen3-dev
```

### 编译项目
```bash
cd catkin_ws
catkin_make
source devel/setup.bash
```

## 使用方法

### 1. 启动完整仿真系统
```bash
roslaunch ackermann_navigation simulation.launch
```

### 2. 启动部分组件（可选）
```bash
# 仅启动Gazebo仿真
roslaunch ackermann_navigation simulation.launch use_rviz:=false

# 仅启动RViz可视化（无Gazebo）
roslaunch ackermann_navigation simulation.launch use_gazebo:=false

# 自定义MPPI参数
roslaunch ackermann_navigation simulation.launch horizon:=30 sample_count:=2000
```

### 3. 交互式控制

#### 设置目标点
在RViz中使用"2D Nav Goal"工具点击地图设置目标点。

#### 手动控制
使用键盘控制车辆：
```bash
# 在另一个终端中运行
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

控制键：
- `i`: 前进
- `,`: 后退
- `j`: 左转
- `l`: 右转
- `k`: 停止

#### 添加障碍物
```bash
# 发布障碍物位置
rostopic pub /add_obstacle geometry_msgs/PoseStamped "header:
  frame_id: 'map'
pose:
  position:
    x: 5.0
    y: 5.0
    z: 0.0
  orientation:
    w: 1.0"
```

## 参数配置

### MPPI控制器参数
- `horizon`: 预测时域长度 (默认: 20)
- `sample_count`: 采样数量 (默认: 1000)
- `dt`: 时间步长 (默认: 0.1s)
- `noise_std_v`: 速度噪声标准差 (默认: 0.5)
- `noise_std_delta`: 转向角噪声标准差 (默认: 0.1)
- `temperature`: 温度参数 (默认: 1.0)

### 车辆参数
- `wheelbase`: 轴距 (默认: 1.6m)
- `max_velocity`: 最大速度 (默认: 5.0m/s)
- `min_velocity`: 最小速度 (默认: 0.5m/s)
- `max_steering`: 最大转向角 (默认: 0.5rad)

### 代价函数权重
- `w_path`: 路径跟踪权重 (默认: 1.0)
- `w_obstacle`: 障碍物避免权重 (默认: 10.0)
- `w_control`: 控制平滑权重 (默认: 0.1)
- `w_velocity`: 速度跟踪权重 (默认: 0.5)

## 话题说明

### 输入话题
- `/ackermann/odom`: 车辆里程计信息
- `/reference_path`: 参考路径
- `/costmap`: 代价地图
- `/move_base_simple/goal`: 目标点设置

### 输出话题
- `/cmd_vel`: 控制指令
- `/predicted_trajectory`: 预测轨迹
- `/path_markers`: 路径标记
- `/obstacle_markers`: 障碍物标记

## 算法原理

### MPPI控制算法
1. **轨迹采样**: 在当前控制序列基础上添加随机噪声
2. **轨迹预测**: 使用车辆运动学模型预测未来状态
3. **代价评估**: 计算每条轨迹的总代价
4. **权重计算**: 基于代价计算每条轨迹的权重
5. **控制更新**: 加权平均更新控制序列

### 阿克曼转向运动学模型
```
ẋ = v * cos(θ + β)
ẏ = v * sin(θ + β)
θ̇ = v * sin(β) / L

其中：
- (x, y): 车辆位置
- θ: 车辆朝向
- v: 线速度
- β: 侧偏角 = atan2(tan(δ), 2)
- δ: 转向角
- L: 轴距
```

### 贝塞尔曲线路径规划
使用三次贝塞尔曲线生成平滑路径：
```
B(t) = (1-t)³P₀ + 3t(1-t)²P₁ + 3t²(1-t)P₂ + t³P₃
```

## 性能优化

### 计算效率
- 使用Eigen库进行矩阵运算
- 并行轨迹评估
- 自适应采样数量

### 控制性能
- 实时控制频率: 10Hz
- 预测时域: 2秒
- 轨迹平滑性保证

## 故障排除

### 常见问题

1. **编译错误**
   ```bash
   # 确保安装了所有依赖
   sudo apt-get install ros-noetic-eigen
   ```

2. **Gazebo模型加载失败**
   ```bash
   # 检查URDF文件路径
   rospack find ackermann_navigation
   ```

3. **控制不响应**
   ```bash
   # 检查话题连接
   rostopic list
   rostopic echo /cmd_vel
   ```

4. **路径规划失败**
   ```bash
   # 检查代价地图
   rostopic echo /costmap
   ```

### 调试技巧
- 使用`rqt_graph`查看节点连接
- 使用`rqt_plot`监控控制信号
- 使用`rqt_console`查看日志信息

## 扩展功能

### 添加新传感器
1. 在URDF中添加传感器模型
2. 创建传感器数据处理节点
3. 更新MPPI控制器以使用传感器数据

### 自定义代价函数
1. 修改`calculatePathCost()`函数
2. 添加新的代价项
3. 调整权重参数

### 多车辆仿真
1. 复制车辆模型
2. 修改命名空间
3. 实现车辆间通信

## 许可证

Apache 2.0 License

## 贡献

欢迎提交Issue和Pull Request来改进这个项目。 