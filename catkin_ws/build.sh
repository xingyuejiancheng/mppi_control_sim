#!/bin/bash

# 阿克曼车辆导航系统构建脚本
# 适用于Linux系统

set -e  # 遇到错误时退出

echo "=========================================="
echo "阿克曼车辆导航系统构建脚本"
echo "=========================================="

# 检查是否在正确的目录
if [ ! -f "src/ackermann_navigation/CMakeLists.txt" ]; then
    echo "错误: 请在catkin_ws目录中运行此脚本"
    exit 1
fi

# 检查ROS环境
if [ -z "$ROS_DISTRO" ]; then
    echo "警告: ROS环境未设置，尝试设置ROS Noetic..."
    source /opt/ros/noetic/setup.bash 2>/dev/null || {
        echo "错误: 无法找到ROS Noetic，请确保已正确安装ROS"
        exit 1
    }
fi

echo "ROS版本: $ROS_DISTRO"

# 检查必要的工具
echo "检查构建工具..."

if ! command -v catkin_make &> /dev/null; then
    echo "错误: catkin_make未找到，请安装catkin"
    exit 1
fi

if ! command -v gcc &> /dev/null; then
    echo "错误: gcc未找到，请安装gcc"
    exit 1
fi

echo "构建工具检查完成"

# 清理之前的构建
echo "清理之前的构建..."
rm -rf build/ devel/ 2>/dev/null || true

# 检查依赖项
echo "检查ROS依赖项..."
if ! rosdep check ackermann_navigation --from-paths src --ignore-src 2>/dev/null; then
    echo "警告: 某些依赖项可能缺失，尝试安装..."
    rosdep install --from-paths src --ignore-src -y --skip-keys="gazebo_ros" || {
        echo "警告: 无法安装所有依赖项，继续构建..."
    }
fi

# 构建项目
echo "开始构建项目..."
if catkin_make; then
    echo "=========================================="
    echo "构建成功！"
    echo "=========================================="
    
    # 设置环境
    source devel/setup.bash
    
    # 检查生成的可执行文件
    echo "检查生成的可执行文件..."
    if [ -f "devel/lib/ackermann_navigation/mppi_controller" ]; then
        echo "✓ mppi_controller"
    else
        echo "✗ mppi_controller"
    fi
    
    if [ -f "devel/lib/ackermann_navigation/path_planner" ]; then
        echo "✓ path_planner"
    else
        echo "✗ path_planner"
    fi
    
    if [ -f "devel/lib/ackermann_navigation/costmap_manager" ]; then
        echo "✓ costmap_manager"
    else
        echo "✗ costmap_manager"
    fi
    
    if [ -f "devel/lib/ackermann_navigation/fake_odom_publisher" ]; then
        echo "✓ fake_odom_publisher"
    else
        echo "✗ fake_odom_publisher"
    fi
    
    echo ""
    echo "运行说明:"
    echo "1. 启动ROS Master: roscore"
    echo "2. 启动系统: roslaunch ackermann_navigation simulation.launch"
    echo "3. 运行测试: python3 src/ackermann_navigation/scripts/simple_test.py"
    echo ""
    echo "更多信息请参考 BUILD_INSTRUCTIONS.md"
    
else
    echo "=========================================="
    echo "构建失败！"
    echo "=========================================="
    echo "请检查以下可能的问题："
    echo "1. ROS环境是否正确设置"
    echo "2. 所有依赖项是否安装"
    echo "3. 代码是否有语法错误"
    echo "4. 编译器版本是否支持C++11"
    echo ""
    echo "详细错误信息请查看上面的输出"
    exit 1
fi 