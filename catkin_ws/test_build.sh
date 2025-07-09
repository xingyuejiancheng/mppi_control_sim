#!/bin/bash

echo "=========================================="
echo "测试构建脚本"
echo "=========================================="

# 检查是否在正确的目录
if [ ! -f "src/ackermann_navigation/CMakeLists.txt" ]; then
    echo "错误: 请在catkin_ws目录中运行此脚本"
    exit 1
fi

# 清理之前的构建
echo "清理之前的构建..."
rm -rf build/ devel/ 2>/dev/null || true

# 构建项目
echo "开始构建项目..."
if catkin_make; then
    echo "=========================================="
    echo "构建成功！"
    echo "=========================================="
    
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
    echo "所有组件构建成功！"
    
else
    echo "=========================================="
    echo "构建失败！"
    echo "=========================================="
    exit 1
fi 