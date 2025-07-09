#include "ackermann_navigation/path_planner.h"
#include <tf/transform_datatypes.h>
#include <algorithm>
#include <random>
#include <cmath>
#include <sstream>

namespace ackermann_navigation {

// 详细注释示例：
// 类说明、成员变量说明、函数说明、流程说明均用中文
// 支持多路径点配置：
// 1. waypoints_可通过服务/参数/批量goal消息设置
// 2. goalCallback支持批量目标点

PathPlanner::PathPlanner() 
    : private_nh_("~"),
      map_received_(false) {
    
    // 加载参数
    private_nh_.param("path_resolution", path_resolution_, 0.1);
    private_nh_.param("max_curvature", max_curvature_, 0.5);
    private_nh_.param("vehicle_width", vehicle_width_, 1.0);
    private_nh_.param("vehicle_length", vehicle_length_, 2.0);
    private_nh_.param("safety_margin", safety_margin_, 0.5);
    private_nh_.param("bezier_order", bezier_order_, 3);
    
    // 设置ROS订阅和发布
    map_sub_ = nh_.subscribe("/costmap", 1, &PathPlanner::mapCallback, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 10, &PathPlanner::goalCallback, this);
    
    path_pub_ = nh_.advertise<nav_msgs::Path>("/reference_path", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/path_markers", 1);
    
    // 设置定时器
    planning_timer_ = nh_.createTimer(ros::Duration(1.0), &PathPlanner::planningTimerCallback, this);
    
    // 初始化路径点为空
    waypoints_.clear();
    
    ROS_INFO("路径规划器初始化完成");
}

void PathPlanner::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    costmap_ = *msg;
    map_received_ = true;
    ROS_DEBUG("Received costmap with resolution: %f", costmap_.info.resolution);
}

void PathPlanner::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    Waypoint new_waypoint;
    new_waypoint.x = msg->pose.position.x;
    new_waypoint.y = msg->pose.position.y;
    
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.orientation, q);
    new_waypoint.theta = tf::getYaw(q);
    
    waypoints_.push_back(new_waypoint);
    ROS_INFO("Added new waypoint at (%.2f, %.2f), 当前waypoints数量: %zu", new_waypoint.x, new_waypoint.y, waypoints_.size());
    // 新增：收到目标点后立即尝试规划路径
    ros::TimerEvent dummy_event;
    planningTimerCallback(dummy_event);
}

void PathPlanner::planningTimerCallback(const ros::TimerEvent& event) {
    ROS_INFO("planningTimerCallback触发，当前waypoints数量: %zu, map_received: %d", waypoints_.size(), map_received_);
    if (!map_received_ || waypoints_.size() < 2) {
        ROS_WARN("条件不足，不发布路径。map_received=%d, waypoints_.size()=%zu", map_received_, waypoints_.size());
        return;
    }
    
    // 生成路径
    current_path_ = generateBezierPath(waypoints_);
    
    // 检查碰撞并优化
    if (!isPathCollisionFree(current_path_)) {
        current_path_ = optimizePath(current_path_);
    }
    
    // 发布路径
    current_path_.header.frame_id = "map";
    current_path_.header.stamp = ros::Time::now();
    path_pub_.publish(current_path_);
    ROS_INFO("已发布路径，点数: %zu", current_path_.poses.size());
    
    // 发布可视化标记
    publishControlPoints();
    publishWaypoints();
}

nav_msgs::Path PathPlanner::generateBezierPath(const std::vector<Waypoint>& waypoints) {
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();
    
    if (waypoints.size() < 2) {
        return path;
    }
    
    // 多段路径拼接
    for (size_t seg = 0; seg < waypoints.size() - 1; ++seg) {
        std::vector<Waypoint> seg_pts;
        seg_pts.push_back(waypoints[seg]);
        seg_pts.push_back(waypoints[seg + 1]);
        auto seg_ctrl = calculateControlPoints(seg_pts);
        for (double t = 0.0; t <= 1.0; t += path_resolution_) {
            geometry_msgs::PoseStamped pose;
            pose.pose.position = evaluateBezierCurve(seg_ctrl, t);
            
            double t_next = std::min(t + path_resolution_, 1.0);
            geometry_msgs::Point next_point = evaluateBezierCurve(seg_ctrl, t_next);
            
            double dx = next_point.x - pose.pose.position.x;
            double dy = next_point.y - pose.pose.position.y;
            double theta = atan2(dy, dx);
            
            tf::Quaternion q = tf::createQuaternionFromYaw(theta);
            tf::quaternionTFToMsg(q, pose.pose.orientation);
            
            path.poses.push_back(pose);
        }
    }
    
    return path;
}

std::vector<geometry_msgs::Point> PathPlanner::calculateControlPoints(const std::vector<Waypoint>& waypoints) {
    std::vector<geometry_msgs::Point> control_points;
    
    if (waypoints.size() < 2) {
        return control_points;
    }
    
    geometry_msgs::Point p0, p1, p2, p3;
    p0.x = waypoints[0].x; p0.y = waypoints[0].y;
    p3.x = waypoints[1].x; p3.y = waypoints[1].y;
    
    // 控制点1/2简单设置为起点/终点附近
    p1.x = p0.x + 0.3 * (p3.x - p0.x);
    p1.y = p0.y + 0.3 * (p3.y - p0.y);
    p2.x = p0.x + 0.7 * (p3.x - p0.x);
    p2.y = p0.y + 0.7 * (p3.y - p0.y);
    
    control_points = {p0, p1, p2, p3};
    return control_points;
}

geometry_msgs::Point PathPlanner::evaluateBezierCurve(const std::vector<geometry_msgs::Point>& control_points, double t) {
    geometry_msgs::Point result;
    
    if (control_points.size() < 4) {
        return result;
    }
    
    double t2 = t * t;
    double t3 = t2 * t;
    double mt = 1.0 - t;
    double mt2 = mt * mt;
    double mt3 = mt2 * mt;
    
    result.x = mt3 * control_points[0].x + 
               3 * mt2 * t * control_points[1].x + 
               3 * mt * t2 * control_points[2].x + 
               t3 * control_points[3].x;
               
    result.y = mt3 * control_points[0].y + 
               3 * mt2 * t * control_points[1].y + 
               3 * mt * t2 * control_points[2].y + 
               t3 * control_points[3].y;
    
    return result;
}

double PathPlanner::calculateCurvature(const std::vector<geometry_msgs::Point>& control_points, double t) {
    if (control_points.size() < 4) {
        return 0.0;
    }
    
    // 计算一阶和二阶导数
    double t2 = t * t;
    double mt = 1.0 - t;
    double mt2 = mt * mt;
    
    // 一阶导数
    double dx_dt = -3 * mt2 * control_points[0].x + 
                   (3 * mt2 - 6 * mt * t) * control_points[1].x + 
                   (-3 * t2 + 6 * mt * t) * control_points[2].x + 
                   3 * t2 * control_points[3].x;
                   
    double dy_dt = -3 * mt2 * control_points[0].y + 
                   (3 * mt2 - 6 * mt * t) * control_points[1].y + 
                   (-3 * t2 + 6 * mt * t) * control_points[2].y + 
                   3 * t2 * control_points[3].y;
    
    // 二阶导数
    double d2x_dt2 = 6 * mt * control_points[0].x + 
                     (-12 * mt + 6 * t) * control_points[1].x + 
                     (6 * mt - 12 * t) * control_points[2].x + 
                     6 * t * control_points[3].x;
                     
    double d2y_dt2 = 6 * mt * control_points[0].y + 
                     (-12 * mt + 6 * t) * control_points[1].y + 
                     (6 * mt - 12 * t) * control_points[2].y + 
                     6 * t * control_points[3].y;
    
    // 曲率公式: κ = |x'y'' - y'x''| / (x'^2 + y'^2)^(3/2)
    double numerator = fabs(dx_dt * d2y_dt2 - dy_dt * d2x_dt2);
    double denominator = pow(dx_dt * dx_dt + dy_dt * dy_dt, 1.5);
    
    if (denominator < 1e-6) {
        return 0.0;
    }
    
    return numerator / denominator;
}

bool PathPlanner::isPathCollisionFree(const nav_msgs::Path& path) {
    for (const auto& pose : path.poses) {
        if (isPointInCollision(pose.pose.position.x, pose.pose.position.y)) {
            return false;
        }
    }
    return true;
}

bool PathPlanner::isPointInCollision(double x, double y) {
    if (!map_received_) {
        return false;
    }
    
    // 转换为地图坐标
    double map_x = (x - costmap_.info.origin.position.x) / costmap_.info.resolution;
    double map_y = (y - costmap_.info.origin.position.y) / costmap_.info.resolution;
    
    int grid_x = static_cast<int>(map_x);
    int grid_y = static_cast<int>(map_y);
    
    // 检查边界
    if (grid_x < 0 || grid_x >= static_cast<int>(costmap_.info.width) || 
        grid_y < 0 || grid_y >= static_cast<int>(costmap_.info.height)) {
        return true;
    }
    
    // 检查车辆周围的网格
    int vehicle_radius = static_cast<int>((vehicle_width_/2 + safety_margin_) / costmap_.info.resolution);
    
    for (int dx = -vehicle_radius; dx <= vehicle_radius; ++dx) {
        for (int dy = -vehicle_radius; dy <= vehicle_radius; ++dy) {
            int check_x = grid_x + dx;
            int check_y = grid_y + dy;
            
            if (check_x >= 0 && check_x < static_cast<int>(costmap_.info.width) &&
                check_y >= 0 && check_y < static_cast<int>(costmap_.info.height)) {
                
                int index = check_y * costmap_.info.width + check_x;
                if (index < static_cast<int>(costmap_.data.size()) && costmap_.data[index] > 50) {
                    return true;
                }
            }
        }
    }
    
    return false;
}

nav_msgs::Path PathPlanner::optimizePath(const nav_msgs::Path& initial_path) {
    nav_msgs::Path optimized_path = initial_path;
    
    // 简单的路径优化：调整控制点位置
    if (control_points_.size() >= 4) {
        // 随机调整控制点位置
        std::random_device rd;
        std::mt19937 gen(rd());
        std::normal_distribution<double> noise(0.0, 0.5);
        
        for (size_t i = 1; i < control_points_.size() - 1; ++i) {
            control_points_[i].x += noise(gen);
            control_points_[i].y += noise(gen);
        }
        
        // 重新生成路径
        optimized_path = generateBezierPath(waypoints_);
    }
    
    return optimized_path;
}

void PathPlanner::publishControlPoints() {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "control_points";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.scale.x = 0.2;
    marker.scale.y = 0.2;
    marker.scale.z = 0.2;
    
    marker.color.r = 0.0;
    marker.color.g = 0.5;
    marker.color.b = 1.0;
    marker.color.a = 1.0;
    
    for (const auto& wp : waypoints_) {
        geometry_msgs::Point pt;
        pt.x = wp.x; pt.y = wp.y; pt.z = 0.0;
        marker.points.push_back(pt);
    }
    
    marker_pub_.publish(marker);
}

void PathPlanner::publishWaypoints() {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "waypoints";
    marker.id = 1;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    
    for (size_t i = 0; i < waypoints_.size(); ++i) {
        geometry_msgs::Point pt;
        pt.x = waypoints_[i].x; pt.y = waypoints_[i].y; pt.z = 0.0;
        marker.points.push_back(pt);
        std_msgs::ColorRGBA color;
        if (i == 0) { color.r = 0.0; color.g = 1.0; color.b = 0.0; } // 起点绿色
        else if (i == waypoints_.size() - 1) { color.r = 1.0; color.g = 0.0; color.b = 0.0; } // 终点红色
        else { color.r = 1.0; color.g = 1.0; color.b = 0.0; } // 中间点黄色
        color.a = 1.0;
        marker.colors.push_back(color);
    }
    
    marker_pub_.publish(marker);
}

double PathPlanner::distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    return sqrt(dx*dx + dy*dy);
}

double PathPlanner::angleBetweenPoints(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2) {
    return atan2(p2.y - p1.y, p2.x - p1.x);
}

} // namespace ackermann_navigation

int main(int argc, char** argv) {
    ros::init(argc, argv, "path_planner");
    ackermann_navigation::PathPlanner planner;
    ros::spin();
    return 0;
} 