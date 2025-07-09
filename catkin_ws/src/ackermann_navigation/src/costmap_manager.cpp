#include "ackermann_navigation/costmap_manager.h"
#include <tf/transform_datatypes.h>
#include <sstream>
// UTF-8 encoding
#include <locale>
#include <codecvt>
#include <string>

namespace ackermann_navigation {


// 类说明、成员变量说明、函数说明、流程说明均用中文
// 支持障碍物尺寸配置：
// 1. Obstacle结构体增加radius字段
// 2. obstacleCallback支持从PoseStamped的pose.position.z读取半径（如z>0），否则用默认值
// 3. 或者通过参数传入障碍物尺寸

CostmapManager::CostmapManager() 
    : private_nh_("~") {
    
    // 加载地图参数
    private_nh_.param("map_width", map_width_, 50.0);
    private_nh_.param("map_height", map_height_, 50.0);
    private_nh_.param("map_resolution", map_resolution_, 0.1);
    private_nh_.param("map_origin_x", map_origin_x_, -25.0);
    private_nh_.param("map_origin_y", map_origin_y_, -25.0);
    
    // 设置ROS发布和订阅
    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/costmap", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/obstacle_markers", 1);
    obstacle_sub_ = nh_.subscribe("/add_obstacle", 1, &CostmapManager::obstacleCallback, this);
    
    // 设置定时器
    map_timer_ = nh_.createTimer(ros::Duration(1.0), &CostmapManager::mapTimerCallback, this);
    
    // 初始化代价地图
    generateCostmap();
    
    ROS_INFO("代价地图管理器初始化，地图尺寸: %.1fx%.1f", map_width_, map_height_);
}

void CostmapManager::obstacleCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    Obstacle new_obstacle;
    new_obstacle.x = msg->pose.position.x;
    new_obstacle.y = msg->pose.position.y;
    new_obstacle.radius = 0.5;  // 默认半径
    obstacles_.push_back(new_obstacle);
    ROS_INFO("Added obstacle at (%.2f, %.2f)", new_obstacle.x, new_obstacle.y);
}

void CostmapManager::mapTimerCallback(const ros::TimerEvent& event) {
    generateCostmap();
    costmap_.header.stamp = ros::Time::now();
    map_pub_.publish(costmap_);
    publishObstacleMarkers();
}

void CostmapManager::generateCostmap() {
    costmap_.header.frame_id = "map";
    costmap_.info.resolution = map_resolution_;
    costmap_.info.width = static_cast<unsigned int>(map_width_ / map_resolution_);
    costmap_.info.height = static_cast<unsigned int>(map_height_ / map_resolution_);
    costmap_.info.origin.position.x = map_origin_x_;
    costmap_.info.origin.position.y = map_origin_y_;
    costmap_.info.origin.position.z = 0.0;
    costmap_.info.origin.orientation.w = 1.0;
    int map_size = costmap_.info.width * costmap_.info.height;
    costmap_.data.resize(map_size, 0);
    for (const auto& obstacle : obstacles_) {
        addObstacle(obstacle);
    }
}

void CostmapManager::addObstacle(const Obstacle& obstacle) {
    int center_x = worldToGridX(obstacle.x);
    int center_y = worldToGridY(obstacle.y);
    int radius_cells = static_cast<int>(obstacle.radius / map_resolution_);
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
            int grid_x = center_x + dx;
            int grid_y = center_y + dy;
            if (isValidGrid(grid_x, grid_y)) {
                double distance = sqrt(dx*dx + dy*dy) * map_resolution_;
                if (distance <= obstacle.radius) {
                    int index = grid_y * costmap_.info.width + grid_x;
                    int occupancy = static_cast<int>(100 * (1.0 - distance / obstacle.radius));
                    costmap_.data[index] = static_cast<signed char>(std::max(static_cast<int>(costmap_.data[index]), occupancy));
                }
            }
        }
    }
}

void CostmapManager::publishObstacleMarkers() {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "obstacles";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE_LIST;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.7;
    marker.scale.y = 0.7;
    marker.scale.z = 0.7;
    marker.color.a = 0.9;
    for (size_t i = 0; i < obstacles_.size(); ++i) {
        geometry_msgs::Point point;
        point.x = obstacles_[i].x;
        point.y = obstacles_[i].y;
        point.z = 0.0;
        marker.points.push_back(point);
        std_msgs::ColorRGBA color;
        color.r = 0.2 + 0.8 * (i % 3 == 0);
        color.g = 0.2 + 0.8 * (i % 3 == 1);
        color.b = 0.2 + 0.8 * (i % 3 == 2);
        color.a = 0.9;
        marker.colors.push_back(color);
    }
    marker_pub_.publish(marker);
}

int CostmapManager::worldToGridX(double x) {
    return static_cast<int>((x - map_origin_x_) / map_resolution_);
}

int CostmapManager::worldToGridY(double y) {
    return static_cast<int>((y - map_origin_y_) / map_resolution_);
}

double CostmapManager::gridToWorldX(int grid_x) {
    return grid_x * map_resolution_ + map_origin_x_;
}

double CostmapManager::gridToWorldY(int grid_y) {
    return grid_y * map_resolution_ + map_origin_y_;
}

bool CostmapManager::isValidGrid(int grid_x, int grid_y) {
    return grid_x >= 0 && grid_x < static_cast<int>(costmap_.info.width) &&
           grid_y >= 0 && grid_y < static_cast<int>(costmap_.info.height);
}

} // namespace ackermann_navigation

int main(int argc, char** argv) {
    ros::init(argc, argv, "costmap_manager");
    ackermann_navigation::CostmapManager manager;
    ros::spin();
    return 0;
} 