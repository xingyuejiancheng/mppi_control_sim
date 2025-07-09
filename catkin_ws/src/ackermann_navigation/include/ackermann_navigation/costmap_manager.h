#ifndef COSTMAP_MANAGER_H
#define COSTMAP_MANAGER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <vector>
#include <cmath>

namespace ackermann_navigation {

// 表示一个圆形障碍物
struct Obstacle {
    double x, y, radius;
    Obstacle(double x = 0, double y = 0, double radius = 0.5) : x(x), y(y), radius(radius) {}
};

// 代价地图管理器，负责生成地图、管理障碍物、发布可视化
class CostmapManager {
public:
    CostmapManager(); // 构造函数，初始化ROS节点和参数
    ~CostmapManager() = default;

private:
    // ROS相关
    ros::NodeHandle nh_;                // 全局节点句柄
    ros::NodeHandle private_nh_;        // 私有节点句柄
    ros::Publisher map_pub_;            // 地图发布器
    ros::Publisher marker_pub_;         // 障碍物Marker发布器
    ros::Subscriber obstacle_sub_;      // 障碍物添加订阅器
    ros::Timer map_timer_;              // 定时器，定期发布地图

    // 地图参数
    double map_width_;                  // 地图宽度（米）
    double map_height_;                 // 地图高度（米）
    double map_resolution_;             // 地图分辨率（米/格）
    double map_origin_x_;               // 地图原点X
    double map_origin_y_;               // 地图原点Y
    
    // 障碍物列表
    std::vector<Obstacle> obstacles_;
    nav_msgs::OccupancyGrid costmap_;   // 代价地图
    
    // 回调函数
    void obstacleCallback(const geometry_msgs::PoseStamped::ConstPtr& msg); // 添加障碍物
    void mapTimerCallback(const ros::TimerEvent& event);                    // 定时发布地图
    
    // 地图生成与障碍物处理
    void generateCostmap();                 // 生成代价地图
    void addObstacle(const Obstacle& obstacle); // 在地图上添加障碍物
    double calculateObstacleCost(double x, double y); // 计算某点的障碍物代价
    
    // 可视化
    void publishObstacleMarkers();          // 发布障碍物Marker
    
    // 辅助函数
    int worldToGridX(double x);             // 世界坐标转网格X
    int worldToGridY(double y);             // 世界坐标转网格Y
    double gridToWorldX(int grid_x);        // 网格X转世界坐标
    double gridToWorldY(int grid_y);        // 网格Y转世界坐标
    bool isValidGrid(int grid_x, int grid_y); // 检查网格是否有效
};

} // namespace ackermann_navigation

#endif // COSTMAP_MANAGER_H 