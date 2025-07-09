#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>
#include <vector>
#include <cmath>

namespace ackermann_navigation {

// 路径点结构体，包含位置和朝向
struct Waypoint {
    double x, y, theta;
    Waypoint(double x = 0, double y = 0, double theta = 0) : x(x), y(y), theta(theta) {}
};

// 路径规划器，负责接收目标点、生成路径、可视化
class PathPlanner {
public:
    PathPlanner(); // 构造函数，初始化参数和ROS接口
    ~PathPlanner() = default;

private:
    // ROS相关
    ros::NodeHandle nh_;                // 全局节点句柄
    ros::NodeHandle private_nh_;        // 私有节点句柄
    ros::Subscriber map_sub_;           // 地图订阅器
    ros::Subscriber goal_sub_;          // 目标点订阅器
    ros::Publisher path_pub_;           // 路径发布器
    ros::Publisher marker_pub_;         // 可视化Marker发布器
    tf::TransformListener tf_listener_; // TF监听器

    // 路径规划参数
    double path_resolution_;            // 路径分辨率
    double max_curvature_;              // 最大曲率
    double vehicle_width_;              // 车辆宽度
    double vehicle_length_;             // 车辆长度
    double safety_margin_;              // 安全边距
    
    // 贝塞尔曲线参数
    int bezier_order_;                  // 贝塞尔曲线阶数
    std::vector<geometry_msgs::Point> control_points_; // 控制点
    
    // 地图和路径
    nav_msgs::OccupancyGrid costmap_;   // 代价地图
    nav_msgs::Path current_path_;       // 当前路径
    std::vector<Waypoint> waypoints_;   // 路径点列表
    bool map_received_;                 // 地图是否已接收
    
    // 回调函数
    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg); // 地图回调
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg); // 目标点回调
    
    // 路径规划函数
    nav_msgs::Path generateBezierPath(const std::vector<Waypoint>& waypoints); // 生成贝塞尔路径
    std::vector<geometry_msgs::Point> calculateControlPoints(const std::vector<Waypoint>& waypoints); // 计算控制点
    geometry_msgs::Point evaluateBezierCurve(const std::vector<geometry_msgs::Point>& control_points, double t); // 评估贝塞尔曲线点
    double calculateCurvature(const std::vector<geometry_msgs::Point>& control_points, double t); // 计算曲率
    
    // 障碍物避免
    bool isPathCollisionFree(const nav_msgs::Path& path); // 路径碰撞检测
    bool isPointInCollision(double x, double y);          // 点碰撞检测
    nav_msgs::Path optimizePath(const nav_msgs::Path& initial_path); // 路径优化
    
    // 可视化
    void publishControlPoints();      // 发布控制点Marker
    void publishWaypoints();          // 发布路径点Marker
    
    // 辅助函数
    double distance(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2); // 两点距离
    double angleBetweenPoints(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2); // 两点夹角
    std::vector<Waypoint> generateDefaultWaypoints(); // 生成默认路径点
    
    // 定时器
    ros::Timer planning_timer_;       // 路径规划定时器
    void planningTimerCallback(const ros::TimerEvent& event); // 定时器回调
};

} // namespace ackermann_navigation

#endif // PATH_PLANNER_H 