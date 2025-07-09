#ifndef ACKERMANN_NAVIGATION_MPPI_CONTROLLER_H
#define ACKERMANN_NAVIGATION_MPPI_CONTROLLER_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <random>

namespace ackermann_navigation {

struct Control {
    double velocity;
    double steering;
};

class MPPIController {
public:
    MPPIController();
    ~MPPIController() = default;

private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber path_sub_;
    ros::Subscriber pose_sub_;
    ros::Publisher cmd_pub_;
    ros::Timer control_timer_;
    
    // 基本参数
    int horizon_;                    // 预测步长
    int num_samples_;               // 采样数量
    double dt_;                     // 时间步长
    
    // 车辆参数
    double wheelbase_;              // 轴距
    double max_velocity_;           // 最大速度
    double min_velocity_;           // 最小速度
    double max_steering_;           // 最大转向角
    double max_steering_rate_;      // 最大转向角速率
    
    // 控制器参数
    double velocity_noise_;         // 速度采样噪声
    double steering_noise_;         // 转向采样噪声
    double temperature_;            // 温度参数
    double time_decay_;            // 时间衰减因子
    
    // 代价权重
    double lateral_weight_;         // 横向误差权重
    double yaw_weight_;            // 朝向误差权重
    double distance_weight_;        // 距离权重
    double control_weight_;         // 控制平滑权重
    double steering_weight_;        // 转向变化权重
    double goal_weight_;           // 目标吸引权重
    
    // 控制平滑
    double smoothing_factor_;       // 控制平滑因子
    
    // 新增参数
    double goal_tolerance_;         // 目标达到容差
    double yaw_tolerance_;          // 航向容差
    double lookahead_distance_;     // 前视距离
    
    // 状态变量
    std::vector<Control> controls_;              // 控制序列
    std::vector<geometry_msgs::Pose> reference_path_;  // 参考路径
    geometry_msgs::Pose current_pose_;           // 当前位姿
    
    // 随机数生成
    std::random_device rd_;
    std::mt19937 gen_;
    std::normal_distribution<double> dist_;
    
    // 功能函数
    void initialize();
    void resetControls();
    double getDistanceToPath(const geometry_msgs::Pose& pose, size_t& closest_idx);
    void generateRollouts();
    void updateControls(const std::vector<std::vector<Control>>& rollouts,
                       const std::vector<double>& costs);
                       
    // 回调函数
    void pathCallback(const nav_msgs::Path::ConstPtr& msg);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void timerCallback(const ros::TimerEvent& event);
};

} // namespace ackermann_navigation

#endif // ACKERMANN_NAVIGATION_MPPI_CONTROLLER_H 