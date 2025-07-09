#include "ackermann_navigation/mppi_controller.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>
#include <cmath>
#include <algorithm>
#include <limits>
#include <locale>
#include <codecvt>
#include <string>

namespace ackermann_navigation {

// 详细注释示例：
// 类说明、成员变量说明、函数说明、流程说明均用中文
// 日志为简体中文

MPPIController::MPPIController() 
    : private_nh_("~"),
      gen_(rd_()),
      noise_v_(0.0, 1.0),
      noise_delta_(0.0, 1.0),
      path_received_(false),
      map_received_(false) {
    
    // 加载MPPI参数
    private_nh_.param("horizon", horizon_, 20);
    private_nh_.param("num_samples", num_samples_, 1500);
    private_nh_.param("dt", dt_, 0.1);
    private_nh_.param("wheelbase", wheelbase_, 1.0);
    private_nh_.param("max_velocity", max_velocity_, 0.8);
    private_nh_.param("min_velocity", min_velocity_, 0.3);
    private_nh_.param("max_steering", max_steering_, 0.4);
    private_nh_.param("max_steering_rate", max_steering_rate_, 0.3);
    private_nh_.param("goal_tolerance", goal_tolerance_, 0.5);
    private_nh_.param("yaw_tolerance", yaw_tolerance_, 0.2);
    private_nh_.param("lookahead_distance", lookahead_distance_, 1.0);
    
    // 加载车辆参数
    private_nh_.param("noise_std_v", noise_std_v_, 0.3);
    private_nh_.param("noise_std_delta", noise_std_delta_, 0.05);
    private_nh_.param("temperature", temperature_, 0.5);
    
    // 加载代价函数权重
    private_nh_.param("w_path", w_path_, 2.0);
    private_nh_.param("w_obstacle", w_obstacle_, 10.0);
    private_nh_.param("w_control", w_control_, 0.2);
    private_nh_.param("w_velocity", w_velocity_, 0.3);
    
    // 初始化随机数生成器
    noise_v_ = std::normal_distribution<double>(0.0, noise_std_v_);
    noise_delta_ = std::normal_distribution<double>(0.0, noise_std_delta_);
    
    // 设置ROS订阅和发布
    odom_sub_ = nh_.subscribe("/ackermann/odom", 1, &MPPIController::odomCallback, this);
    path_sub_ = nh_.subscribe("/reference_path", 1, &MPPIController::pathCallback, this);
    map_sub_ = nh_.subscribe("/costmap", 1, &MPPIController::mapCallback, this);
    
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    trajectory_pub_ = nh_.advertise<nav_msgs::Path>("/predicted_trajectory", 1);
    
    // 设置控制定时器
    control_timer_ = nh_.createTimer(ros::Duration(dt_), &MPPIController::timerCallback, this);
    
    ROS_INFO("[MPPI] 初始化完成，最大速度: %.2f, 最大转向: %.2f", max_velocity_, max_steering_);
}

void MPPIController::odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    // 更新当前车辆状态
    current_state_.x = msg->pose.pose.position.x;
    current_state_.y = msg->pose.pose.position.y;
    
    // 从四元数提取偏航角
    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q);
    current_state_.theta = tf::getYaw(q);
    
    current_state_.v = sqrt(pow(msg->twist.twist.linear.x, 2) + pow(msg->twist.twist.linear.y, 2));
    if (current_state_.v > 0.1) {
        current_state_.delta = atan2(wheelbase_ * msg->twist.twist.angular.z, current_state_.v);
    } else {
        current_state_.delta = 0.0;
    }
}

void MPPIController::pathCallback(const nav_msgs::Path::ConstPtr& msg) {
    reference_path_ = *msg;
    path_received_ = true;
    ROS_DEBUG("Received reference path with %lu points", reference_path_.poses.size());
}

void MPPIController::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    costmap_ = *msg;
    map_received_ = true;
    ROS_DEBUG("Received costmap with resolution: %f", costmap_.info.resolution);
}

void MPPIController::timerCallback(const ros::TimerEvent& event) {
    if (!path_received_ || !map_received_) {
        ROS_WARN_THROTTLE(1.0, "Waiting for path and map data...");
        return;
    }
    
    controlLoop();
}

void MPPIController::controlLoop() {
    // 生成标称控制序列
    std::vector<ControlInput> nominal_controls(horizon_);
    for (int i = 0; i < horizon_; ++i) {
        nominal_controls[i].v = std::max(min_velocity_, std::min(max_velocity_, current_state_.v));
        nominal_controls[i].delta = std::max(-max_steering_, std::min(max_steering_, current_state_.delta));
    }
    
    // 生成轨迹样本
    auto trajectories = generateTrajectories(current_state_, nominal_controls);
    
    // 评估轨迹代价
    auto costs = evaluateTrajectories(trajectories, nominal_controls);
    
    // 更新控制序列
    auto updated_controls = updateControls(nominal_controls, trajectories, costs);
    
    // 应用第一个控制指令
    geometry_msgs::Twist cmd;
    cmd.linear.x = updated_controls[0].v;
    cmd.angular.z = updated_controls[0].v * tan(updated_controls[0].delta) / wheelbase_;
    cmd_pub_.publish(cmd);
    
    // 发布预测轨迹用于可视化
    if (!trajectories.empty()) {
        publishTrajectory(trajectories[0]);
    }
}

std::vector<std::vector<VehicleState>> MPPIController::generateTrajectories(
    const VehicleState& initial_state,
    const std::vector<ControlInput>& nominal_controls) {
    
    std::vector<std::vector<VehicleState>> trajectories(num_samples_);
    
    for (int i = 0; i < num_samples_; ++i) {
        trajectories[i].resize(horizon_ + 1);
        trajectories[i][0] = initial_state;
        
        for (int j = 0; j < horizon_; ++j) {
            // 添加噪声到控制输入
            ControlInput noisy_control = nominal_controls[j];
            noisy_control.v += noise_v_(gen_) * noise_std_v_;
            noisy_control.delta += noise_delta_(gen_) * noise_std_delta_;
            
            // 限制控制输入
            noisy_control.v = std::max(min_velocity_, std::min(max_velocity_, noisy_control.v));
            noisy_control.delta = std::max(-max_steering_, std::min(max_steering_, noisy_control.delta));
            
            // 预测下一个状态
            trajectories[i][j + 1] = predictState(trajectories[i][j], noisy_control, dt_);
        }
    }
    
    return trajectories;
}

VehicleState MPPIController::predictState(const VehicleState& state, const ControlInput& control, double dt) {
    VehicleState next_state = state;
    
    // 阿克曼转向运动学模型
    double beta = atan2(tan(control.delta), 2.0);  // 侧偏角近似
    
    next_state.x += control.v * cos(state.theta + beta) * dt;
    next_state.y += control.v * sin(state.theta + beta) * dt;
    next_state.theta += control.v * sin(beta) / wheelbase_ * dt;
    next_state.v = control.v;
    next_state.delta = control.delta;
    
    return next_state;
}

std::vector<double> MPPIController::evaluateTrajectories(
    const std::vector<std::vector<VehicleState>>& trajectories,
    const std::vector<ControlInput>& nominal_controls) {
    
    std::vector<double> costs(num_samples_);
    
    for (int i = 0; i < num_samples_; ++i) {
        double total_cost = 0.0;
        
        for (int j = 0; j <= horizon_; ++j) {
            const auto& state = trajectories[i][j];
            
            // 路径跟踪代价
            total_cost += w_path_ * calculatePathCost(state);
            
            // 障碍物代价
            total_cost += w_obstacle_ * calculateObstacleCost(state);
            
            // 速度代价
            total_cost += w_velocity_ * calculateVelocityCost(state);
            
            // 控制代价（除了最后一个状态）
            if (j < horizon_) {
                total_cost += w_control_ * calculateControlCost(nominal_controls[j]);
            }
        }
        
        costs[i] = total_cost;
    }
    
    return costs;
}

double MPPIController::calculatePathCost(const VehicleState& state) {
    return getDistanceToPath(state);
}

double MPPIController::calculateObstacleCost(const VehicleState& state) {
    if (isInCollision(state)) {
        return 1000.0;  // 高代价表示碰撞
    }
    return 0.0;
}

double MPPIController::calculateControlCost(const ControlInput& control) {
    // 控制平滑性代价
    double cost = 0.0;
    cost += 0.1 * pow(control.v - current_state_.v, 2);
    cost += 0.1 * pow(control.delta - current_state_.delta, 2);
    return cost;
}

double MPPIController::calculateVelocityCost(const VehicleState& state) {
    // 鼓励保持合理速度
    double target_velocity = 2.0;  // 目标速度
    return pow(state.v - target_velocity, 2);
}

double MPPIController::getDistanceToPath(const VehicleState& state) {
    if (reference_path_.poses.empty()) {
        return 0.0;
    }
    
    double min_distance = std::numeric_limits<double>::max();
    double heading_cost = 0.0;
    size_t closest_idx = 0;
    
    // 找到最近的路径点
    for (size_t i = 0; i < reference_path_.poses.size(); ++i) {
        const auto& pose = reference_path_.poses[i];
        double dx = state.x - pose.pose.position.x;
        double dy = state.y - pose.pose.position.y;
        double distance = sqrt(dx*dx + dy*dy);
        if (distance < min_distance) {
            min_distance = distance;
            closest_idx = i;
        }
    }
    
    // 计算期望朝向
    double target_theta;
    if (closest_idx < reference_path_.poses.size() - 1) {
        // 使用当前点和下一个点计算期望朝向
        const auto& current = reference_path_.poses[closest_idx];
        const auto& next = reference_path_.poses[closest_idx + 1];
        target_theta = atan2(next.pose.position.y - current.pose.position.y,
                           next.pose.position.x - current.pose.position.x);
    } else {
        // 对于最后一个点，使用其四元数中的朝向
        tf::Quaternion q;
        tf::quaternionMsgToTF(reference_path_.poses[closest_idx].pose.orientation, q);
        target_theta = tf::getYaw(q);
    }
    
    // 计算朝向差异（归一化到[-pi, pi]）
    double theta_diff = state.theta - target_theta;
    while (theta_diff > M_PI) theta_diff -= 2*M_PI;
    while (theta_diff < -M_PI) theta_diff += 2*M_PI;
    
    // 综合距离和朝向代价
    heading_cost = 2.0 * fabs(theta_diff);  // 增加朝向权重
    
    return min_distance + heading_cost;
}

bool MPPIController::isInCollision(const VehicleState& state) {
    if (!map_received_) {
        return false;
    }
    
    // 将车辆位置转换为地图坐标
    double map_x = (state.x - costmap_.info.origin.position.x) / costmap_.info.resolution;
    double map_y = (state.y - costmap_.info.origin.position.y) / costmap_.info.resolution;
    
    int grid_x = static_cast<int>(map_x);
    int grid_y = static_cast<int>(map_y);
    
    // 检查边界
    if (grid_x < 0 || grid_x >= static_cast<int>(costmap_.info.width) || 
        grid_y < 0 || grid_y >= static_cast<int>(costmap_.info.height)) {
        return true;  // 超出地图边界
    }
    
    // 检查车辆周围的网格
    int vehicle_radius = static_cast<int>(0.5 / costmap_.info.resolution);  // 车辆半径
    
    for (int dx = -vehicle_radius; dx <= vehicle_radius; ++dx) {
        for (int dy = -vehicle_radius; dy <= vehicle_radius; ++dy) {
            int check_x = grid_x + dx;
            int check_y = grid_y + dy;
            
            if (check_x >= 0 && check_x < static_cast<int>(costmap_.info.width) &&
                check_y >= 0 && check_y < static_cast<int>(costmap_.info.height)) {
                
                int index = check_y * costmap_.info.width + check_x;
                if (index < static_cast<int>(costmap_.data.size()) && costmap_.data[index] > 50) {
                    return true;  // 检测到障碍物
                }
            }
        }
    }
    
    return false;
}

std::vector<ControlInput> MPPIController::updateControls(
    const std::vector<ControlInput>& nominal_controls,
    const std::vector<std::vector<VehicleState>>& trajectories,
    const std::vector<double>& costs) {
    
    std::vector<ControlInput> updated_controls = nominal_controls;
    
    // 找到最小代价及其对应的轨迹索引
    double min_cost = std::numeric_limits<double>::max();
    size_t best_trajectory_idx = 0;
    for (size_t i = 0; i < costs.size(); ++i) {
        if (costs[i] < min_cost) {
            min_cost = costs[i];
            best_trajectory_idx = i;
        }
    }
    
    // 计算权重
    std::vector<double> weights(num_samples_);
    double total_weight = 0.0;
    
    for (int i = 0; i < num_samples_; ++i) {
        weights[i] = exp(-(costs[i] - min_cost) / temperature_);
        total_weight += weights[i];
    }
    
    // 归一化权重
    if (total_weight > 0) {
        for (int i = 0; i < num_samples_; ++i) {
            weights[i] /= total_weight;
        }
    }
    
    // 更新控制序列
    for (int j = 0; j < horizon_; ++j) {
        double weighted_v = 0.0;
        double weighted_delta = 0.0;
        
        // 使用最佳轨迹的控制作为基准
        ControlInput best_control = nominal_controls[j];
        best_control.v += noise_v_(gen_) * noise_std_v_;
        best_control.delta += noise_delta_(gen_) * noise_std_delta_;
        
        // 加权平均所有轨迹的控制
        for (int i = 0; i < num_samples_; ++i) {
            ControlInput noisy_control = nominal_controls[j];
            noisy_control.v += noise_v_(gen_) * noise_std_v_;
            noisy_control.delta += noise_delta_(gen_) * noise_std_delta_;
            
            // 增加最佳轨迹的权重
            double weight = weights[i];
            if (i == best_trajectory_idx) {
                weight *= 2.0;  // 增加最佳轨迹的影响
            }
            
            weighted_v += weight * noisy_control.v;
            weighted_delta += weight * noisy_control.delta;
        }
        
        // 平滑控制变化
        const double alpha = 0.7;  // 平滑因子
        updated_controls[j].v = alpha * best_control.v + (1-alpha) * weighted_v;
        updated_controls[j].delta = alpha * best_control.delta + (1-alpha) * weighted_delta;
        
        // 限制控制范围
        updated_controls[j].v = std::max(min_velocity_, std::min(max_velocity_, updated_controls[j].v));
        updated_controls[j].delta = std::max(-max_steering_, std::min(max_steering_, updated_controls[j].delta));
    }
    
    return updated_controls;
}

void MPPIController::publishTrajectory(const std::vector<VehicleState>& trajectory) {
    nav_msgs::Path path;
    path.header.frame_id = "map";
    path.header.stamp = ros::Time::now();
    
    for (const auto& state : trajectory) {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = state.x;
        pose.pose.position.y = state.y;
        
        tf::Quaternion q = tf::createQuaternionFromYaw(state.theta);
        tf::quaternionTFToMsg(q, pose.pose.orientation);
        
        path.poses.push_back(pose);
    }
    
    trajectory_pub_.publish(path);
}

} // namespace ackermann_navigation

int main(int argc, char** argv) {
    ros::init(argc, argv, "mppi_controller");
    ackermann_navigation::MPPIController controller;
    ros::spin();
    return 0;
}