#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

// =============================
// 车辆虚拟里程计发布器
// 支持仿真车辆位置与姿态，便于RViz可视化
// =============================
class FakeOdomPublisher {
public:
    FakeOdomPublisher() : nh_("~") {
        // 加载参数：发布频率，默认50Hz
        nh_.param("publish_rate", publish_rate_, 50.0);
        
        // 设置里程计发布器
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/odom", 1);
        // 订阅速度指令
        cmd_sub_ = nh_.subscribe("/ackermann/cmd_vel", 1, &FakeOdomPublisher::cmdCallback, this);
        
        // 初始化车辆状态
        x_ = 0.0;   // 位置x
        y_ = 0.0;   // 位置y
        theta_ = 0.0; // 航向角
        v_ = 0.0;     // 线速度
        omega_ = 0.0; // 角速度
        
        // 定时器：定时发布里程计和TF
        timer_ = nh_.createTimer(ros::Duration(1.0/publish_rate_), &FakeOdomPublisher::timerCallback, this);
        
        ROS_INFO("虚拟里程计发布器初始化，发布/odom和TF，支持RViz可视化");
    }
    
    // 速度指令回调，更新车辆速度
    void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        v_ = msg->linear.x;
        omega_ = msg->angular.z;
    }
    
    // 定时回调，更新车辆状态并发布里程计与TF
    void timerCallback(const ros::TimerEvent& event) {
        // 计算时间步长
        double dt = 1.0 / publish_rate_;
        // 基于阿克曼运动学更新位置
        x_ += v_ * cos(theta_) * dt;
        y_ += v_ * sin(theta_) * dt;
        theta_ += omega_ * dt;
        
        // 构造并发布nav_msgs/Odometry消息
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "odom";         // 世界坐标系
        odom.child_frame_id = "base_link";     // 车辆坐标系
        
        // 设置位置
        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        
        // 设置朝向（四元数）
        tf::Quaternion q = tf::createQuaternionFromYaw(theta_);
        tf::quaternionTFToMsg(q, odom.pose.pose.orientation);
        
        // 设置速度
        odom.twist.twist.linear.x = v_;
        odom.twist.twist.linear.y = 0.0;
        odom.twist.twist.angular.z = omega_;
        
        // 发布里程计消息
        odom_pub_.publish(odom);
        
        // 发布TF变换，便于RViz显示车辆实时位置
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(x_, y_, 0.0));
        transform.setRotation(q);
        tf_broadcaster_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));
    }
    
private:
    ros::NodeHandle nh_;                // ROS节点句柄
    ros::Publisher odom_pub_;           // 里程计发布器
    ros::Subscriber cmd_sub_;           // 速度指令订阅器
    ros::Timer timer_;                  // 定时器
    tf::TransformBroadcaster tf_broadcaster_; // TF变换发布器
    
    double publish_rate_;               // 发布频率
    double x_, y_, theta_;              // 车辆状态：位置与航向
    double v_, omega_;                  // 车辆速度
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "fake_odom_publisher");
    FakeOdomPublisher publisher;
    ros::spin();
    return 0;
} 