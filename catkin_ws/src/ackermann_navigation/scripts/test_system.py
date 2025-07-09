#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path, OccupancyGrid
from visualization_msgs.msg import Marker
import tf

class SystemTester:
    def __init__(self):
        rospy.init_node('system_tester', anonymous=True)
        
        # 发布器
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.obstacle_pub = rospy.Publisher('/add_obstacle', PoseStamped, queue_size=1)
        self.cmd_pub = rospy.Publisher('/manual_cmd_vel', Twist, queue_size=1)
        
        # 订阅器
        self.path_sub = rospy.Subscriber('/reference_path', Path, self.path_callback)
        self.trajectory_sub = rospy.Subscriber('/predicted_trajectory', Path, self.trajectory_callback)
        self.costmap_sub = rospy.Subscriber('/costmap', OccupancyGrid, self.costmap_callback)
        
        # 状态变量
        self.path_received = False
        self.trajectory_received = False
        self.costmap_received = False
        
        rospy.loginfo("System Tester initialized")
    
    def path_callback(self, msg):
        self.path_received = True
        rospy.loginfo(f"Received path with {len(msg.poses)} points")
    
    def trajectory_callback(self, msg):
        self.trajectory_received = True
        rospy.loginfo(f"Received predicted trajectory with {len(msg.poses)} points")
    
    def costmap_callback(self, msg):
        self.costmap_received = True
        rospy.loginfo(f"Received costmap: {msg.info.width}x{msg.info.height}")
    
    def test_goal_setting(self):
        """测试目标点设置"""
        rospy.loginfo("Testing goal setting...")
        
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = 10.0
        goal.pose.position.y = 5.0
        goal.pose.position.z = 0.0
        
        # 设置朝向
        q = tf.transformations.quaternion_from_euler(0, 0, np.pi/4)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]
        
        self.goal_pub.publish(goal)
        rospy.loginfo("Published goal at (10, 5)")
    
    def test_obstacle_adding(self):
        """测试障碍物添加"""
        rospy.loginfo("Testing obstacle adding...")
        
        obstacle = PoseStamped()
        obstacle.header.frame_id = "map"
        obstacle.header.stamp = rospy.Time.now()
        obstacle.pose.position.x = 5.0
        obstacle.pose.position.y = 3.0
        obstacle.pose.position.z = 0.0
        obstacle.pose.orientation.w = 1.0
        
        self.obstacle_pub.publish(obstacle)
        rospy.loginfo("Published obstacle at (5, 3)")
    
    def test_manual_control(self):
        """测试手动控制"""
        rospy.loginfo("Testing manual control...")
        
        cmd = Twist()
        cmd.linear.x = 2.0  # 前进速度
        cmd.angular.z = 0.5  # 转向速度
        
        self.cmd_pub.publish(cmd)
        rospy.loginfo("Published manual control command")
    
    def check_system_status(self):
        """检查系统状态"""
        rospy.loginfo("Checking system status...")
        
        status = {
            "Path Planning": self.path_received,
            "MPPI Control": self.trajectory_received,
            "Costmap": self.costmap_received
        }
        
        for component, status_bool in status.items():
            status_str = "✓" if status_bool else "✗"
            rospy.loginfo(f"{component}: {status_str}")
        
        return all(status.values())
    
    def run_tests(self):
        """运行所有测试"""
        rospy.loginfo("Starting system tests...")
        
        # 等待系统启动
        rospy.sleep(2.0)
        
        # 测试1: 检查系统状态
        if not self.check_system_status():
            rospy.logwarn("Some components not ready, waiting...")
            rospy.sleep(3.0)
            if not self.check_system_status():
                rospy.logerr("System not ready for testing")
                return
        
        # 测试2: 添加障碍物
        self.test_obstacle_adding()
        rospy.sleep(1.0)
        
        # 测试3: 设置目标点
        self.test_goal_setting()
        rospy.sleep(1.0)
        
        # 测试4: 手动控制
        self.test_manual_control()
        rospy.sleep(1.0)
        
        rospy.loginfo("All tests completed!")
    
    def run_continuous_test(self):
        """运行连续测试"""
        rospy.loginfo("Starting continuous test...")
        
        rate = rospy.Rate(1)  # 1Hz
        test_count = 0
        
        while not rospy.is_shutdown():
            test_count += 1
            rospy.loginfo(f"Continuous test iteration {test_count}")
            
            # 定期检查系统状态
            if test_count % 10 == 0:
                self.check_system_status()
            
            # 定期添加随机目标点
            if test_count % 20 == 0:
                goal = PoseStamped()
                goal.header.frame_id = "map"
                goal.header.stamp = rospy.Time.now()
                goal.pose.position.x = np.random.uniform(-10, 10)
                goal.pose.position.y = np.random.uniform(-10, 10)
                goal.pose.position.z = 0.0
                goal.pose.orientation.w = 1.0
                self.goal_pub.publish(goal)
                rospy.loginfo(f"Published random goal at ({goal.pose.position.x:.1f}, {goal.pose.position.y:.1f})")
            
            rate.sleep()

def main():
    tester = SystemTester()
    
    try:
        # 运行初始测试
        tester.run_tests()
        
        # 询问是否运行连续测试
        rospy.loginfo("Press Enter to start continuous testing (or Ctrl+C to exit)...")
        input()
        
        # 运行连续测试
        tester.run_continuous_test()
        
    except KeyboardInterrupt:
        rospy.loginfo("Test interrupted by user")
    except Exception as e:
        rospy.logerr(f"Test failed with error: {e}")

if __name__ == '__main__':
    main() 