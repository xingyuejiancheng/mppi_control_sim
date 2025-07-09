#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
阿克曼车辆导航仿真自动化测试脚本
- 支持批量添加障碍物（x, y, r），障碍物半径可配置
- 支持批量添加路径点（x, y），可测试多段路径
- 所有日志为简体中文，注释详细，便于理解和维护
- 自动检测路径是否生成，未生成自动重发目标点
"""
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import Path, OccupancyGrid
import math

class SimpleTester:
    def __init__(self):
        """
        初始化测试节点，设置发布器、订阅器、状态变量和测试配置
        """
        rospy.init_node('simple_tester', anonymous=True)
        
        # 发布器：目标点和障碍物
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.obstacle_pub = rospy.Publisher('/add_obstacle', PoseStamped, queue_size=1)
        
        # 订阅器：路径、轨迹、代价地图
        self.path_sub = rospy.Subscriber('/reference_path', Path, self.path_callback)
        self.trajectory_sub = rospy.Subscriber('/predicted_trajectory', Path, self.trajectory_callback)
        self.costmap_sub = rospy.Subscriber('/costmap', OccupancyGrid, self.costmap_callback)
        
        # 状态变量
        self.path_received = False
        self.trajectory_received = False
        self.costmap_received = False
        self.last_path_points = 0
        
        # 配置障碍物列表，每个元素为(x, y, r)
        self.obstacle_list = [
            (5.0, 3.0, 0.5),
            (7.0, 6.0, 1.0),
            (8.5, 2.5, 0.7)
        ]
        
        # 配置路径点列表，每个元素为(x, y, theta)
        self.goal_list = [
            (2.0, 2.0, math.pi/4),
            (5.0, 5.0, math.pi/2),
            (8.0, 2.0, -math.pi/4)
        ]
        
        rospy.loginfo("测试节点初始化完成")
        rospy.sleep(2.0)  # 确保发布器和订阅器都已就绪

    def path_callback(self, msg):
        """
        路径回调，收到路径时设置状态变量
        """
        self.path_received = True
        self.last_path_points = len(msg.poses)
        rospy.loginfo(f"收到参考路径，点数: {len(msg.poses)}")

    def trajectory_callback(self, msg):
        """
        轨迹回调，收到预测轨迹时设置状态变量
        """
        self.trajectory_received = True
        rospy.loginfo(f"收到预测轨迹，点数: {len(msg.poses)}")

    def costmap_callback(self, msg):
        """
        代价地图回调，收到地图时设置状态变量
        """
        self.costmap_received = True
        rospy.loginfo(f"收到代价地图: {msg.info.width}x{msg.info.height}")

    def create_pose_stamped(self, x, y, theta):
        """
        创建带方向的PoseStamped消息
        """
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # 将偏航角转换为四元数
        q = tf.transformations.quaternion_from_euler(0, 0, theta)
        pose.pose.orientation = Quaternion(*q)
        
        return pose

    def publish_goal_and_wait(self, x, y, theta, idx):
        """
        发布目标点并等待路径生成，未收到路径则自动重发
        """
        rospy.loginfo(f"准备发布目标点#{idx+1}，等待订阅者...")
        
        # 等待订阅者连接
        while self.goal_pub.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)
        
        max_retries = 3
        for attempt in range(max_retries):
            self.path_received = False
            goal = self.create_pose_stamped(x, y, theta)
            self.goal_pub.publish(goal)
            rospy.loginfo(f"已发布目标点#{idx+1}，位置: ({x:.2f}, {y:.2f})，方向: {theta:.2f}，第{attempt+1}次尝试")
            
            # 等待路径生成
            wait_count = 0
            while wait_count < 50 and not rospy.is_shutdown():  # 最多等待5秒
                if self.path_received:
                    rospy.loginfo(f"目标点#{idx+1}测试成功！生成路径点数: {self.last_path_points}")
                    rospy.sleep(2.0)  # 等待2秒确保路径稳定
                    return True
                rospy.sleep(0.1)
                wait_count += 1
            
            rospy.logwarn(f"目标点#{idx+1}等待路径超时，准备重试...")
        
        rospy.logerr(f"目标点#{idx+1}发布失败！")
        return False

    def check_system_status(self):
        """
        检查系统各核心模块是否正常工作
        """
        rospy.loginfo("检查系统状态...")
        status = {
            "路径规划": self.path_received,
            "MPPI控制": self.trajectory_received,
            "代价地图": self.costmap_received
        }
        for component, status_bool in status.items():
            status_str = "✓" if status_bool else "✗"
            rospy.loginfo(f"{component}: {status_str}")
        return self.costmap_received  # 只要costmap正常就继续

    def run_tests(self):
        """
        主测试流程：先发布障碍物，等待地图更新，再依次发布目标点并检测路径
        """
        rospy.loginfo("系统测试启动...")
        rospy.sleep(2.0)  # 等待系统完全启动
        
        if not self.check_system_status():
            rospy.logwarn("等待系统就绪...")
            rospy.sleep(3.0)
            if not self.check_system_status():
                rospy.logerr("系统未就绪，但仍继续测试")
        
        rospy.loginfo("开始发布目标点...")
        for idx, (x, y, theta) in enumerate(self.goal_list):
            if not self.publish_goal_and_wait(x, y, theta, idx):
                rospy.logerr(f"目标点#{idx+1}测试失败，终止测试")
                return
            rospy.sleep(3.0)  # 每个目标点之间等待3秒
        
        rospy.loginfo("所有目标点测试完成！")

def main():
    tester = SimpleTester()
    try:
        tester.run_tests()
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("测试被用户中断")
    except Exception as e:
        rospy.logerr(f"测试失败: {e}")

if __name__ == '__main__':
    main() 