#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image
import math
import time
import os

# 需要cv_bridge和OpenCV来处理图像
# sudo apt-get install ros-noetic-cv-bridge
# pip install opencv-python
try:
    from cv_bridge import CvBridge, CvBridgeError
    import cv2
except ImportError as e:
    rospy.logerr("Failed to import cv_bridge or cv2. Please install them.")
    rospy.logerr("For ROS, try: sudo apt-get install ros-<distro>-cv-bridge")
    rospy.logerr("For OpenCV, try: pip install opencv-python")
    exit(1)

class PointCloudFilterAndCapture:
    def __init__(self):
        rospy.init_node('pointcloud_filter_node', anonymous=True)

        # --- 保留的原始参数 ---
        input_topic = rospy.get_param('~input_topic', '/livox/lidar')
        output_topic = rospy.get_param('~output_topic', '/filtered_points')

        self.x_min = rospy.get_param('~x_min', 0.0)
        self.x_max = rospy.get_param('~x_max', 10.0)
        self.z_min = rospy.get_param('~z_min', -1.0)
        self.z_max = rospy.get_param('~z_max', 1.0)
        self.angle_min_deg = rospy.get_param('~angle_min', -45.0)
        self.angle_max_deg = rospy.get_param('~angle_max', 45.0)

        # --- 新增的捕获功能参数 ---
        self.enable_capture = rospy.get_param('~enable_capture', False)
        self.collection_duration = rospy.get_param('~collection_duration', 5.0)  # N seconds
        self.image_topic = rospy.get_param('~image_topic', '/camera/image_raw')
        self.output_dir = rospy.get_param('~output_dir', '.')

        # --- 打印参数信息 ---
        rospy.loginfo("--- PointCloud Filter and Capture Node ---")
        rospy.loginfo("Input topic: %s", input_topic)
        rospy.loginfo("Output topic: %s", output_topic)
        rospy.loginfo("Filtering parameters:")
        rospy.loginfo("  X-axis range: [%.2f, %.2f] m", self.x_min, self.x_max)
        rospy.loginfo("  Z-axis range: [%.2f, %.2f] m", self.z_min, self.z_max)
        rospy.loginfo("  Angle range: [%.2f, %.2f] degrees", self.angle_min_deg, self.angle_max_deg)
        rospy.loginfo("Capture mode enabled: %s", self.enable_capture)
        if self.enable_capture:
            rospy.loginfo("  Collection duration: %.2f seconds", self.collection_duration)
            rospy.loginfo("  Image topic: %s", self.image_topic)
            rospy.loginfo("  Output directory: %s", os.path.abspath(self.output_dir))
        
        # --- 初始化发布者和订阅者 ---
        self.publisher = rospy.Publisher(output_topic, PointCloud2, queue_size=10)
        self.pc_subscriber = rospy.Subscriber(input_topic, PointCloud2, self.pointcloud_callback)
        
        # --- 初始化捕获功能所需变量 ---
        if self.enable_capture:
            self.accumulated_points = []
            self.is_collecting = False
            self.image_saved = False
            self.start_time = None
            self.cv_bridge = CvBridge()
            self.image_subscriber = None # 将在捕获开始时初始化

    def pointcloud_callback(self, ros_point_cloud):
        filtered_points = []
        point_generator = pc2.read_points(ros_point_cloud, field_names=("x", "y", "z"), skip_nans=True)

        for p in point_generator:
            x, y, z = p[0], p[1], p[2]

            # 1. X轴 - 纵深
            if not (self.x_min <= x <= self.x_max):
                continue
            
            # 2. Z轴 - 高度
            if not (self.z_min <= z <= self.z_max):
                continue

            # 3. Y轴 - 水平角
            angle = math.degrees(math.atan2(y, x))
            if not (self.angle_min_deg <= angle <= self.angle_max_deg):
                continue
            
            filtered_points.append([x, y, z])

        # --- 保留原始功能：发布过滤后的点云 ---
        if len(filtered_points) > 0:
            header = ros_point_cloud.header
            filtered_cloud_msg = pc2.create_cloud_xyz32(header, filtered_points)
            self.publisher.publish(filtered_cloud_msg)

        # --- 新增功能：如果处于收集中，则累积点云 ---
        if self.enable_capture and self.is_collecting:
            self.accumulated_points.extend(filtered_points)

    def image_callback(self, ros_image):
        # 仅在采集周期过半且尚未保存图像时执行
        midpoint_time = self.start_time + rospy.Duration(self.collection_duration / 2.0)
        if not self.image_saved and rospy.Time.now() >= midpoint_time:
            rospy.loginfo("Midpoint time reached. Attempting to save image...")
            try:
                # 将ROS图像消息转换为OpenCV图像格式（bgr8是标准的彩色格式）
                cv_image = self.cv_bridge.imgmsg_to_cv2(ros_image, "bgr8")
                
                # 创建带时间戳的文件名
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                filename = os.path.join(self.output_dir, f"capture_{timestamp}.png")
                
                # 保存图像并标记为已保存
                cv2.imwrite(filename, cv_image)
                self.image_saved = True
                rospy.loginfo(f"Successfully saved image to {filename}")

                # 任务完成，取消订阅以节省资源
                if self.image_subscriber:
                    self.image_subscriber.unregister()

            except CvBridgeError as e:
                rospy.logerr(f"CvBridge Error: {e}")
            except Exception as e:
                rospy.logerr(f"Failed to save image: {e}")

    def save_pcd_file(self):
        if not self.accumulated_points:
            rospy.logwarn("No points were accumulated. PCD file will not be created.")
            return

        rospy.loginfo(f"Saving {len(self.accumulated_points)} points to PCD file...")
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(self.output_dir, f"capture_{timestamp}.pcd")

        try:
            with open(filename, 'w') as f:
                num_points = len(self.accumulated_points)
                # 编写PCD文件头
                f.write("# .PCD v0.7 - Point Cloud Data file format\n")
                f.write("VERSION 0.7\n")
                f.write("FIELDS x y z\n")
                f.write("SIZE 4 4 4\n")
                f.write("TYPE F F F\n")
                f.write("COUNT 1 1 1\n")
                f.write(f"WIDTH {num_points}\n")
                f.write("HEIGHT 1\n")
                f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
                f.write(f"POINTS {num_points}\n")
                f.write("DATA ascii\n")
                
                # 写入点数据
                for p in self.accumulated_points:
                    f.write(f"{p[0]:.4f} {p[1]:.4f} {p[2]:.4f}\n")
            
            rospy.loginfo(f"Successfully saved PCD file to {filename}")
        except Exception as e:
            rospy.logerr(f"Failed to write PCD file: {e}")

    def run_capture_session(self):
        """执行一次性的捕获任务"""
        rospy.loginfo(f"Starting {self.collection_duration}-second capture session...")
        
        # 确保输出目录存在
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            rospy.loginfo(f"Created output directory: {self.output_dir}")

        # 开始订阅图像话题
        self.image_subscriber = rospy.Subscriber(self.image_topic, Image, self.image_callback)
        
        # 启动点云收集
        self.start_time = rospy.Time.now()
        self.is_collecting = True
        
        # 等待指定的收集时间
        rospy.sleep(self.collection_duration)
        
        # 停止收集并注销订阅者，防止后续数据影响
        self.is_collecting = False
        self.pc_subscriber.unregister()
        if self.image_subscriber:
            self.image_subscriber.unregister()
        rospy.loginfo("Point cloud and image collection finished.")
        
        if not self.image_saved:
            rospy.logwarn("Image was not saved during the capture session. Check if the image topic is publishing.")
            
        # 保存累积的点云到PCD文件
        self.save_pcd_file()
        
        rospy.loginfo("Capture session complete. Shutting down.")
        rospy.signal_shutdown("Capture session complete.")

    def run(self):
        if self.enable_capture:
            self.run_capture_session()
        else:
            rospy.loginfo("Running in continuous filtering mode. (Original functionality)")
            rospy.spin()

if __name__ == '__main__':
    try:
        node = PointCloudFilterAndCapture()
        node.run()
    except rospy.ROSInterruptException:
        pass