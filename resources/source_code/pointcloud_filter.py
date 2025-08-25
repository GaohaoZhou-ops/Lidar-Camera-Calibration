#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
import math

class PointCloudFilter:
    def __init__(self):
        rospy.init_node('pointcloud_filter_node', anonymous=True)

        input_topic = rospy.get_param('~input_topic', '/livox/lidar')
        output_topic = rospy.get_param('~output_topic', '/filtered_points')

        self.x_min = rospy.get_param('~x_min', 0.0)
        self.x_max = rospy.get_param('~x_max', 10.0)
        self.z_min = rospy.get_param('~z_min', -1.0)
        self.z_max = rospy.get_param('~z_max', 1.0)
        self.angle_min_deg = rospy.get_param('~angle_min', -45.0)
        self.angle_max_deg = rospy.get_param('~angle_max', 45.0)

        rospy.loginfo("Input topic: %s", input_topic)
        rospy.loginfo("Output topic: %s", output_topic)
        rospy.loginfo("Filtering parameters:")
        rospy.loginfo("  X-axis range: [%.2f, %.2f] m", self.x_min, self.x_max)
        rospy.loginfo("  Z-axis range: [%.2f, %.2f] m", self.z_min, self.z_max)
        rospy.loginfo("  Angle range: [%.2f, %.2f] degrees", self.angle_min_deg, self.angle_max_deg)
        
        self.publisher = rospy.Publisher(output_topic, PointCloud2, queue_size=10)
        self.subscriber = rospy.Subscriber(input_topic, PointCloud2, self.pointcloud_callback)

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

        if len(filtered_points) > 0:
            header = ros_point_cloud.header
            filtered_cloud_msg = pc2.create_cloud_xyz32(header, filtered_points)
            self.publisher.publish(filtered_cloud_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        pc_filter = PointCloudFilter()
        pc_filter.run()
    except rospy.ROSInterruptException:
        pass