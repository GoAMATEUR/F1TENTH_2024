#!/usr/bin/env python3
import os
import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

class LocalizationEvaluation(Node):
    """ 
    Evaluation localization Accuracy
    """
    def __init__(self):
        super().__init__('localization_evaluation_node')

        gt_pose_topic_name = self.declare_parameter('gt_pose_topic_name', 'default_value').get_parameter_value().string_value
        estimated_pose_topic_name = self.declare_parameter('estimated_pose_topic_name', 'default_value').get_parameter_value().string_value
        
        # self.create_subscription(Odometry, gt_pose_topic_name, self.gt_pose_callback, 10)
        # self.create_subscription(Odometry, estimated_pose_topic_name, self.estimated_pose_callback, 10)
        self.create_subscription(Odometry, "/ego_racecar/odom", self.gt_pose_callback, 10)
        self.create_subscription(Odometry, "/ego_racecar/odom", self.estimated_pose_callback, 10)

        self.current_gt_pose_msg = None

        # create output csv
        now = datetime.datetime.now()
        formatted_date = now.strftime("%Y%m%d%H%M%S")
        output_dir = Path(os.getcwd()) / "datas" / formatted_date
        os.makedirs(output_dir, exist_ok=True)

        self.output_file_path = output_dir / "localization_evaluation.csv"

        output_file = open(self.output_file_path, "w")
        output_file.write("timestamp,gt_x,gt_y,gt_yaw,est_x,est_y,est_yaw\n")
        output_file.close()

    def gt_pose_callback(self, pose_msg):                
        self.current_gt_pose_msg = pose_msg

        return 

    def estimated_pose_callback(self, pose_msg):
        if self.current_gt_pose_msg == None:
            return

        gt_x = self.current_gt_pose_msg.pose.pose.position.x
        gt_y = self.current_gt_pose_msg.pose.pose.position.y
        gt_yaw = self.current_gt_pose_msg.pose.pose.orientation.z

        est_x = pose_msg.pose.pose.position.x
        est_y = pose_msg.pose.pose.position.y
        est_yaw = pose_msg.pose.pose.orientation.z

        timestamp = pose_msg.header.stamp.sec + pose_msg.header.stamp.nanosec * 1e-9

        output_file = open(self.output_file_path, "a")
        output_file.write(f"{timestamp},{gt_x},{gt_y},{gt_yaw},{est_x},{est_y},{est_yaw}\n")
        output_file.close()

        return

def main(args=None):
    rclpy.init(args=args)
    print("Localization Evaluation Initialized")

    localization_eval_node = LocalizationEvaluation()

    rclpy.spin(localization_eval_node)

    localization_eval_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
