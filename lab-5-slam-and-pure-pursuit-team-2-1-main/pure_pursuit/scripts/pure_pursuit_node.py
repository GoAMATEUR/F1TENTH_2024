#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial.transform import Rotation as R
import tf2_ros
import csv
from time import sleep

class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')
        
        self.vel = 2.0
        self.lookahead = 1.0
        self.p = 0.5

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_subscription(Odometry, '/ego_racecar/odom', self.pose_callback, 10)
        # self.create_subscription(PoseStamped, '/pf/viz/inferred_pose', self.pose_callback, 10)
        self.waypoints_publisher = self.create_publisher(MarkerArray, '/pure_pursuit/waypoints', 50)
        self.goalpoint_publisher = self.create_publisher(Marker, '/pure_pursuit/goalpoint', 5)
        self.testpoint_publisher = self.create_publisher(MarkerArray, '/pure_pursuit/testpoints', 10)
        self.future_pos_publisher = self.create_publisher(Marker, '/pure_pursuit/future_pos', 5)
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        
        # self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.waypoints = self.load_waypoints("waypoints/wp_interpolated.csv")
        self.publish_waypoints()
        

    def load_waypoints(self, path):
        waypoints = []
        with open(path, newline='') as f:
            reader = csv.reader(f)
            waypoints = list(reader)
            waypoints = [np.array([float(wp[0]), float(wp[1])]) for wp in waypoints]
        
        return waypoints

    def publish_waypoints(self):
        if len(self.waypoints) == 0:
            return
        
        markerArray = MarkerArray()
        for i, wp in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.pose.position.x = float(wp[0])
            marker.pose.position.y = float(wp[1])
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            markerArray.markers.append(marker)
        self.waypoints_publisher.publish(markerArray)

    def find_current_waypoint(self, current_pos, current_heading):
        euler_angles = current_heading.as_euler('zyx')
        future_pos = current_pos + self.lookahead * np.array([np.cos(euler_angles[0]), np.sin(euler_angles[0])])
        closest_wp = None
        min_dist = float('inf')
        for idx, wp in enumerate(self.waypoints):
            dist = np.linalg.norm(np.array(wp) - future_pos)
            if dist < min_dist:
                min_dist = dist
                closest_wp = wp
                min_idx = idx
        
        # self.publish_testpoints([closest_wp])
        dist_to_curr_pos = np.linalg.norm(closest_wp - current_pos)
        if dist_to_curr_pos <= self.lookahead:
            two_wps = [self.waypoints[min_idx]]
            if min_idx+1 < len(self.waypoints):
                two_wps.append(self.waypoints[min_idx+1])
            else:
                two_wps.append(self.waypoints[0])
        else:
            two_wps = []
            if min_idx-1 >= 0:
                two_wps.append(self.waypoints[min_idx-1])
            else:
                two_wps.append(self.waypoints[-1])
            two_wps.append(self.waypoints[min_idx])
        # print(two_wps, future_pos)
        # print(future_pos, two_wps)
        self.publish_future_pos(future_pos)
        self.publish_testpoints(two_wps)
        return self.interpolate_waypoints(two_wps, current_pos)
    

    def interpolate_waypoints(self, two_wps, curr_pos):
        # print(two_wps)
        self.publish_testpoints(two_wps)

        wp_vec = two_wps[0] - two_wps[1]
        pos_vec = two_wps[0] - curr_pos
        alpha = np.arccos(np.dot(wp_vec, pos_vec) / (np.linalg.norm(wp_vec) * np.linalg.norm(pos_vec)))
        beta = np.pi - alpha
        a = np.linalg.norm(pos_vec) * np.cos(beta)
        b = np.linalg.norm(pos_vec) * np.sin(beta)
        c = np.sqrt(self.lookahead**2 - b**2) - a
        return two_wps[0] - c * wp_vec / np.linalg.norm(wp_vec)

    def publish_future_pos(self, future_pos):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.position.x = future_pos[0]
        marker.pose.position.y = future_pos[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.25
        marker.scale.y = 0.25
        marker.scale.z = 0.25
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        self.future_pos_publisher.publish(marker)

    def publish_testpoints(self, testpoints):
        markerArray = MarkerArray()
        for i, tp in enumerate(testpoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.pose.position.x = tp[0]
            marker.pose.position.y = tp[1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.x = 0.0
            marker.pose.orientation.y = 0.0
            marker.pose.orientation.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.21
            marker.scale.y = 0.21
            marker.scale.z = 0.21
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            markerArray.markers.append(marker)
        self.testpoint_publisher.publish(markerArray)


    def publish_goalpoint(self, goalpoint):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = 0
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.position.x = goalpoint[0]
        marker.pose.position.y = goalpoint[1]
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.goalpoint_publisher.publish(marker)
    
    def pose_callback(self, pose_msg):
        # self.publish_waypoints()

        # TODO: find the current waypoint to track using methods mentioned in lecture
        # try:
        #     map_to_car_transform = self.tf_buffer.lookup_transform("map", "ego_racecar/base_link", rclpy.time.Time())
        #     # map_to_car_translation = map_to_car_transform.transform.translation
        #     # map_to_car_rotation = R.from_quat(map_to_car_transform.transform.rotation)
        #     # self.get_logger().info("transform")
        # except Exception as error:
        #     print(error)
        #     return
        # map_to_car_translation = map_to_car_transform.transform.translation
        # map_to_car_translation = np.array([map_to_car_translation.x, map_to_car_translation.y, map_to_car_translation.z])
        # map_to_car_rotation = map_to_car_transform.transform.rotation
        # map_to_car_rotation = R.from_quat([map_to_car_rotation.x, map_to_car_rotation.y, map_to_car_rotation.z, map_to_car_rotation.w])
        # # print(map_to_car_translation, map_to_car_rotation.as_matrix())
        current_pos = np.array([pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y])
        current_heading = R.from_quat(np.array([pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w]))
        # # map_pos = (car_pos - map_to_car_translation) @ map_to_car_rotation.inv().as_matrix()
        # # print(car_pos, map_to_car_translation)
        # print(car_pos, map_to_car_translation)
        # current_heading = np.array([pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w])

        # find current waypoint by projecting the car forward by lookahead distance, then finding the closest waypoint to that projected position
        # depending on the distance of the closest waypoint to current position, we will find two waypoints that sandwich the current position plus lookahead distance
        # then we interpolate between these two waypoints to find the current waypoint
        current_waypoint = self.find_current_waypoint(current_pos, current_heading)
        self.publish_goalpoint(current_waypoint)
    
        # transform the current waypoint to the vehicle frame of reference
        map_to_car_transform = self.tf_buffer.lookup_transform("map", "ego_racecar/base_link", rclpy.time.Time())
        map_to_car_translation = map_to_car_transform.transform.translation
        map_to_car_translation = np.array([map_to_car_translation.x, map_to_car_translation.y, map_to_car_translation.z])
        map_to_car_rotation = map_to_car_transform.transform.rotation
        map_to_car_rotation = R.from_quat([map_to_car_rotation.x, map_to_car_rotation.y, map_to_car_rotation.z, map_to_car_rotation.w])
        # print(current_waypoint)
        # current_waypoint = np.array(current_waypoint +)
        # print
        wp_car_frame = (np.array([current_waypoint[0], current_waypoint[1], 0]) - map_to_car_translation)
        # print(wp_car_frame)
        wp_car_frame = wp_car_frame @ map_to_car_rotation.as_matrix()
        # print(wp_car_frame)

        # TODO: calculate curvature/steering angle
        # print(wp_car_frame)
        curvature = 2 * wp_car_frame[1] / self.lookahead**2
        # print(curvature)
        # TODO: publish drive message, don't forget to limit the steering angle.
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "ego_racecar/base_link"
        drive_msg.drive.steering_angle = self.p * curvature
        drive_msg.drive.speed = self.vel
        self.drive_publisher.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
