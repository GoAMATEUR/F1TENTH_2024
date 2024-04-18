#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.time import Time
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker, MarkerArray
from scipy.spatial.transform import Rotation as R
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
import csv
from time import sleep


class PurePursuit(Node):
    """ 
    Implement Pure Pursuit on the car
    This is just a template, you are free to implement your own node!
    """
    def __init__(self):
        super().__init__('pure_pursuit_node')

        self.lookahead = 1.0

        
        # self.create_subscription(Odometry, '/ego_racecar/odom', self.pose_callback, 10)
        self.create_subscription(Odometry, '/pf/pose/odom', self.pose_callback, 10)
        self.waypoints_publisher = self.create_publisher(MarkerArray, '/pure_pursuit/waypoints', QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL, reliability=QoSReliabilityPolicy.RELIABLE))
        self.goalpoint_publisher = self.create_publisher(Marker, '/pure_pursuit/goalpoint', QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.testpoint_publisher = self.create_publisher(MarkerArray, '/pure_pursuit/testpoints', QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.future_pos_publisher = self.create_publisher(Marker, '/pure_pursuit/future_pos', QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.drive_publisher = self.create_publisher(AckermannDriveStamped, '/drive', 10)
        
        self.map_to_car_rotation = None
        self.map_to_car_translation = None

        waypoints = self.load_waypoints("race2/waypoints/race1_gtl4_seg.csv")
        self.waypoints = waypoints[:, :2] # x, y
        self.params = waypoints[:, 2:] #  v, vel percent, look_ahead, p, d, index
        
        ## Scale global speed
        # velocities = self.params[:, 0].copy()
        # gloabl_v_min = velocities.min()
        # global_v_max = velocities.max()
        # set_v_min = 2.2
        # set_v_max = 5.5
        # self.params[:, 0] = (velocities - gloabl_v_min) / (global_v_max - gloabl_v_min) * (set_v_max - set_v_min) + set_v_min
        
        
        self.publish_waypoints()
        self.last_curve = 0.0
        self.vis = True
        

    def load_waypoints(self, path):
        waypoints = np.loadtxt(path, delimiter=',')
        
        return waypoints

    

    def find_current_waypoint(self, current_pos, current_heading):
        euler_angles = current_heading.as_euler('zyx')
        future_pos = current_pos + self.lookahead * np.array([np.cos(euler_angles[0]), np.sin(euler_angles[0])])
        dist = np.linalg.norm(self.waypoints - future_pos, axis=1)
        min_idx = np.argmin(dist)
        closest_wp = self.waypoints[min_idx]
        # closest_wp = None
        # min_dist = float('inf')
        # for idx, wp in enumerate(self.waypoints):
        #     dist = np.linalg.norm(np.array(wp) - future_pos)
        #     if dist < min_dist:
        #         min_dist = dist
        #         closest_wp = wp
        #         min_idx = idx
        
        # self.publish_testpoints([closest_wp])
        dist_to_curr_pos = dist[min_idx]#np.linalg.norm(closest_wp - current_pos)
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
        
        ### find waypoint closest to current position and use it for params
        local_dist = np.linalg.norm(self.waypoints - current_pos, axis=1)
        min_local_idx = np.argmin(local_dist)
        
        return self.interpolate_waypoints(two_wps, current_pos), self.params[min_local_idx]
    

    def interpolate_waypoints(self, two_wps, curr_pos):
        # print(two_wps)
        self.publish_testpoints(two_wps)

        wp_vec = two_wps[0] - two_wps[1]
        pos_vec = two_wps[0] - curr_pos
        alpha = np.arccos(np.dot(wp_vec, pos_vec) / (np.linalg.norm(wp_vec) * np.linalg.norm(pos_vec)))
        beta = np.pi - alpha
        a = np.linalg.norm(pos_vec) * np.cos(beta)
        b = np.linalg.norm(pos_vec) * np.sin(beta)
        if self.lookahead**2 - b**2 > 0:
            c = np.sqrt(self.lookahead**2 - b**2) - a
            return two_wps[0] - c * wp_vec / np.linalg.norm(wp_vec)
        else:
            return two_wps[1]

    
    
    def pose_callback(self, pose_msg):
        # t0 = Time.from_msg(pose_msg.header.stamp)
        current_pos = np.array([pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y])
        current_heading = R.from_quat(np.array([pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w]))
        
        
        # find current waypoint by projecting the car forward by lookahead distance, then finding the closest waypoint to that projected position
        # depending on the distance of the closest waypoint to current position, we will find two waypoints that sandwich the current position plus lookahead distance
        # then we interpolate between these two waypoints to find the current waypoint
        current_waypoint, current_params = self.find_current_waypoint(current_pos, current_heading)
        self.publish_goalpoint(current_waypoint)
    
        # transform the current waypoint to the vehicle frame of reference
        self.map_to_car_translation = np.array([pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y, pose_msg.pose.pose.position.z])
        self.map_to_car_rotation = R.from_quat([pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w])

        
        wp_car_frame = (np.array([current_waypoint[0], current_waypoint[1], 0]) - self.map_to_car_translation)
        wp_car_frame = wp_car_frame @ self.map_to_car_rotation.as_matrix()

        self.lookahead = current_params[2] 
        curvature = 2 * wp_car_frame[1] / self.lookahead**2
        
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "ego_racecar/base_link"
        drive_msg.drive.steering_angle = current_params[3] * curvature + current_params[4] * (self.last_curve - curvature)
        pf_speed = np.linalg.norm(np.array([pose_msg.twist.twist.linear.x, pose_msg.twist.twist.linear.y]))
        drive_msg.drive.speed = self.interpolate_vel(pf_speed, current_params[0] * current_params[1])
        self.get_logger().info("pf speed: {} seg speed: {} command: {}".format(pf_speed, current_params[0] * current_params[1], drive_msg.drive.speed))
        self.drive_publisher.publish(drive_msg)
        # t1 = Time.from_msg(drive_msg.header.stamp)
        # self.get_logger().info("Time taken: {}".format((t1 - t0).nanoseconds / 1e9))

    def interpolate_vel(self, current_vel, seg_vel):
        """
        param:
            current_vel : current velocity given by the particle filter
            seg_vel : velocity in current segment
        returns:
            command_vel : interpolated velocity
        """
        # return seg_vel
        
        timestep = 1.0
        
        if current_vel < seg_vel: # if we are accelerating
            acc = max(0.25, 0.25 * current_vel**2)
            command_vel = current_vel + acc * timestep
            command_vel = min(command_vel, seg_vel)
        else: # decelrating
            acc = max(0.2, 0.1 * current_vel**2)
            command_vel = current_vel - acc * timestep
            command_vel = max(command_vel, seg_vel)
        return command_vel

    ### Visulization functions
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

    def publish_future_pos(self, future_pos):
        if not self.vis:
            return
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
        if not self.vis:
            return
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
        if not self.vis:
            return
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
    

def main(args=None):
    rclpy.init(args=args)
    print("PurePursuit Initialized")
    pure_pursuit_node = PurePursuit()
    rclpy.spin(pure_pursuit_node)

    pure_pursuit_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
