#include "rclcpp/rclcpp.hpp"
#include <string>
#include <cmath>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        // TODO: create ROS subscribers and publishers
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic, 10, std::bind(&WallFollow::scan_callback, this, std::placeholders::_1));
        // odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        //     odom_topic, 10, std::bind(&WallFollow::odom_callback, this, std::placeholders::_1));
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);
        RCLCPP_INFO(this->get_logger(), "Wall Follow Node has been initialized");
    }

private:
    // PID CONTROL PARAMS
    const double kp = 2.5;
    const double kd = 0.75;
    const double ki = 0.0001;

    const double theta = M_PI / 36 * 7.5;

    const double LOOKAHEAD = 0.4;

    double DESIRED_DISTANCE = 0.7;

    double servo_offset = 0.0;
    

    double prev_error = 0.0;
    double error = 0.0;
    double integral = 0.0;

    double lidar_angle_min = NULL;
    double lidar_angle_max = NULL;
    double lidar_angle_increment = NULL;
    // Topics
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    // std::string odom_topic = "/odom";

    /// TODO: create ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    // rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    double get_range(std::vector<float> range_data, double angle)
    {
        /*
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle
        */
        int idx = (angle - lidar_angle_min) / lidar_angle_increment;
        return range_data[idx];
    }

    double get_error(std::vector<float> range_data, double dist)
    {
        /*
        Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

        Args:
            range_data: single range array from the LiDAR
            dist: desired distance to the wall

        Returns:
            error: calculated error
        */

        double b = get_range(range_data, M_PI/4);
        double a = get_range(range_data, M_PI/4 - theta);
    
        double alpha = atan((a * cos(theta) - b) / (a * sin(theta)));
        double curr_dist = b * cos(alpha);
        double future_dist = curr_dist + LOOKAHEAD * sin(alpha);

        return dist - future_dist;
    }

    void pid_control(double error)
    {
        /*
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        */
        double angle = 0.0;
        // TODO: Use kp, ki & kd to implement a PID controller

        double derivative = error - prev_error;
        prev_error = error;
        integral += error;
        
        angle = -(kp * error + ki * integral + kd * derivative);

        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = angle;
        // drive_msg.drive.speed = 0.5;
        if (angle > -M_PI/36 or angle < M_PI/36)
        {
            drive_msg.drive.speed = 1.5;
        }
        else if ((angle > -M_PI/18 and angle < -M_PI/36) or (angle < M_PI/18 and angle > M_PI/36))
        {
            drive_msg.drive.speed = 1.0;
        }
        else
        {
            drive_msg.drive.speed = 0.5;
        }
        // RCLCPP_INFO(this->get_logger(), "angle: %f", angle);
        drive_pub_->publish(drive_msg);
    }

    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        /*
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        */
        if (lidar_angle_min == NULL)
        {
            lidar_angle_min = scan_msg->angle_min;
            lidar_angle_max = scan_msg->angle_max;
            lidar_angle_increment = scan_msg->angle_increment;
        } 
        double error = get_error(scan_msg->ranges, DESIRED_DISTANCE); // TODO: replace with error calculated by get_error()
        RCLCPP_INFO(this->get_logger(), "error: %f", error);
        pid_control(error);
        // double velocity = 0.0; // TODO: calculate desired car velocity based on error
        // TODO: actuate the car with PID

    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}