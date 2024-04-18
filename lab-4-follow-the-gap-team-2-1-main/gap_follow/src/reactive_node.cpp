#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"


// vehicle parameters, for bubble radius.
const float CHASSIE_WIDTH = 0.2032;
const float TIRE_WIDTH = 0.0381;
const float CAR_WIDTH = CHASSIE_WIDTH + 2 * TIRE_WIDTH;
const float MAX_STEER = 0.36;


class ReactiveFollowGap : public rclcpp::Node {

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        // 1. LiDAR subscriber
        subscriber_scan = this -> create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1)
        );

        // 2. Car actuator publisher
        publisher_ = this -> create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "drive", 10
        );
        
        // 3. Declare parameters
        // param for preprocessing
        this->declare_parameter("reject_thres", 2.5);           // preprocess depth 
        this->declare_parameter("window_radius", 2);            // avg over window 
        // param for finding max gap
        this->declare_parameter("consecutive_hits", 5);         // # of consecutive hits at depth > distance_thres count as a valid gap.
        this->declare_parameter("distance_thres", 2.0);         // threshold depth of valid gap
        // param for bubble
        this->declare_parameter("bubble_radius_factor", 0.7);   // bubble radius
        this->declare_parameter("disparities_thres", 0.2);      // disparity extension threshold for depth difference.
        // param for steering PD control
        this->declare_parameter("k_p", 1.0);                    // Steering Gains
        this->declare_parameter("k_d", 0.1);
        this->declare_parameter("k_i", 0.0001);
        this->declare_parameter("alpha", 0.4);                  // steering smoothing
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_scan;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;

    // laser scan params
    int range_length;       // length of ranges
    float range_min;        // valid range
    float range_max;
    float angle_min;        // angle range
    float angle_max;
    float angle_increment;
    int front_idx_start;    // idx of scans in -90~90 degrees ahead
    int front_idx_end;

    // PD controller history
    float integral      = 0.0;
    float prev_angle    = 0.0;
    float prev_steer    = 0.0; 
    
    int angle_to_index(float angle) {
        // Helper function, find index in ranges given angle.
        if (angle < angle_min) {
            return 0;
        }
        else {
            return (int)round((std::min(angle, angle_max) - angle_min) / angle_increment);
        }

    }

    float index_to_angle(int index) {
        // Helper function, find angle given ray index.
        float angle = angle_min + index * angle_increment;
        return angle;
    }

    void preprocess_lidar(const float* ranges, float* processed_ranges)
    {   
        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)
        int window_radius = this->get_parameter("window_radius").as_int();
        float reject_thres = this->get_parameter("reject_thres").as_double();
        for (int i=0; i < range_length; i++) {
            // Find avg in window
            float avg = 0.0;
            float count = 0;
            for (int j=-window_radius; j<=window_radius; j++) {
                if (i+j>=0 && i+j<range_length) {
                    avg += ranges[i+j]; 
                    count++;
                }
            }
            avg = avg / count;
            if (avg > reject_thres) {
                processed_ranges[i] = reject_thres;
            }
            else {
                processed_ranges[i] = avg;
            }
        }
        return;
    }

    void find_max_gap(const float* ranges, int* indices)
    {   
        // Return the start index & end index of the max gap in free_space_ranges
        int consecutive_hits = this->get_parameter("consecutive_hits").as_int();
        float distance_thres = this->get_parameter("distance_thres").as_double();
        int gap_counter     = 0; //count the # of consective gaps
        int gap_max_size    = 0; //save the max-gap
        int gap_start_idx   = 0; //save the start index
        for (int i=0; i<range_length; i++){
            if (ranges[i] >= distance_thres){
                gap_counter += 1;
                if (gap_counter == 1) {
                    gap_start_idx = i;
                }
                else {
                    if (gap_counter >= consecutive_hits && gap_counter > gap_max_size){
                        gap_max_size = gap_counter;
                        indices[0] = gap_start_idx;
                        indices[1] = i;
                    }
                }
            }
            else {
                gap_counter = 0;
            }
        }
        // RCLCPP_INFO(this->get_logger(), "gap_max: %f", gap_max);
        return;
    }

    int find_best_point(float* ranges, const int* gap_indices) {   
        // TODO: If ranges has consecutive max values, use the one in the middle.
        // Start_i & end_i are start and end indicies of max-gap range, respectively
        // Return index of best point in ranges
	    // Naive: Choose the furthest point within ranges and go there
        int index;
        float farthest_distance = ranges[gap_indices[0]];
        int consecutive_count = 1;
        int fartheset_index_start = gap_indices[0];
        // *indice = gap_start_idx;
        for (int i = gap_indices[0] + 1; i <= gap_indices[1]; i++) {
            if (ranges[i] > farthest_distance) {
                farthest_distance = ranges[i];
                fartheset_index_start = i;
                consecutive_count = 1;
            } 
            else if (ranges[i] == farthest_distance) {
                consecutive_count++;
            }
        }
        index = fartheset_index_start + consecutive_count / 2;
        return index;
    }

    // void find_disparities(const float* ranges, std::vector<int>& bubble_indices) {
        
    //     return;
    // }

    std::vector<std::pair<int, int>> find_bubbles(const float* ranges) {
        // Find center of bubbles
        float disparities_thres = this->get_parameter("disparities_thres").as_double();
        std::vector<std::pair<int, int>> bubbles_indices; // (index, filling pattern)
        float closest_distance = 1000.0;
        int closest_idx;
        for (int i=0; i < range_length; i++) {
            if (ranges[i] < closest_distance) {
                closest_idx = i;
                closest_distance = ranges[i];
            }
            if (ranges[i] - ranges[i-1] > disparities_thres) {
                if (i > 0) {
                    bubbles_indices.push_back(std::make_pair(i-1, 1));
                }
            }
            else if (ranges[i]-ranges[i-1]<-disparities_thres) {
                bubbles_indices.push_back(std::make_pair(i, -1));
            }
            
        }
        bubbles_indices.push_back(std::make_pair(closest_idx, 0));
        return bubbles_indices;
    }

    void create_bubbles(float* ranges, std::vector<std::pair<int, int>>& bubble_indices) {
        // create bubbles given a set of bubble centers.
        // scans that fall into the bubble will become 0.
        float bubble_radius_factor = this->get_parameter("bubble_radius_factor").as_double();
        float bubble_radius = bubble_radius_factor * CAR_WIDTH;
        float center_angle;
        float center_distance;
        float fill_value;
        int center_index;
        int direction_flag;
        float theta; // angle between center line & tangent line of the bubble
        int bubble_index_start;
        int bubble_index_end;
        std::vector<std::tuple<int, int, float>> zero_indices;
        for (const std::pair<int, int>& p : bubble_indices) {
            center_index = p.first;
            direction_flag = p.second;
            center_angle = index_to_angle(center_index);
            center_distance = ranges[center_index];
            if (center_distance > bubble_radius) {
                theta = abs(asin(bubble_radius / center_distance));
            }
            else {
                theta = M_1_PI / 12;
                std::cout << center_index << "INSIDE BUBBLE "  << center_distance << std::endl;
            }
            if (direction_flag == 1) {
                bubble_index_start = center_index;
                bubble_index_end =   angle_to_index(center_angle+theta);
            } 
            else if (direction_flag == -1) {
                bubble_index_start = angle_to_index(center_angle-theta);
                bubble_index_end =   center_index;
            } 
            else {
                bubble_index_start = angle_to_index(center_angle-theta);
                bubble_index_end =   angle_to_index(center_angle+theta);
            }            
            
            zero_indices.push_back(std::make_tuple(bubble_index_start, bubble_index_end, center_distance));
            std::cout << "theta " << theta << " center_dis " << center_distance << 
            " BUBBLE INDEX START" << bubble_index_start << ", " <<  bubble_index_end << std::endl;

        }
        for (const auto& p : zero_indices) {
            bubble_index_start = std::get<0>(p);
            bubble_index_end = std::get<1>(p);
            fill_value = std::get<2>(p);
            for (int i=bubble_index_start; i <= bubble_index_end; i++) {
                ranges[i] = fill_value;
            }
        }
        
        return;
    }

    float get_velocity(float distance) {
        // TODO: Tweak 2: Choose velocity based on gap distance
        float low_vel = 0.5;
        float high_vel = 3.0;
        float atten = 1.5;
        float velocity = (high_vel - low_vel) * exp(-abs(distance) * atten) + low_vel;
        return velocity;
    }

        // """ Get desired velocity based on current error
        // """
        // # Speed is exponential w.r.t error
        // low_vel = self.get_parameter('low_vel').get_parameter_value().double_value
        // high_vel = self.get_parameter('high_vel').get_parameter_value().double_value
        // atten = self.get_parameter('velocity_attenuation').get_parameter_value().double_value

        // return (high_vel - low_vel) * np.exp(-abs(error) * atten) + low_vel


    float get_steering_angle(float point_angle) {
        // TODO: steering angle control
        float k_p = this->get_parameter("k_p").as_double();
        float k_d = this->get_parameter("k_d").as_double();
        float alpha = this->get_parameter("alpha").as_double();
        float steer_angle = k_p * point_angle + k_d * (point_angle - prev_angle);
        steer_angle = std::max(-MAX_STEER, std::min(steer_angle, MAX_STEER));
        prev_angle = point_angle;
        steer_angle = alpha * steer_angle + (1-alpha) * prev_steer;
        prev_steer = steer_angle;
        return steer_angle;
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        std::cout <<  "@=> Start of new scan data" << std::endl;
        // 1. update lidar params
        range_length =    scan_msg->ranges.size();
        range_min =       scan_msg->range_min;
        range_max =       scan_msg->range_max;
        angle_min =       scan_msg->angle_min;
        angle_max =       scan_msg->angle_max;
        angle_increment = scan_msg->angle_increment;
        front_idx_start = angle_to_index(-M_PI / 2);
        front_idx_end   = range_length - front_idx_start; // range(front_idx_start, front_idx_end)
        std::cout << "range length " << range_length << "angle min " << angle_min <<std::endl; 
        std::cout <<  M_PI << "-90~90 indices " << front_idx_start << ", "<< 
        front_idx_end << ", " << scan_msg->angle_min + scan_msg->angle_increment * front_idx_start<<std::endl;
        
        // 2. Preprocess ranges data
        float ranges[range_length];
        preprocess_lidar(&scan_msg->ranges[0], ranges);
        for (int i=0; i<range_length; i++) {
            if (i % 10 == 0) {
                std::cout << std::endl;
            }
            std::cout << ranges[i] << " ";
            
        }
        std::cout << std::endl;

        // 3. Find closest point and (disparity) points and create bubble.
        std::vector<std::pair<int, int>> bubble_indices = find_bubbles(ranges);
        create_bubbles(ranges, bubble_indices);
        std::cout << "BUBBLE_INDEX: " << std::endl;
        for (int i=0; i < bubble_indices.size(); i++) {
            std::cout << bubble_indices[i].first << ", " << bubble_indices[i].second << ";";
        }
        for (int i=0; i < range_length; i++) {
            if (i % 10 == 0) {
                std::cout << std::endl;
            }
            std::cout << ranges[i] << " ";
            
        }

        // 4. Find max length gap
        int gap_indices[2];
        find_max_gap(ranges, gap_indices);
        std::cout <<  std::endl << "gap_indices " << gap_indices[0] << ", stop " << gap_indices[1] << std::endl;
        
        // 5. Find the best point in the gap
        int target_index;
        double angle;
        target_index = find_best_point(ranges, gap_indices);
        angle = scan_msg->angle_min + scan_msg->angle_increment * target_index;
        std::cout <<  std::endl << "best idx " << target_index << ", value " << 
        ranges[target_index] << "heading: " << angle << std::endl;
        
        // 6. Publish Drive message
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.drive.steering_angle = get_steering_angle(angle);

        // TODO: SPEED AND STEERING CONTROL
        drive_msg.drive.speed = get_velocity(drive_msg.drive.steering_angle);

        // if (angle > -M_PI/36 or angle < M_PI/36) {
        //     drive_msg.drive.speed = 5.0;
        // }
        // else if ((angle > -M_PI/18 and angle < -M_PI/36) or (angle < M_PI/18 and angle > M_PI/36)) {
        //     drive_msg.drive.speed = 1.0;
        // }
        // else {
        //     drive_msg.drive.speed = 0.5;
        // }
        
        // drive_msg.drive.speed = 1.0;
        publisher_->publish(drive_msg);
    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}