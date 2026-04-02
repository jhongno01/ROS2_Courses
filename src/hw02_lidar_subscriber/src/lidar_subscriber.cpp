#include <functional>
#include <memory>
#include <chrono>
#include <array>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "more_interfaces/msg/hw02_numbers.hpp"

using std::placeholders::_1, std::placeholders::_2, std::placeholders::_3;
using namespace std::chrono_literals;

class LidarSubscriber : public rclcpp::Node
{
public:
    LidarSubscriber()
        : Node("lidar_subscriber"), angles_deg_{0, 0, 0}, angles_rad_{0.0f, 0.0f, 0.0f}, distances_{0.0f, 0.0f, 0.0f} 
        , prev_obstacle_state_{false, false, false}, first_check_(true)  
    {
        lidar_angle_sub_ = this->create_subscription<more_interfaces::msg::Hw02Numbers>
            ("lidar_angle", 10, std::bind(&LidarSubscriber::lidar_angle_topic_callback, this, _1));
        
        // QoS 설정을 SensorDataQoS로 변경하여 센서 데이터에 적합한 QoS 프로파일을 사용하도록 한다.
            // QoS : Quality of Service의 약자로, ROS 2에서 메시지 전달의 신뢰성, 지연 시간, 내구성 등을 제어하는 설정이다. 토픽의 송수신 QoS가 같아야 한다.
        auto default_qos = rclcpp::SensorDataQoS();
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", default_qos, std::bind(&LidarSubscriber::lidar_callback, this, _1));
            
        timer_ = this->create_wall_timer(500ms, std::bind(&LidarSubscriber::timer_callback, this));
    }

private:
    void lidar_angle_topic_callback(const more_interfaces::msg::Hw02Numbers::SharedPtr msg)
    {
        angles_deg_[0] = msg->a;  // deg로 받은 각도를 그대로 저장한다.
        angles_deg_[1] = msg->b;
        angles_deg_[2] = msg->c;    
        RCLCPP_INFO(this->get_logger(), "Monitor angles: '%d', '%d', '%d'", 
            msg->a, msg->b, msg->c );
        angles_rad_[0] = M_PI * angles_deg_[0] / 180.0; // 각도를 라디안으로 변환하여 저장한다.
        angles_rad_[1] = M_PI * angles_deg_[1] / 180.0;
        angles_rad_[2] = M_PI * angles_deg_[2] / 180.0;   
    }

    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // 라이다 스캔 메시지에서 각도에 해당하는 인덱스를 계산하여 해당 인덱스의 거리 값을 가져온다.
        int index0 = (0 - msg->angle_min) / msg->angle_increment; // 0도에 해당하는 인덱스 계산
        int index1 = (angles_rad_[0] - msg->angle_min) / msg->angle_increment;
        int index2 = (angles_rad_[1] - msg->angle_min) / msg->angle_increment;
        int index3 = (angles_rad_[2] - msg->angle_min) / msg->angle_increment;

        distances_[0] = msg->ranges[index1-index0]; // 0도에서의 인덱스를 기준으로 각도에 해당하는 인덱스의 거리 값을 가져온다.
        distances_[1] = msg->ranges[index2-index0];
        distances_[2] = msg->ranges[index3-index0];
    }

    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Distances at angles deg:\n%d deg: %f m\n %d deg: %f m\n %d deg: %f m", 
        angles_deg_[0], distances_[0], angles_deg_[1], distances_[1], angles_deg_[2], distances_[2]);
        
        for (int i = 0; i < 3; i++)
        {
            bool current_obstacle = std::isfinite(distances_[i]) && (distances_[i] < 0.4f);

            if (!first_check_)
            {
                if (!prev_obstacle_state_[i] && current_obstacle)
                {
                    RCLCPP_INFO(this->get_logger(),
                        "(%d deg) 장애물 생김", angles_deg_[i]);
                }
                else if (prev_obstacle_state_[i] && !current_obstacle)
                {
                    RCLCPP_INFO(this->get_logger(),
                        "(%d deg) 장애물 사라짐", angles_deg_[i]);
                }
            }

            prev_obstacle_state_[i] = current_obstacle;
        }

        first_check_ = false;
    }
    
    rclcpp::Subscription<more_interfaces::msg::Hw02Numbers>::SharedPtr lidar_angle_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::array<int, 3> angles_deg_{0, 0, 0}; // 각도를 degree로 저장하는 배열
    std::array<float, 3> angles_rad_{0.0f, 0.0f, 0.0f};
    std::array<float, 3> distances_{0.0f, 0.0f, 0.0f};

    std::array<bool, 3> prev_obstacle_state_{false, false, false}; // 이전 타이머 콜백에서 각도별로 장애물이 감지되었는지 여부를 저장하는 배열
    bool first_check_ = true;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto aaa = std::make_shared<LidarSubscriber>();
    rclcpp::spin(aaa);
    rclcpp::shutdown();
    return 0;
};
