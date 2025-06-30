#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/bool.hpp>
#include <chrono>
#include <random>

#include <sensor_msgs/msg/laser_scan.hpp>

using namespace std::chrono_literals;

enum class State { FORWARD, BACKING_UP, TURNING };

// class BumpAndGoFSM : public rclcpp::Node
// {
// public:
//     BumpAndGoFSM() : Node("bump_and_go_fsm"), state_(State::FORWARD)
//     {
//         cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
//         bump_sub_ = this->create_subscription<std_msgs::msg::Bool>(
//             "/bump", 10, std::bind(&BumpAndGoFSM::bump_callback, this, std::placeholders::_1));

//         timer_ = this->create_wall_timer(100ms, std::bind(&BumpAndGoFSM::control_loop, this));

//         rng_.seed(std::random_device()());
//     }

// private:
//     void bump_callback(const std_msgs::msg::Bool::SharedPtr msg)
//     {
//         if (msg->data && state_ == State::FORWARD)
//         {
//             RCLCPP_INFO(this->get_logger(), "Bump detected! Switching to BACKING_UP state.");
//             state_ = State::BACKING_UP;
//             state_timer_ = now();
//         }
//     }

//     void control_loop()
//     {
//         geometry_msgs::msg::Twist cmd;

//         switch (state_)
//         {
//         case State::FORWARD:
//             cmd.linear.x = 0.2;
//             break;

//         case State::BACKING_UP:
//             cmd.linear.x = -0.2;
//             if ((now() - state_timer_).seconds() > 1.0)
//             {
//                 state_ = State::TURNING;
//                 state_timer_ = now();
//                 turn_direction_ = random_turn_direction();
//                 RCLCPP_INFO(this->get_logger(), "Backing up done. Turning %s.", turn_direction_ > 0 ? "left" : "right");
//             }
//             break;

//         case State::TURNING:
//             cmd.angular.z = turn_direction_ * 1.0;
//             if ((now() - state_timer_).seconds() > 1.5)
//             {
//                 state_ = State::FORWARD;
//                 RCLCPP_INFO(this->get_logger(), "Turning done. Moving forward.");
//             }
//             break;
//         }

//         cmd_pub_->publish(cmd);
//     }

//     double random_turn_direction()
//     {
//         std::uniform_int_distribution<int> dist(0, 1);
//         return dist(rng_) == 0 ? -1.0 : 1.0;
//     }

//     rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
//     rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bump_sub_;
//     rclcpp::TimerBase::SharedPtr timer_;

//     State state_;
//     rclcpp::Time state_timer_;

//     std::mt19937 rng_;
//     double turn_direction_;
// };


class BumpAndGoFSM : public rclcpp::Node
{
public:
    BumpAndGoFSM() : Node("bump_and_go_fsm"), state_(State::FORWARD)
    {
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&BumpAndGoFSM::scan_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(100ms, std::bind(&BumpAndGoFSM::control_loop, this));

        rng_.seed(std::random_device()());
    }

private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        if (state_ == State::FORWARD && obstacle_detected(msg))
        {
            RCLCPP_INFO(this->get_logger(), "Obstacle detected with LIDAR! Switching to BACKING_UP state.");
            state_ = State::BACKING_UP;
            state_timer_ = now();
        }
    }

    bool obstacle_detected(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        
        float detection_distance = 0.5;  // meters
        int center_range = msg->ranges.size() / 3;
        
        for (size_t i = center_range; i < center_range * 2; ++i)  // Check front 1/3 of scan
        {
            float range = msg->ranges[i];
            if (!std::isnan(range) && !std::isinf(range) && range < detection_distance)
            {
                return true;
            }
        }
        return false;
    }

    void control_loop()
    {
        geometry_msgs::msg::Twist cmd;

        switch (state_)
        {
        case State::FORWARD:
            cmd.linear.x = 0.2;
            break;

        case State::BACKING_UP:
            cmd.linear.x = -0.2;
            if ((now() - state_timer_).seconds() > 2.0)
            {
                state_ = State::TURNING;
                state_timer_ = now();
                turn_direction_ = random_turn_direction();
                RCLCPP_INFO(this->get_logger(), "Backing up done. Turning %s.", turn_direction_ > 0 ? "left" : "right");
            }
            break;

        case State::TURNING:
            cmd.angular.z = turn_direction_ * 2.0;
            if ((now() - state_timer_).seconds() > 5.0)
            {
                state_ = State::FORWARD;
                RCLCPP_INFO(this->get_logger(), "Turning done. Moving forward.");
            }
            break;
        }

        cmd_pub_->publish(cmd);
    }

    double random_turn_direction()
    {
        std::uniform_int_distribution<int> dist(0, 1);
        return dist(rng_) == 0 ? -1.0 : 1.0;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    State state_;
    rclcpp::Time state_timer_;

    std::mt19937 rng_;
    double turn_direction_;
};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BumpAndGoFSM>());
    rclcpp::shutdown();
    return 0;
}
