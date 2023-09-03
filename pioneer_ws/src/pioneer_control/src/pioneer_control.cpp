#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "wheel_msgs/msg/wheel_velocities.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/impl/utils.h"
#include <cmath>
#include <fstream>
#include <sstream>
#include <string>

using std::placeholders::_1;

#define MAX_LINEAR_SPEED 0.7
#define MAX_ANGULAR_SPEED 140.0 * (M_PI / 180.0)

class RoboticControl : public rclcpp::Node
{
    public:
    RoboticControl() : Node("robotic_control")
    {
        // Initialize PID parameters
        kp = 0.5;
        ki = 0.01;
        kd = 0.05;
        previous_error_x = 0;
        previous_error_y = 0;
        previous_error_theta = 0;

        // Initialize the desired position from position.txt
        std::ifstream infile("position.txt");
        std::string line;
        if (std::getline(infile, line))
        {
            std::istringstream iss(line);
            iss >> desired_x >> desired_y >> desired_theta;
        }

        // Initialize the publishers
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Initialize the subscriber
        robot_state_subscriber_ = this->create_subscription<robot_msgs::msg::RobotState>(
            "current_state",
            10,
            std::bind(&RoboticControl::robot_state_callback, this, std::placeholders::_1));

        // Initialize Timer for PID control loop
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&RoboticControl::control_loop, this));
    }

private:
    void robot_state_callback(const robot_msgs::msg::RobotState::SharedPtr msg)
    {
        current_x = msg->x;
        current_y = msg->y;
        current_theta = msg->theta;
    }

    private:
        void control_loop()
        {
            // Compute the PID errors
            double error_x = desired_x - current_x;
            double error_y = desired_y - current_y;
            double error_theta = desired_theta - current_theta;

            double delta_error_x = error_x - previous_error_x;
            double delta_error_y = error_y - previous_error_y;
            double delta_error_theta = error_theta - previous_error_theta;

            // Compute the control outputs
            double control_output_x = kp * error_x + ki * (error_x + previous_error_x) / 2 + kd * delta_error_x;
            double control_output_y = kp * error_y + ki * (error_y + previous_error_y) / 2 + kd * delta_error_y;
            double control_output_theta = kp * error_theta + ki * (error_theta + previous_error_theta) / 2 + kd * delta_error_theta;

            // Update previous errors
            previous_error_x = error_x;
            previous_error_y = error_y;
            previous_error_theta = error_theta;

            // Publish the Twist message for the robot
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.linear.x = control_output_x;
            twist_msg.linear.y = control_output_y;
            twist_msg.angular.z = control_output_theta;
            twist_publisher_->publish(twist_msg);

            // Publish the WheelVelocities message
            auto wheel_velocities_msg = wheel_msgs::msg::WheelVelocities();
            wheel_velocities_msg.left_velocity = control_output_x - control_output_theta / 2.0;
            wheel_velocities_msg.right_velocity = control_output_x + control_output_theta / 2.0;
            wheel_velocities_publisher_->publish(wheel_velocities_msg);
        }

        double kp, ki, kd;
        double previous_error_x, previous_error_y, previous_error_theta;
        double desired_x, desired_y, desired_theta;
        double current_x = 0.0, current_y = 0.0, current_theta = 0.0;

        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
        rclcpp::Publisher<wheel_msgs::msg::WheelVelocities>::SharedPtr wheel_velocities_publisher_;
        rclcpp::Subscription<robot_msgs::msg::RobotState>::SharedPtr robot_state_subscriber_;
        rclcpp::TimerBase::SharedPtr timer_;
};

class WheelVelocitiesPublisher : public rclcpp::Node
{
    public:
        WheelVelocitiesPublisher()
        : Node("wheel_velocities_publisher"), timer_period_(0.1), run_duration_(0)
        {
            std::ifstream infile("velocities.txt");
            std::string line;
            if (std::getline(infile, line))
            {
                std::istringstream iss(line);
                iss >> left_velocity_ >> right_velocity_ >> run_duration_;
            }

            publisher_ = this->create_publisher<wheel_msgs::msg::WheelVelocities>("/wheel_velocities", 10);
            timer_ = this->create_wall_timer(
                std::chrono::duration<float>(timer_period_),
                std::bind(&WheelVelocitiesPublisher::publish_velocities, this)
            );
        }
    private:
        void publish_velocities()
        {
            if (rclcpp::Clock().now().seconds() <= run_duration_)
            {
                auto message = wheel_msgs::msg::WheelVelocities();
                message.left_velocity = left_velocity_;
                message.right_velocity = right_velocity_;
                publisher_->publish(message);
            }
            else
            {
                timer_->cancel();
            }
        }
        rclcpp::Publisher<wheel_msgs::msg::WheelVelocities>::SharedPtr publisher_;
            rclcpp::TimerBase::SharedPtr timer_;
            float timer_period_;
            float left_velocity_;
            float right_velocity_;
            float run_duration_;
};

class PioneerController : public rclcpp::Node
{
    public:
        PioneerController()
        : Node("pioneer_control")
        {
            velocities_subscription_ = this->create_subscription<wheel_msgs::msg::WheelVelocities>(
                "wheel_velocities", 10, std::bind(&PioneerController::velocities_callback, this, _1));

            twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

            pose_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "pose", 10, std::bind(&PioneerController::pose_callback, this, _1));

            wheel_velocities_publisher_ = this->create_publisher<wheel_msgs::msg::WheelVelocities>("/wheel_velocities", 10);
        }

    private:
        void velocities_callback(const wheel_msgs::msg::WheelVelocities::SharedPtr msg)
        {
            double linear_velocity = (msg->left_velocity + msg->right_velocity) / 2.0;
            double angular_velocity = (msg->right_velocity - msg->left_velocity) / 0.268;

            auto twist = geometry_msgs::msg::Twist();
            twist.linear.x = linear_velocity > MAX_LINEAR_SPEED ? MAX_LINEAR_SPEED : linear_velocity;
            twist.angular.z = angular_velocity > MAX_ANGULAR_SPEED ? MAX_ANGULAR_SPEED : angular_velocity;
            twist_publisher_->publish(twist);
        }

        void pose_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
        {
            double x = msg->pose.pose.position.x;
            double y = msg->pose.pose.position.y;

            if (x != 0 || y !=0) {
                RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Pose: value of x: " << x << ", and value of y: " << y);
            }
        }

        rclcpp::Subscription<wheel_msgs::msg::WheelVelocities>::SharedPtr velocities_subscription_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_subscription_;
        rclcpp::Publisher<wheel_msgs::msg::WheelVelocities>::SharedPtr wheel_velocities_publisher_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    auto node_1 = std::make_shared<PioneerController>();
    auto node_2 = std::make_shared<WheelVelocitiesPublisher>();
    auto node_3 = std::make_shared<RoboticControl>();

    rclcpp::executors::SingleThreadedExecutor executor;

    executor.add_node(node_1);
    executor.add_node(node_2);
    executor.add_node(node_3);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
