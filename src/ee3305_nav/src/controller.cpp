#include <iostream>
#include <iomanip>
#include <memory>
#include <chrono>
#include <vector>
#include <cmath> // Include cmath for math functions

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "ee3305_nav/ee3305_nav.hpp"

using namespace std::chrono_literals; // required for using the chrono literals such as "ms" or "s"

namespace ee3305
{
    /** Publishes the raw occupancy grid data, and the global costmap that has inflation cost values. */
    class Controller : public rclcpp::Node
    {

    private:
        // ----------- Publishers / Subscribers --------------
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;
        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr sub_path;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_cmd_vel;

        // ----------- Timers -------------------
        rclcpp::TimerBase::SharedPtr timer_main; // contains the timer that runs the main looping function at regular intervals.

        // ----------- Parameters ---------------
        double frequency;
        double lookahead_distance;
        double stop_thres;
        double lookahead_lin_vel;

        double max_lin_vel;
        double max_ang_vel;
        double max_lin_acc;
        double max_ang_acc;

        // ----------- States / Others -------------
        std::vector<double> plan_flat;
        double rbt_x;
        double rbt_y;
        double rbt_h;

        double prev_lin_vel;
        double prev_ang_vel;
        rclcpp::Time prev_time;

    public:
        /** Constructor. Run only once when this node is first created in `main()`. */
        explicit Controller()
            : Node("controller")
        {
            initStates();
            initParams();
            initTopics();

            initTimers();
        }

    private:
        void initStates()
        {
            rbt_x = NAN;
            rbt_y = NAN;
            rbt_h = NAN;

            prev_lin_vel = 0.0;
            prev_ang_vel = 0.0;
            prev_time = this->now();
        }

        /** Initializes and read parameters, if any. */
        void initParams()
        {
            initParam(this, "frequency", frequency);
            initParam(this, "lookahead_distance", lookahead_distance);
            initParam(this, "stop_thres", stop_thres);
            initParam(this, "lookahead_lin_vel", lookahead_lin_vel);

            // Read maximum velocities and accelerations
            initParam(this, "max_lin_vel", max_lin_vel);
            initParam(this, "max_ang_vel", max_ang_vel);
            initParam(this, "max_lin_acc", max_lin_acc);
            initParam(this, "max_ang_acc", max_ang_acc);
        }

        /** Initializes topics and messages, if any. */
        void initTopics()
        {
            sub_path = this->create_subscription<nav_msgs::msg::Path>(
                "path", rclcpp::SensorDataQoS(),
                std::bind(&Controller::cbPath, this, std::placeholders::_1));

            sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
                "odom", rclcpp::SensorDataQoS(),
                std::bind(&Controller::cbOdom, this, std::placeholders::_1));

            pub_cmd_vel = this->create_publisher<geometry_msgs::msg::Twist>(
                "cmd_vel", rclcpp::SystemDefaultsQoS());
        }

        void cbPath(nav_msgs::msg::Path::SharedPtr msg)
        {
            // Flatten the path into a vector of x and y coordinates
            plan_flat.clear();
            for (const geometry_msgs::msg::PoseStamped &pose : msg->poses)
            {
                plan_flat.push_back(pose.pose.position.x);
                plan_flat.push_back(pose.pose.position.y);
            }
        }

        void cbOdom(nav_msgs::msg::Odometry::SharedPtr msg)
        {
            // Update robot's current position and heading
            rbt_x = msg->pose.pose.position.x;
            rbt_y = msg->pose.pose.position.y;
            const auto &q = msg->pose.pose.orientation;
            double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
            double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
            rbt_h = atan2(siny_cosp, cosy_cosp);
        }

        /** Initializes the timers with their callbacks.*/
        void initTimers()
        {
            timer_main = this->create_wall_timer(
                std::chrono::duration<double>(1.0 / frequency),
                std::bind(&Controller::cbTimerMain, this));
        }

        /** The function that is run at regular intervals */
        void cbTimerMain()
        {
            // Check if robot's pose is available
            if (std::isnan(rbt_x) || std::isnan(rbt_y) || std::isnan(rbt_h))
            {
                RCLCPP_WARN(this->get_logger(), "Robot pose not available.");
                return;
            }

            // Get current time and compute elapsed time
            rclcpp::Time current_time = this->now();
            double dt = (current_time - prev_time).seconds();
            if (dt == 0.0)
            {
                dt = 1.0 / frequency; // default to expected dt
            }
            prev_time = current_time;

            // Helper function for sign
            auto sgn = [](double x) -> int { return (x > 0) - (x < 0); };

            // Find the closest point on the path
            double min_dist_sq = std::numeric_limits<double>::infinity();
            size_t closest_idx = 0;
            for (size_t i = 0; i < plan_flat.size(); i += 2)
            {
                double px = plan_flat[i];
                double py = plan_flat[i + 1];

                double dx = px - rbt_x;
                double dy = py - rbt_y;
                double dist_sq = dx * dx + dy * dy;

                if (dist_sq < min_dist_sq)
                {
                    min_dist_sq = dist_sq;
                    closest_idx = i;
                }
            }

            // Find the lookahead point
            double lookahead_distance_sq = lookahead_distance * lookahead_distance;
            bool lookahead_point_found = false;
            size_t lookahead_idx = closest_idx;

            for (size_t i = closest_idx; i < plan_flat.size(); i += 2)
            {
                double px = plan_flat[i];
                double py = plan_flat[i + 1];

                double dx = px - rbt_x;
                double dy = py - rbt_y;
                double dist_sq = dx * dx + dy * dy;

                if (dist_sq >= lookahead_distance_sq)
                {
                    lookahead_idx = i;
                    lookahead_point_found = true;
                    break;
                }
            }

            if (!lookahead_point_found)
            {
                lookahead_idx = plan_flat.size() - 2;
            }

            // Get the lookahead point
            double lx = plan_flat[lookahead_idx];
            double ly = plan_flat[lookahead_idx + 1];

            // Transform lookahead point to robot frame
            double dx = lx - rbt_x;
            double dy = ly - rbt_y;

            double cos_h = cos(rbt_h);
            double sin_h = sin(rbt_h);

            double x_r = cos_h * dx + sin_h * dy;
            double y_r = -sin_h * dx + cos_h * dy;

            // Compute curvature
            double curvature = 0.0;
            if (x_r != 0.0 || y_r != 0.0)
            {
                curvature = 2.0 * y_r / (x_r * x_r + y_r * y_r);
            }

            // Compute desired velocities
            double desired_lin_vel = lookahead_lin_vel;
            double desired_ang_vel = curvature * desired_lin_vel;

            // Compute distance to goal
            double goal_x = plan_flat[plan_flat.size() - 2];
            double goal_y = plan_flat[plan_flat.size() - 1];
            double dx_goal = goal_x - rbt_x;
            double dy_goal = goal_y - rbt_y;
            double dist_to_goal = sqrt(dx_goal * dx_goal + dy_goal * dy_goal);

            if (dist_to_goal < stop_thres)
            {
                // Stop the robot
                desired_lin_vel = 0.0;
                desired_ang_vel = 0.0;
                prev_lin_vel = 0.0;
                prev_ang_vel = 0.0;

                // Publish zero velocities
                geometry_msgs::msg::Twist msg;
                msg.linear.x = desired_lin_vel;
                msg.angular.z = desired_ang_vel;
                pub_cmd_vel->publish(msg);

                RCLCPP_INFO(this->get_logger(), "Goal reached.");
                return;
            }

            // Apply linear acceleration constraints
            double max_lin_vel_change = max_lin_acc * dt;
            double lin_vel_diff = desired_lin_vel - prev_lin_vel;
            if (std::abs(lin_vel_diff) > max_lin_vel_change)
            {
                desired_lin_vel = prev_lin_vel + sgn(lin_vel_diff) * max_lin_vel_change;
            }

            // Apply linear velocity constraints
            if (desired_lin_vel > max_lin_vel)
            {
                desired_lin_vel = max_lin_vel;
            }
            else if (desired_lin_vel < -max_lin_vel)
            {
                desired_lin_vel = -max_lin_vel;
            }

            // Apply angular acceleration constraints
            double max_ang_vel_change = max_ang_acc * dt;
            double ang_vel_diff = desired_ang_vel - prev_ang_vel;
            if (std::abs(ang_vel_diff) > max_ang_vel_change)
            {
                desired_ang_vel = prev_ang_vel + sgn(ang_vel_diff) * max_ang_vel_change;
            }

            // Apply angular velocity constraints
            if (desired_ang_vel > max_ang_vel)
            {
                desired_ang_vel = max_ang_vel;
            }
            else if (desired_ang_vel < -max_ang_vel)
            {
                desired_ang_vel = -max_ang_vel;
            }

            // Update previous velocities
            prev_lin_vel = desired_lin_vel;
            prev_ang_vel = desired_ang_vel;

            // Publish velocity command
            geometry_msgs::msg::Twist msg;
            msg.linear.x = desired_lin_vel;
            msg.angular.z = desired_ang_vel;
            pub_cmd_vel->publish(msg);
        }
    };
}

/** The main function to compile. */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ee3305::Controller>());
    rclcpp::shutdown();
    return 0;
}
