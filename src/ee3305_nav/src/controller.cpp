#include <iostream>
#include <iomanip>
#include <memory>
#include <chrono>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
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
        double max_lin_acc;
        double max_lin_vel;
        double max_ang_acc;
        double max_ang_vel;

        // ----------- States / Others -------------
        std::vector<double> plan_flat;
        double rbt_x;
        double rbt_y;
        double rbt_h;
        double prev_lin_vel;
        double prev_ang_vel;
        double prev_time;

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
        }

        /** Initializes and read parameters, if any. */
        void initParams()
        {
            initParam(this, "frequency", frequency);
            initParam(this, "lookahead_distance", lookahead_distance);
            initParam(this, "stop_thres", stop_thres);
            initParam(this, "lookahead_lin_vel", lookahead_lin_vel);
            initParam(this, "max_lin_acc", max_lin_acc);
            initParam(this, "max_lin_vel", max_lin_vel);
            initParam(this, "max_ang_acc", max_ang_acc);
            initParam(this, "max_ang_vel", max_ang_vel);
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
            plan_flat.clear();
            for (const geometry_msgs::msg::PoseStamped &pose : msg->poses)
            {
                plan_flat.push_back(pose.pose.position.x);
                plan_flat.push_back(pose.pose.position.y);
            }
        }

        void cbOdom(nav_msgs::msg::Odometry::SharedPtr msg)
        { // transform assumes zero transform between `map` frame and `odom` frame.
            rbt_x = msg->pose.pose.position.x;
            rbt_y = msg->pose.pose.position.y;
            const auto &q = msg->pose.pose.orientation;
            double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
            double cosy_cosp = 1 - 2 * (q.y + q.y + q.z * q.z);
            rbt_h = atan2(siny_cosp, cosy_cosp);
        }


        /** Initializes the timers with their callbacks.*/
        void initTimers()
        {
            timer_main = this->create_wall_timer(
                1s / frequency,
                std::bind(&Controller::cbTimerMain, this));
        }

        /** The function that is run at regular intervals */
        /*void cbTimerMain()
        {
            // ensure the robot has a valid path and pose data
            if (plan_flat.empty() || std::isnan(rbt_x))
            {
                return;
            }

            double current_time = this->now().seconds();
            double elapsed_time = current_time - prev_time;
            prev_time = current_time;

            // Publish message
            geometry_msgs::msg::Twist msg;
            msg.angular.x = 0; // redundancy
            msg.angular.y = 0; // redundancy
            msg.angular.z = ang_vel;
            msg.linear.x = lin_vel;
            msg.linear.y = 0; // redundancy
            msg.linear.z = 0; // redundancy
            pub_cmd_vel->publish(msg);
        }*/

       void cbTimerMain()
        {

            // -- explaination of the code overall 
            /**
             * The ontroller file implements a ROS2 node that controls a robot using a
             * pure pursuit algorithm. It subscribes to path and odometry topics, 
             * calculates a lookahead point based on the robot’s current position,
             *  and transforms this point into the robot’s frame to determine curvature. 
             * 
             * The controller then also calculates the necessary linear and 
             * angular velocities to follow the path, applying constraints to 
             * acceleration and velocity based on configurable parameters. 
             * These velocities are published as cmd_vel messages to drive 
             * the robot smoothly along the path. 
             * 
             * Key components include (for now)
             * - velocity constraints, 
             * - timer-based control loops, 
             * - transformations for path following. (need to double cfm)
             */

            // --- code starts here //
            // 1. ensure the robot has a valid path and pose data
            if (plan_flat.empty() || std::isnan(rbt_x)) {
                return;
            }

            // 2. Finding the closest point on the path
            double closest = INFINITY;
            int ci = 0;
            for (int i = 0; i < plan_flat.size(); i += 2)
            {
                double dx = plan_flat[i] - rbt_x;
                double dy = plan_flat[i + 1] - rbt_y;
                double dist = dx * dx + dy * dy;
                if (dist < closest)
                {
                    closest = dist;
                    ci = i;
                }
            }

            // 3. Find the lookahead point
            double look_x = plan_flat[ci];
            double look_y = plan_flat[ci + 1];
            for (int i = ci; i < plan_flat.size(); i += 2)
            {
                double dx = plan_flat[i] - rbt_x;
                double dy = plan_flat[i + 1] - rbt_y;
                if (sqrt(dx * dx + dy * dy) > lookahead_distance)
                {
                    look_x = plan_flat[i];
                    look_y = plan_flat[i + 1];
                    break;
                }
            }

            // 4. Transform the lookahead point to the robot frame
            double dx = look_x - rbt_x;
            double dy = look_y - rbt_y;
            double look_rx = dx * cos(rbt_h) + dy * sin(rbt_h);
            double look_ry = dy * cos(rbt_h) - dx * sin(rbt_h);

            // 5. Calculate curvaturethen get the elapsed time
            double curvature = (2 * look_ry) / (look_rx * look_rx + look_ry * look_ry);

            double current_time = this->now().seconds();
            double elapsed_time = current_time - prev_time;
            prev_time = current_time;

            // 6. Constrain the linear acc
            double lin_acc = (lookahead_lin_vel - prev_lin_vel) / elapsed_time;
            if (fabs(lin_acc) > max_lin_acc) {
                lin_acc = copysign(max_lin_acc, lin_acc);
            }

            double lin_vel = prev_lin_vel + lin_acc * elapsed_time;
            lin_vel = std::min(lin_vel, max_lin_vel);
            prev_lin_vel = lin_vel;

            // 7. Calculate angular vel (using curavture info and linear velcity))
            double ang_vel = curvature * lin_vel;

            // 8. Constrain angular acceleration
            double ang_acc = (ang_vel - prev_ang_vel) / elapsed_time;
            if (fabs(ang_acc) > max_ang_acc) {
                ang_acc = copysign(max_ang_acc, ang_acc);
            }

            // need ot double check this
            ang_vel = prev_ang_vel + ang_acc * elapsed_time;
            ang_vel = std::min(ang_vel, max_ang_vel);
            prev_ang_vel = ang_vel;

            // Publish message
            geometry_msgs::msg::Twist msg;
            msg.angular.x = 0; // redundancy
            msg.angular.y = 0; // redundancy
            msg.angular.z = ang_vel;
            msg.linear.x = lin_vel;
            msg.linear.y = 0; // redundancy
            msg.linear.z = 0; // redundancy
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