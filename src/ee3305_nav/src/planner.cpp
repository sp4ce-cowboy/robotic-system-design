#include <iostream>
#include <iomanip>
#include <memory>
#include <chrono>
#include <vector>

// added by dawn
#include <queue>
#include <cmath>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/srv/get_plan.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ee3305_nav/ee3305_nav.hpp"
#include "ee3305_nav/planner.hpp"

using namespace std::chrono_literals; // required for using the chrono literals such as "ms" or "s"

namespace ee3305
{
    class Planner : public rclcpp::Node // this class handles global path planning
    {

    private:
        // introduce publisher, subscriber, service
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_global_costmap;
        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_path;
        rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr srv_get_plan;

        // insert parameters if any

        // introduce states
        std::vector<int8_t> map;
        double resolution;
        double origin_x;
        double origin_y;
        int rows;
        int cols;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public:
        explicit Planner() : rclcpp::Node("planner") // constructor with 4 methods
        {
            initStates();
            initParams();
            initTopics();
            initServices();
        }

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    private: // to elaborate on private functions/methods

        void initStates() // initialises internal state variables
        {
            resolution = 0;
            origin_x = 0;
            origin_y = 0;
            cols = 0;
        }

        void initParams() // initialises and read parameters
        {
            initParam(this, "frequency", frequency);
            initParam(this, "goal_tolerance", goal_tolerance);
            initParam(this, "max_path_length", max_path_length);
            initParam(this, "inflation_radius", inflation_radius);
            initParam(this, "obstacle_cost_threshold", obstacle_cost_threshold);
            initParam(this, "start_x", start_x);
            initParam(this, "start_y", start_y);
            initParam(this, "goal_x", goal_x);
            initParam(this, "goal_y", goal_y);
            initParam(this, "allow_unknown", allow_unknown);
            initParam(this, "path_smoothing", path_smoothing);
        }

        void initTopics() // initialises topics and messages
        {
            auto qos = rclcpp::SensorDataQoS();
            qos.durability(rclcpp::DurabilityPolicy::TransientLocal);
            qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
            qos.keep_last(1);

            // create subscriber
            sub_global_costmap = this->create_subscription<nav_msgs::msg::OccupancyGrid>("global_costmap", qos, std::bind(&Planner::cbGlobalCostmap, this, std::placeholders::_1)); 

            // create publisher
            qos = rclcpp::SensorDataQoS();
            pub_path = this->create_publisher<nav_msgs::msg::Path>("path", qos);
        }

        void initServices()
        {
            // create service
            srv_get_plan = this->create_service<nav_msgs::srv::GetPlan>("get_plan", std::bind(&Planner::cbSrvGetPlan, this, std::placeholders::_1, std::placeholders::_2), rmw_qos_profile_services_default);
        }

        void cbGlobalCostmap(nav_msgs::msg::OccupancyGrid::SharedPtr msg) // this is called when a new occupancy grid is published for use (occupancy grid to be used as input)
        {
            // creates map
            map = msg->data;
            resolution = msg->info.resolution;
            origin_x = msg->info.origin.position.x;
            origin_y = msg->info.origin.position.y;
            rows = msg->info.height;
            cols = msg->info.width;
        }

        /** Service server that returns a path in the map coordinates */
        void cbSrvGetPlan(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
                  std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "GetPlan service request received.");

            while (map.empty()) { 
                RCLCPP_WARN_STREAM(this->get_logger(), "Waiting for a global costmap...");
                rclcpp::sleep_for(100ms);
                rclcpp::spin_some(this->get_node_base_interface());
            }

            double start_x = request->start.pose.position.x;
            double start_y = request->start.pose.position.y;
            double goal_x = request->goal.pose.position.x;
            double goal_y = request->goal.pose.position.y;

            int start_i = floor((start_x - origin_x) / resolution);
            int start_j = floor((start_y - origin_y) / resolution);
            int goal_i = floor((goal_x - origin_x) / resolution);
            int goal_j = floor((goal_y - origin_y) / resolution);

            // Check if goal is within goal tolerance
            double dist_to_goal = std::hypot(goal_x - start_x, goal_y - start_y);
            if (dist_to_goal <= goal_tolerance) {
                RCLCPP_INFO_STREAM(this->get_logger(), "Goal already reached.");
                return;
            }

            // Call the run function to generate the path
            std::vector<int> path_flat = run(start_i, start_j, goal_i, goal_j, map, rows, cols);

            nav_msgs::msg::Path path;
            for (int p = path_flat.size() - 2; p >= 0; p -= 2) {
                double i = path_flat[p];
                double j = path_flat[p + 1];

                geometry_msgs::msg::PoseStamped pose_stamped;
                pose_stamped.header.frame_id = "map";
                pose_stamped.header.stamp = this->now();
                pose_stamped.pose.position.x = (i * resolution) + origin_x + resolution / 2;
                pose_stamped.pose.position.y = (j * resolution) + origin_y + resolution / 2;

                pose_stamped.pose.position.z = 0;
                pose_stamped.pose.orientation.x = 0;
                pose_stamped.pose.orientation.y = 0;
                pose_stamped.pose.orientation.z = 0;
                pose_stamped.pose.orientation.w = 1;

                path.poses.push_back(pose_stamped);
            }
            path.header.frame_id = "map";
            pub_path->publish(path);
            response->plan = path;
        }


        /*void cbSrvGetPlan(const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
                          std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
        {
            RCLCPP_INFO_STREAM(this->get_logger(), "GetPlan service request received.");

            // if map is empty:
            while (map.empty())
            { 
                // wait until a map is published
                RCLCPP_WARN_STREAM(this->get_logger(), "global_costmap topic is not published yet. Waiting for a global costmap to be published into the topic.");
                rclcpp::sleep_for(100ms);
                rclcpp::spin_some(this->get_node_base_interface());
            }

            // otherwise:
            double start_x = request->start.pose.position.x;
            double start_y = request->start.pose.position.y;
            double goal_x = request->goal.pose.position.x;
            double goal_y = request->goal.pose.position.y;

            int start_i = floor((start_x - origin_x) / resolution);
            int start_j = floor((start_y - origin_y) / resolution);
            int goal_i = floor((goal_x - origin_x) / resolution);
            int goal_j = floor((goal_y - origin_y) / resolution);

            // added this: 9.03pm
            // Convert map from vector of signed char to vector of int
            std::vector<int> map_int(map.begin(), map.end());

            // original
            // std::vector<int> path_flat = run(start_i, start_j, goal_i, goal_j); // call the run function and store the result in path_flat (all coordinates alr obtained here)
            std::vector<int> path_flat = run(start_i, start_j, goal_i, goal_j, map_int, rows, cols);
            // path_flat stores the path in grid coordinates, from goal to start
            // run function is written below, under "the main path finding algorithm"

            // the lines below are supposed to convert grid coordinates of the path to map coordinates and store the path in response:
            nav_msgs::msg::Path path;
            std::cout << "path: "; // print this

            // now that we have the list of coordinates ie. path_flat,
            // start with index p as the second last index to get i and j as shown below. then iterate through this list with p skipping every other index
            for (int p = path_flat.size() - 2; p >= 0; p -= 2)
            {
                double i = path_flat[p]; // obtain i for every p (i is a grid coordinate)
                double j = path_flat[p + 1]; // obtain j for every p+1 (j is a grid coordinate)

                geometry_msgs::msg::PoseStamped pose_stamped; // this stores the position and orientation of each point in world coordinates
                pose_stamped.header.frame_id = "map"; // allows coordinates to be referenced to world map frame
                pose_stamped.header.stamp = this->now();
                pose_stamped.pose.position.x = (i * resolution) + origin_x + resolution / 2; // obtain world coordinate x, using grid coordinate i, divide 2 to shift coordinate to the centre of the cell
                pose_stamped.pose.position.y = (j * resolution) + origin_y + resolution / 2; // obtain world coordinate y, using grid coordinate j
                std::cout << pose_stamped.pose.position.x << "," << pose_stamped.pose.position.y << ";  "; // print this

                // parameters set to ensure no rotation in 3d space, z = 0 such that it's in 2d space
                pose_stamped.pose.position.z = 0;
                pose_stamped.pose.orientation.x = 0;
                pose_stamped.pose.orientation.y = 0;
                pose_stamped.pose.orientation.z = 0;
                pose_stamped.pose.orientation.w = 1;

                path.poses.push_back(pose_stamped); // append these pose_stamped values to path.poses vector
            }
            std::cout << std::endl; // print this
            path.header.frame_id = "map";
            // publish to topic
            pub_path->publish(path); // the publisher object, pub_path, publishes "path"

            // write to response
            response->plan = path; // response returns the path from start to goal (reversed from path_flat)
        }*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////
// INSERT A* PLANNER RUN FUNCTION HERE
        std::vector<int> run(int start_x, int start_y, int goal_x, int goal_y, const std::vector<int>& map, int rows, int cols) {

            struct PlannerNode {
                int x, y;
                double g, h;
                PlannerNode* parent;

                // allows for new creation of node ...
                PlannerNode(int x, int y, double g = 0, double h = 0, PlannerNode* parent = nullptr) : x(x), y(y), g(g), h(h), parent(parent) {}

                double f() const {return g + h;}
            };

            struct Compare {
                bool operator()(PlannerNode* a, PlannerNode* b) { // operator function will return true if a > b
                    return a->f() > b->f();
                }
            };

            // calculate h cost using octile distance
            auto octile_distance = [&](int x1, int y1, int x2, int y2) {
                int dx = std::abs(x1 - x2);
                int dy = std::abs(y1 - y2);
                return (dx + dy) + (std::sqrt(2) - 2) * std::min(dx, dy);
            };

            // create priority queue called open_list which uses Compare to determine priority
            std::priority_queue<PlannerNode*, std::vector<PlannerNode*>, Compare> open_list;

            // create visited vector: all cells initialised as 'false'
            std::vector<std::vector<bool>> visited(rows, std::vector<bool>(cols, false));

            // create starting point with g-cost = 0 and h-cost to be calculated using octile distance
            PlannerNode* start_node = new PlannerNode(start_x, start_y, 0, octile_distance(start_x, start_y, goal_x, goal_y));
            open_list.push(start_node); // append the start_node to the open_list

            std::vector<std::pair<int, int>> directions = {{-1, -1}, {-1, 1}, {1, -1}, {1, 1}, {-1, 0}, {1, 0}, {0, -1}, {0, 1}};

            while (!open_list.empty()) {
                PlannerNode* current = open_list.top(); // obtain the node at the top of the priority queue and make it the current node
                open_list.pop(); // remove this node from the open list

                // do this when the goal has been reached, ie. when current coordinates = goal coordinates
                if (current->x == goal_x && current->y == goal_y) {
                    std::vector<int> path;
                    PlannerNode* node = current; // start with the current node, ie. the goal coordinates
                    while (node) { // iterate through all nodes and append them to the path (resultant path will be reversed)
                        path.push_back(node->x);
                        path.push_back(node->y);
                        node = node->parent; // assign node to the previous version, iterate through until it becomes nullptr
                    }

                    while (!open_list.empty()) {
                        delete open_list.top();
                        open_list.pop();
                    }
                    return path;
                }

                visited[current->x][current->y] = true;

                for (const auto& [dx, dy] : directions) {
                    int nx = current->x + dx;
                    int ny = current->y + dy;

                    if (nx >= 0 && nx < rows && ny >= 0 && ny < cols &&
                        !visited[nx][ny] &&
                        (allow_unknown || map[nx * cols + ny] <= obstacle_cost_threshold)) {
                        
                        double g_new = current->g + ((dx == 0 || dy == 0) ? 1 : std::sqrt(2));
                        double h_new = octile_distance(nx, ny, goal_x, goal_y);

                        PlannerNode* neighbor = new PlannerNode(nx, ny, g_new, h_new, current);
                        if (g_new < neighbor->g || h_new < neighbor->h) {
                            neighbor->g = g_new;
                            neighbor->h = h_new;
                            neighbor->parent = current;
                            open_list.push(neighbor);
                        }
                    }
                }

                /*for (const auto& [dx, dy] : directions) {
                    int nx = current->x + dx;
                    int ny = current->y + dy;

                    if (nx >= 0 && nx < rows && ny >= 0 && ny < cols && !visited[nx][ny] && map[nx * cols + ny] == 0) {
                        double g_new = current->g + ((dx == 0 || dy == 0) ? 1 : std::sqrt(2));
                        double h_new = octile_distance(nx, ny, goal_x, goal_y);

                        PlannerNode* neighbor = new PlannerNode(nx, ny, g_new, h_new, current);
                        if (g_new < neighbor->g || h_new < neighbor->h) {
                            neighbor->g = g_new;
                            neighbor->h = h_new;
                            neighbor->parent = current;
                            open_list.push(neighbor);
                        }
                    }
                }*/
            }

            std::cerr << "No path found." << std::endl;
            return {};
        }
    };
};

/** The main function to compile. */
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ee3305::Planner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}