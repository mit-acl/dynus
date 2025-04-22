
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/header.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "dynus/dynus.hpp" // TODO<investigate>: To make dynus_interfaces::msg::State work, we need to include dynus/dynus.hpp, but I'm not sure why. Investigate this.
#include <dynus_interfaces/msg/state.hpp>

#ifndef ODOMETRY_TO_STATE_NODE_HPP
#define ODOMETRY_TO_STATE_NODE_HPP

class OdometryToStateNode : public rclcpp::Node
{
public:
    OdometryToStateNode();

private:
    void callback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Publisher<dynus_interfaces::msg::State>::SharedPtr state_publisher_;
};

#endif // ODOMETRY_TO_STATE_NODE_HPP