#include "ee4308_drone/controller.hpp"

#include <algorithm>
#include <limits>

namespace ee4308::drone
{
    Controller::Controller(
        const rclcpp::NodeOptions &options,
        const std::string &name = "controller") 
        : Node(name, options)
    {
        this->frequency_ = ee4308::getParameter<double>(this, "frequency", 20.0).as_double();
        this->enable_ = ee4308::getParameter<bool>(this, "enable", true).as_bool();
        this->lookahead_distance_ = ee4308::getParameter<double>(this, "lookahead_distance", 1.0).as_double();
        this->max_xy_vel_ = ee4308::getParameter<double>(this, "max_xy_vel", 1.0).as_double();
        this->max_z_vel_ = ee4308::getParameter<double>(this, "max_z_vel", 0.5).as_double();
        this->yaw_vel_ = ee4308::getParameter<double>(this, "yaw_vel", 0.3).as_double();
        this->kp_xy_ = ee4308::getParameter<double>(this, "kp_xy", 1.0).as_double();
        this->kp_z_ = ee4308::getParameter<double>(this, "kp_z", 1.0).as_double();

        this->pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", rclcpp::ServicesQoS());
        this->sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::SensorDataQoS(),
            std::bind(&Controller::callbackSubOdom_, this, std::placeholders::_1));
        this->sub_plan_ = this->create_subscription<nav_msgs::msg::Path>(
            "plan", rclcpp::SensorDataQoS(),
            std::bind(&Controller::callbackSubPlan_, this, std::placeholders::_1));

        this->received_odom_ = false;

        this->timer_ = this->create_timer(1s / this->frequency_, std::bind(&Controller::callbackTimer_, this));
    }

    void Controller::callbackSubOdom_(const nav_msgs::msg::Odometry msg)
    {
        this->odom_ = msg;
        this->received_odom_ = true;
    }

    void Controller::callbackSubPlan_(const nav_msgs::msg::Path msg)
    {
        this->plan_ = msg;
    }

    void Controller::callbackTimer_()
    {
        if (!enable_) return;

        // Hover if no odom or no plan
        if (!received_odom_ || plan_.poses.empty()) {
            publishCmdVel_(0.0, 0.0, 0.0, 0.0);
            return;
        }

        // Current drone position in world frame
        const auto &pos = odom_.pose.pose.position;
        const double x = pos.x;
        const double y = pos.y;
        const double z = pos.z;

        // 1) Find closest point on path in x-y plane
        size_t closest_idx = 0;
        double closest_dist = std::numeric_limits<double>::infinity();

        for (size_t i = 0; i < plan_.poses.size(); ++i) {
            const auto &p = plan_.poses[i].pose.position;
            const double d = std::hypot(p.x - x, p.y - y);
            if (d < closest_dist) {
                closest_dist = d;
                closest_idx = i;
            }
        }

        // 2) Search forward for lookahead point
        // Use first point whose x-y distance is >= lookahead_distance_
        // If not found, use the last path point
        size_t lookahead_idx = plan_.poses.size() - 1;
        for (size_t i = closest_idx; i < plan_.poses.size(); ++i) {
            const auto &p = plan_.poses[i].pose.position;
            const double d = std::hypot(p.x - x, p.y - y);
            if (d >= lookahead_distance_) {
                lookahead_idx = i;
                break;
            }
        }

        const auto &target = plan_.poses[lookahead_idx].pose.position;

        // 3) Compute error to target
        const double dx_world = target.x - x;
        const double dy_world = target.y - y;
        const double dz_world = target.z - z;

        // Convert x/y error from world frame to drone body frame
        const double yaw = ee4308::getYawFromQuaternion(odom_.pose.pose.orientation);
        const double cy = std::cos(yaw);
        const double sy = std::sin(yaw);

        const double dx_body =  cy * dx_world + sy * dy_world;
        const double dy_body = -sy * dx_world + cy * dy_world;

        // Proportional control
        double x_vel = kp_xy_ * dx_body;
        double y_vel = kp_xy_ * dy_body;
        double z_vel = kp_z_ * dz_world;

        // 4) Clamp horizontal velocity magnitude
        const double xy_speed = std::hypot(x_vel, y_vel);
        if (xy_speed > max_xy_vel_ && xy_speed > 1e-9) {
            const double scale = max_xy_vel_ / xy_speed;
            x_vel *= scale;
            y_vel *= scale;
        }

        // Clamp vertical velocity
        z_vel = std::clamp(z_vel, -max_z_vel_, max_z_vel_);

        // 5) Publish body-frame velocity command and yaw velocity
        publishCmdVel_(x_vel, y_vel, z_vel, yaw_vel_);
    }

        // ==== make use of ====
        // plan_.poses
        // odom_
        // ee4308::getYawFromQuaternion()
        // std::hypot()
        // std::clamp()
        // std::cos(), std::sin() 
        // lookahead_distance_
        // kp_xy_
        // kp_z_
        // max_xy_vel_
        // max_z_vel_
        // yaw_vel_
        // publishCmdVel__()
        // =========

        // publish

    // ================================  PUBLISHING ========================================
    void Controller::publishCmdVel_(double x_vel, double y_vel, double z_vel, double yaw_vel)
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = x_vel;
        cmd_vel.linear.y = y_vel;
        cmd_vel.linear.z = z_vel;
        cmd_vel.angular.z = yaw_vel;
        // RCLCPP_INFO(this->get_logger(), "%f,%f,%f,%f", x_vel, y_vel, z_vel, yaw_vel);
        if (!std::isfinite(x_vel) || !std::isfinite(y_vel) || !std::isfinite(z_vel) || !std::isfinite(yaw_vel))
        {
            RCLCPP_WARN(this->get_logger(), 
                "Cmd velocities are inf or nan. Controller or estimator problem. CmdVels(x,y,z,yaw): %6.3f, %6.3f, %6.3f, %6.3f", 
                x_vel, y_vel, z_vel, yaw_vel);
        }
        pub_cmd_vel_->publish(cmd_vel);
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ee4308::drone::Controller);
