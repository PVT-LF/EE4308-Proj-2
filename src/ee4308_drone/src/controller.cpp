#include "ee4308_drone/controller.hpp"
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
        if (!enable_)
            return;

        if (!this->received_odom_)
        {
            publishCmdVel_(0, 0, 0, yaw_vel_);
            return;
        }

        if (plan_.poses.empty())
        {
            // RCLCPP_WARN_STREAM(this->get_logger(), "No path published");
            publishCmdVel_(0, 0, 0, yaw_vel_);
            return;
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

        double drone_x = odom_.pose.pose.position.x;
        double drone_y = odom_.pose.pose.position.y;
        double drone_z = odom_.pose.pose.position.z;
        double drone_yaw = ee4308::getYawFromQuaternion(odom_.pose.pose.orientation);

        size_t closest_idx = 0;
        double closest_dist_sq = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < plan_.poses.size(); ++i)
        {
            double dx = plan_.poses[i].pose.position.x - drone_x;
            double dy = plan_.poses[i].pose.position.y - drone_y;
            double dz = plan_.poses[i].pose.position.z - drone_z;
            double dist_sq = dx * dx + dy * dy + dz * dz;
            if (dist_sq < closest_dist_sq)
            {
                closest_dist_sq = dist_sq;
                closest_idx = i;
            }
        }

        size_t lookahead_idx = plan_.poses.size() - 1;
        for (size_t i = closest_idx; i < plan_.poses.size(); ++i)
        {
            double dx = plan_.poses[i].pose.position.x - drone_x;
            double dy = plan_.poses[i].pose.position.y - drone_y;
            double dz = plan_.poses[i].pose.position.z - drone_z;
            double distance = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (distance >= lookahead_distance_)
            {
                lookahead_idx = i;
                break;
            }
        }

        const auto &lookahead = plan_.poses[lookahead_idx].pose.position;
        double err_x_world = lookahead.x - drone_x;
        double err_y_world = lookahead.y - drone_y;
        double err_z_world = lookahead.z - drone_z;

        double vx_world = kp_xy_ * err_x_world;
        double vy_world = kp_xy_ * err_y_world;
        double vz_world = kp_z_ * err_z_world;

        double xy_speed = std::hypot(vx_world, vy_world);
        if (xy_speed > max_xy_vel_ && xy_speed > ee4308::THRES)
        {
            double scale = max_xy_vel_ / xy_speed;
            vx_world *= scale;
            vy_world *= scale;
        }

        vz_world = std::clamp(vz_world, -max_z_vel_, max_z_vel_);

        double cos_yaw = std::cos(drone_yaw);
        double sin_yaw = std::sin(drone_yaw);
        double vx_body = cos_yaw * vx_world + sin_yaw * vy_world;
        double vy_body = -sin_yaw * vx_world + cos_yaw * vy_world;

        publishCmdVel_(vx_body, vy_body, vz_world, yaw_vel_);
    }

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