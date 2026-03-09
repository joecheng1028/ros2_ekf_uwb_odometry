#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include <Eigen/Dense>
#include <cmath>

class EkfNode : public rclcpp::Node
{
public:
    EkfNode() : Node("ekf_node")
    {
        x_ = Eigen::Vector3d::Zero();                    // zero state vector
        P_ = Eigen::Matrix3d::Zero();                    // zero 3x3 matrix
        P_(0,0) = 1.0;
        P_(1,1) = 1.0;
        P_(2,2) = 0.1;
        Q_ = Eigen::Matrix3d::Identity() * 0.01;

        H_ = Eigen::MatrixXd::Zero(2,3);                    // 2x3 zero matrix
        H_(0,0) = 1.0;
        H_(1,1) = 1.0;

        sigma2_base_ = 0.01;
        initialised_ = false;
        uwb_origin_set_ = false;
        uwb_origin_x_ = 0.0;
        uwb_origin_y_ = 0.0;

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odometry/filtered", 10,
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                odom_callback(msg);
            });

        uwb_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/uwb_pose", 10,
            [this](const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
                uwb_callback(msg);
            });

        publisher_ = create_publisher<geometry_msgs::msg::PoseStamped>(
            "/ekf_pose", 10);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        if (!initialised_) {
            last_stamp_ = rclcpp::Time(msg->header.stamp);       // construct rclcpp::Time
            initialised_ = true;
            return;
        }

        rclcpp::Time current_stamp(msg->header.stamp);
        double dt = (current_stamp - last_stamp_).seconds();     // dt in seconds
        last_stamp_ = current_stamp;

        if (dt <= 0.0) return;

        double v     = -msg->twist.twist.linear.x;              // linear velocity (negated for UWB frame)
        double omega = -msg->twist.twist.angular.z;             // angular velocity (negated for UWB frame)
        double theta = x_(2);

        x_(0) += v * cos(theta) * dt;                           // motion model px
        x_(1) += v * sin(theta) * dt;                           // motion model py
        x_(2) += omega * dt;                                    // motion model theta

        Eigen::Matrix3d F = Eigen::Matrix3d::Identity();
        F(0,2) = -v * sin(theta) * dt;                          // Jacobian entry
        F(1,2) = v * cos(theta) * dt;                           // Jacobian entry

        P_ = F * P_ * F.transpose() + Q_;                       // covariance prediction

        publish_pose(msg->header.stamp);
    }

    void uwb_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if (!initialised_) return;

        double ux = msg->pose.pose.position.x / 10.0;   // raw dm → m
        double uy = msg->pose.pose.position.y / 10.0;

        // Store first UWB measurement as origin — subtract from all subsequent
        if (!uwb_origin_set_) {
            uwb_origin_x_ = ux;
            uwb_origin_y_ = uy;
            uwb_origin_set_ = true;
        }

        Eigen::Vector2d z;
        z(0) = ux - uwb_origin_x_;                      // UWB x origin-shifted
        z(1) = uy - uwb_origin_y_;                      // UWB y origin-shifted

        double PDOP = 1.0;
        Eigen::Matrix2d R =
            Eigen::Matrix2d::Identity() * sigma2_base_ * PDOP * PDOP;     // R matrix

        Eigen::Vector2d y = z - H_ * x_;                          // innovation
        Eigen::Matrix2d S = H_ * P_ * H_.transpose() + R;         // innovation covariance
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();    // Kalman gain

        x_ = x_ + K * y;                                          // corrected state
        Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        P_ = (I - K * H_) * P_;                                   // corrected covariance

        publish_pose(msg->header.stamp);
    }

    void publish_pose(const builtin_interfaces::msg::Time & stamp)
    {
        geometry_msgs::msg::PoseStamped out;
        out.header.stamp    = stamp;
        out.header.frame_id = "map";
        out.pose.position.x = x_(0);
        out.pose.position.y = x_(1);
        out.pose.position.z = 0.0;
        publisher_->publish(out);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr         odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr  uwb_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr     publisher_;

    Eigen::Vector3d  x_;
    Eigen::Matrix3d  P_;
    Eigen::Matrix3d  Q_;
    Eigen::MatrixXd  H_;
    double           sigma2_base_;

    rclcpp::Time last_stamp_;
    bool         initialised_;
    bool         uwb_origin_set_;
    double       uwb_origin_x_;
    double       uwb_origin_y_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<EkfNode>());
    rclcpp::shutdown();
    return 0;
}
