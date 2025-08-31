#ifndef RADAR_CALIBRATE_H
#define RADAR_CALIBRATE_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <memory>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/photo.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"
#include "radar_utils.h"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace tdt_radar {
static cv::Mat                  cvimage_;
static std::vector<cv::Point2f> pick_points;
static cv::Mat                  camera_matrix;
static cv::Mat                  dist_coeffs;
static cv::Mat                  rvec;
static cv::Mat                  tvec;
static bool                     is_calibrating = false;

class Calibrate final : public rclcpp::Node {
public:
    std::vector<cv::Point3f> real_points;

    cv::Point3f self_FORTRESS = cv::Point3f(5.471, -7.5, 0.0);
    cv::Point3f self_Tower = cv::Point3f(10.936, -11.161, 0.868);
    cv::Point3f enemy_Base = cv::Point3f(25.49, -7.5, 1.24524);
    cv::Point3f enemy_Tower = cv::Point3f(16.925, -3.625, 1.745);
    cv::Point3f enemy_High = cv::Point3f(20.20, -10.8, 0.8);

    explicit Calibrate(const rclcpp::NodeOptions& options);
    void callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void compressed_callback(
        const sensor_msgs::msg::CompressedImage::SharedPtr msg);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
                                                   compressed_image_sub;
    void                                           solve();
    parser*                                        parser_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    geometry_msgs::msg::TransformStamped           transformStamped;
};
void mousecallback(int event, int x, int y, int flags, void* userdata);

}  // namespace tdt_radar
#endif
