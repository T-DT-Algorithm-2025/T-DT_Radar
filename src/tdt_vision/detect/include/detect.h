#ifndef RADAR_DETECT_H
#define RADAR_DETECT_H

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "classify.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "NvidiaInterface.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "vision_interface/msg/detect_result.hpp"
#include "yolos.hpp"
#include "BaseInfer.hpp"
#include <fstream>
namespace tdt_radar {

class Detect final : public rclcpp::Node {
public:
    explicit Detect(const rclcpp::NodeOptions& options);
    void callback(const std::shared_ptr<sensor_msgs::msg::Image> msg);
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub;
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr
        compressed_image_sub;

private:
    std::shared_ptr<Infer<yolo::BoxArray>>     yolo;
    std::shared_ptr<Infer<yolo::BoxArray>>     armor_yolo;
    std::shared_ptr<Infer<int>> classifier;
    rclcpp::Publisher<vision_interface::msg::DetectResult>::SharedPtr pub;

    bool        if_rosbag = false;
    int         EnemyColor;
    int         debug;
    std::string yolo_path;
    std::string armor_path;
    std::string classify_path;
};
class Car {
public:
    cv::Rect       car_rect;
    yolo::Box      car;
    yolo::BoxArray armors;
    cv::Point2f    center;
    cv::Rect       center_rect;
    int            number = 0;
    int            color = 1;
};
}  // namespace tdt_radar

#endif
