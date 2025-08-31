#include <pcl/kdtree/kdtree.h>
#include <chrono>
#include <numeric>
#include <vector>
#include <pcl/impl/point_types.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "pcl/io/pcd_io.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
namespace tdt_radar {
class Cluster : public rclcpp::Node {
public:
    Cluster(const rclcpp::NodeOptions& node_options);
    ~Cluster() {}

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    pub_;
    void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> accumulated_clouds_;
};
}  // namespace tdt_radar
