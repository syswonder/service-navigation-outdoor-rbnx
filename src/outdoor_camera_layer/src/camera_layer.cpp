#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <nav2_costmap_2d/layer.hpp>
#include <nav2_costmap_2d/layered_costmap.hpp>
#include <nav2_costmap_2d/cost_values.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_ros/buffer.h>
#include <pluginlib/class_list_macros.hpp>

namespace outdoor_camera_layer
{
/// Merge a bounded current visual grid, preserving all costs from earlier sensor layers.
class CameraLayer : public nav2_costmap_2d::Layer
{
public:
  void onInitialize() override
  {
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("grid_topic", rclcpp::ParameterValue(std::string("/outdoor/vision/filtered_costmap")));
    declareParameter("max_age_s", rclcpp::ParameterValue(1.0));
    auto node = node_.lock();
    std::string topic;
    node->get_parameter(name_ + ".enabled", enabled_);
    node->get_parameter(name_ + ".grid_topic", topic);
    node->get_parameter(name_ + ".max_age_s", max_age_);
    sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(topic, 10,
      [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        const auto & q = msg->info.origin.orientation;
        if (msg->data.size() != static_cast<size_t>(msg->info.width) * msg->info.height ||
            msg->info.resolution <= 0 || msg->header.frame_id.empty() ||
            std::abs(q.x) > 1e-6 || std::abs(q.y) > 1e-6 || std::abs(q.z) > 1e-6 || std::abs(q.w-1) > 1e-6) {return;}
        std::lock_guard<std::mutex> lock(mutex_);
        grid_ = msg;
      });
    current_ = true;
  }

  void updateBounds(double, double, double, double * min_x, double * min_y,
                    double * max_x, double * max_y) override
  {
    auto * master = layered_costmap_->getCostmap();
    // Rebuild the bounded master window even on expiry, so old camera marks retire.
    *min_x = std::min(*min_x, master->getOriginX());
    *min_y = std::min(*min_y, master->getOriginY());
    *max_x = std::max(*max_x, master->getOriginX()+master->getSizeInMetersX());
    *max_y = std::max(*max_y, master->getOriginY()+master->getSizeInMetersY());
  }

  void updateCosts(nav2_costmap_2d::Costmap2D & master, int min_i, int min_j,
                   int max_i, int max_j) override
  {
    if (!enabled_) {return;}
    nav_msgs::msg::OccupancyGrid::SharedPtr grid;
    {std::lock_guard<std::mutex> lock(mutex_); grid = grid_;}
    auto node = node_.lock();
    if (!grid || !node) {return;}
    const double age = (node->now()-rclcpp::Time(grid->header.stamp)).seconds();
    if (age < 0 || age >= max_age_) {return;}
    geometry_msgs::msg::TransformStamped tf;
    try {
      tf = tf_->lookupTransform(grid->header.frame_id, layered_costmap_->getGlobalFrameID(),
                                tf2::TimePointZero);
    } catch (const std::exception &) {return;}
    const auto & q = tf.transform.rotation;
    const double a = 1-2*(q.y*q.y+q.z*q.z), b = 2*(q.x*q.y-q.z*q.w);
    const double c = 2*(q.x*q.y+q.z*q.w), d = 1-2*(q.x*q.x+q.z*q.z);
    for (int y=min_j; y<max_j; ++y) {
      for (int x=min_i; x<max_i; ++x) {
        double wx, wy;
        master.mapToWorld(x,y,wx,wy);
        const double gx = a*wx+b*wy+tf.transform.translation.x;
        const double gy = c*wx+d*wy+tf.transform.translation.y;
        const int ix = std::floor((gx-grid->info.origin.position.x)/grid->info.resolution);
        const int iy = std::floor((gy-grid->info.origin.position.y)/grid->info.resolution);
        if (ix<0 || iy<0 || ix>=static_cast<int>(grid->info.width) || iy>=static_cast<int>(grid->info.height)) {continue;}
        const int value = grid->data[static_cast<size_t>(iy)*grid->info.width+ix];
        if (value<0 || value>100) {continue;}
        const unsigned char cost = value==100 ? nav2_costmap_2d::LETHAL_OBSTACLE :
          static_cast<unsigned char>(std::lround(value*252.0/99.0));
        const auto before = master.getCost(x,y);
        master.setCost(x,y,before==nav2_costmap_2d::NO_INFORMATION ? cost : std::max(before,cost));
      }
    }
  }

  void reset() override {std::lock_guard<std::mutex> lock(mutex_); grid_.reset();}
  bool isClearable() override {return false;}

private:
  /// Latest acquisition-stamped visual cost grid; no persistent occupancy memory.
  nav_msgs::msg::OccupancyGrid::SharedPtr grid_;
  /// Subscription to post-filter original visual geometry.
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  /// Protects grid replacement against the costmap update thread.
  std::mutex mutex_;
  /// Maximum accepted sensor acquisition age in seconds.
  double max_age_{1.0};
};
}
PLUGINLIB_EXPORT_CLASS(outdoor_camera_layer::CameraLayer, nav2_costmap_2d::Layer)
