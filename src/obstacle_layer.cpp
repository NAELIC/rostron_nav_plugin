#include "rostron_nav_plugin/obstacle_layer.hpp"

#include "nav2_costmap_2d/costmap_math.hpp"
#include "nav2_costmap_2d/footprint.hpp"
#include "rclcpp/parameter_events_filter.hpp"

using nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
using nav2_costmap_2d::LETHAL_OBSTACLE;
using nav2_costmap_2d::NO_INFORMATION;

namespace rostron_nav_costmap_plugin
{

  ObstacleLayer::ObstacleLayer()
  {
  }

  // This method is called at the end of plugin initialization.
  // It contains ROS parameter(s) declaration and initialization
  // of need_recalculation_ variable.
  void
  ObstacleLayer::onInitialize()
  {
    global_frame_ = layered_costmap_->getGlobalFrameID();

    // auto node = node_.lock();
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("robot_id", rclcpp::ParameterValue(0.0));
    declareParameter("map_topic", rclcpp::ParameterValue(""));
    declareParameter("transform_tolerance", rclcpp::ParameterValue(0.0));
    declareParameter("team", rclcpp::ParameterValue(""));

    node_->get_parameter(name_ + "." + "enabled", enabled_);
    node_->get_parameter(name_ + "." + "team", team);

    double id;
    node_->get_parameter(name_ + ".robot_id", id);
    robot_id = (uint32_t)id;
    std::cout << "/" + team  + "/allies" << std::endl;
    pub_allies_ = node_->create_subscription<rostron_interfaces::msg::Robots>(
        "/" + team  + "/allies",
        10,
        std::bind(&ObstacleLayer::allies_callback, this, std::placeholders::_1));
    pub_opponents_ = node_->create_subscription<rostron_interfaces::msg::Robots>(
        "/" + team + "/opponents",
        10,
        std::bind(&ObstacleLayer::opponents_callback, this, std::placeholders::_1));
    double temp_tf_tol = 0.0;
    node_->get_parameter("transform_tolerance", temp_tf_tol);
    transform_tolerance_ = tf2::durationFromSec(temp_tf_tol);
    std::string map_topic;

    node_->get_parameter("map_topic", map_topic);

    rclcpp::QoS map_qos(10); // initialize to default
    map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
        map_topic, map_qos,
        std::bind(&ObstacleLayer::incomingMap, this, std::placeholders::_1));
  }

  // The method is called to ask the plugin: which area of costmap it needs to update.
  // Inside this method window bounds are re-calculated if need_recalculation_ is true
  // and updated independently on its value.
  void
  ObstacleLayer::updateBounds(
      double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/, double *min_x,
      double *min_y, double *max_x, double *max_y)
  {
    double wx, wy;
    layered_costmap_->getCostmap()->mapToWorld(0, 0, wx, wy);
    *min_x = std::min(*min_x, wx);
    *min_y = std::min(*min_y, wy);

    layered_costmap_->getCostmap()->mapToWorld(
        layered_costmap_->getCostmap()->getSizeInCellsX(),
        layered_costmap_->getCostmap()->getSizeInCellsY(),
        wx, wy);
    *max_x = std::max(*max_x, wx);
    *max_y = std::max(*max_y, wy);
  }

  // The method is called when footprint was changed.
  // Here it just resets need_recalculation_ variable.
  void
  ObstacleLayer::onFootprintChanged()
  {
    RCLCPP_DEBUG(rclcpp::get_logger(
                     "nav2_costmap_2d"),
                 "GradientLayer::onFootprintChanged(): num footprint points: %lu",
                 layered_costmap_->getFootprint().size());
  }

  void ObstacleLayer::processMap(const nav_msgs::msg::OccupancyGrid &new_map)
  {
    map_frame_ = new_map.header.frame_id;
  }

  void ObstacleLayer::incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr new_map)
  {
    if (!map_received_)
    {
      map_received_ = true;
      processMap(*new_map);
    }
  }

  // The method is called when costmap recalculation is required.
  // It updates the costmap within its window bounds.
  // Inside this method the costmap gradient is generated and is writing directly
  // to the resulting costmap master_grid without any merging with previous layers.
  void
  ObstacleLayer::updateCosts(
      nav2_costmap_2d::Costmap2D &master_grid, int min_i, int min_j,
      int max_i,
      int max_j)
  {
    if (!enabled_)
      return;

    unsigned char *master_array = master_grid.getCharMap();
    unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

    min_i = std::max(0, min_i);
    min_j = std::max(0, min_j);
    max_i = std::min(static_cast<int>(size_x), max_i);
    max_j = std::min(static_cast<int>(size_y), max_j);

    if (!layered_costmap_->isRolling())
    {
      // Simply computing one-by-one cost per each cell
      for (int j = min_j; j < max_j; j++)
      {
        for (int i = min_i; i < max_i; i++)
        {
          int index = master_grid.getIndex(i, j);
          double x, y;
          master_grid.mapToWorld(i, j, x, y);
          for (auto r : allies_.robots)
          {
            if (r.id == robot_id || !r.active)
              continue;

            auto d = dist(x, y, r.pose.position.x, r.pose.position.y);
            if (d - 0.3 < 0.01)
            {
              master_grid.setCost(i, j, LETHAL_OBSTACLE);
            }
          }

          for (auto r : opponents_.robots)
          {
            if (!r.active)
              continue;

            auto d = dist(x, y, r.pose.position.x, r.pose.position.y);
            if (d - 0.25 < 0.001)
            {
              master_array[index] = LETHAL_OBSTACLE;
            }
          }
        }
      }
    }
    else
    {
      if (!map_received_)
        return;

      geometry_msgs::msg::TransformStamped transform;
      try
      {
        transform = tf_->lookupTransform(
            map_frame_, global_frame_, tf2::TimePointZero,
            transform_tolerance_);
      }
      catch (tf2::TransformException &ex)
      {
        RCLCPP_ERROR(node_->get_logger(), "StaticLayer: %s", ex.what());
        return;
      }

      tf2::Transform tf2_transform;
      tf2::fromMsg(transform.transform, tf2_transform);

      for (int j = min_j; j < max_j; j++)
      {
        for (int i = min_i; i < max_i; i++)
        {
          double x, y;
          master_grid.mapToWorld(i, j, x, y);
          tf2::Vector3 p(x, y, 0);
          p = tf2_transform * p;
        
          for (auto r : allies_.robots)
          {
            if (r.id == robot_id || !r.active)
              continue;

            auto d = dist(p.x(), p.y(), r.pose.position.x, r.pose.position.y);
            if (d - 0.3 < 0.01)
            {
              master_grid.setCost(i, j, LETHAL_OBSTACLE);
            }
          }

          for (auto r : opponents_.robots)
          {
            if (!r.active)
              continue;

            auto d = dist(p.x(), p.y(), r.pose.position.x, r.pose.position.y);
            if (d - 0.25 < 0.001)
            {
              master_grid.setCost(i, j, LETHAL_OBSTACLE);
            }
          }
        }
      }
    }
  }

  void ObstacleLayer::allies_callback(const rostron_interfaces::msg::Robots::SharedPtr msg)
  {
    allies_.set__robots(msg->robots);
  }

  void ObstacleLayer::opponents_callback(const rostron_interfaces::msg::Robots::SharedPtr msg)
  {
    opponents_.set__robots(msg->robots);
  }

  double ObstacleLayer::dist(const double x1, const double y1, const double x2, const double y2)
  {
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
  }

} // namespace nav2_gradient_costmap_plugin

// This is the macro allowing a nav2_gradient_costmap_plugin::GradientLayer class
// to be registered in order to be dynamically loadable of base type nav2_costmap_2d::Layer.
// Usually places in the end of cpp-file where the loadable class written.
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(rostron_nav_costmap_plugin::ObstacleLayer, nav2_costmap_2d::Layer)