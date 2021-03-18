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
    // auto node = node_.lock();
    declareParameter("enabled", rclcpp::ParameterValue(true));
    declareParameter("robot_id", rclcpp::ParameterValue(0.0));

    
    node_->get_parameter(name_ + "." + "enabled", enabled_);
    
    double id;
    node_->get_parameter(name_ + ".robot_id", id);
    robot_id = (uint32_t) id;

    pub_allies_ = node_->create_subscription<rostron_interfaces::msg::Robots>(
        "/yellow/allies",
        10,
        std::bind(&ObstacleLayer::allies_callback, this, std::placeholders::_1));
    pub_opponents_ = node_->create_subscription<rostron_interfaces::msg::Robots>(
        "/yellow/opponents",
        10,
        std::bind(&ObstacleLayer::opponents_callback, this, std::placeholders::_1));
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

    RCLCPP_INFO(rclcpp::get_logger(
                    "nav2_costmap_2d"),
                "robot_id %d",
                robot_id);

    // master_array - is a direct pointer to the resulting master_grid.
    // master_grid - is a resulting costmap combined from all layers.
    // By using this pointer all layers will be overwritten!
    // To work with costmap layer and merge it with other costmap layers,
    // please use costmap_ pointer instead (this is pointer to current
    // costmap layer grid) and then call one of updates methods:
    // - updateWithAddition()
    // - updateWithMax()
    // - updateWithOverwrite()
    // - updateWithTrueOverwrite()
    // In this case using master_array pointer is equal to modifying local costmap_
    // pointer and then calling updateWithTrueOverwrite():
    unsigned char *master_array = master_grid.getCharMap();
    unsigned int size_x = master_grid.getSizeInCellsX(), size_y = master_grid.getSizeInCellsY();

    // {min_i, min_j} - {max_i, max_j} - are update-window coordinates.
    // These variables are used to update the costmap only within this window
    // avoiding the updates of whole area.
    //
    // Fixing window coordinates with map size if necessary.
    // min_i = std::max(0, min_i);
    // min_j = std::max(0, min_j);
    // max_i = std::min(static_cast<int>(size_x), max_i);
    // max_j = std::min(static_cast<int>(size_y), max_j);

    // Simply computing one-by-one cost per each cell
    for (unsigned int j = 0; j < size_y; j++)
    {
      for (unsigned int i = 0; i < size_x; i++)
      {
        int index = master_grid.getIndex(i, j);
        double x, y;
        master_grid.mapToWorld(i, j, x, y);

        // setting the gradient cost
        // master_array[index] = 0;

        for (auto r : allies_.robots)
        {
          if (r.id == robot_id || !r.active)
            continue;

          auto d = dist(x, y, r.pose.position.x, r.pose.position.y);
          if (d - 0.3 < 0.01)
          {
            master_array[index] = LETHAL_OBSTACLE;
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