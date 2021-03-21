/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  Copyright (c) 2020, Samsung R&D Institute Russia
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *         Alexey Merzlyakov
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_costmap2d_plugin.html
 *********************************************************************/
#pragma once

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "rostron_interfaces/msg/robots.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "tf2/convert.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
namespace rostron_nav_costmap_plugin
{

  class ObstacleLayer : public nav2_costmap_2d::Layer
  {
  public:
    ObstacleLayer();

    virtual void onInitialize();
    virtual void updateBounds(
        double robot_x, double robot_y, double robot_yaw, double *min_x,
        double *min_y,
        double *max_x,
        double *max_y);
    virtual void updateCosts(
        nav2_costmap_2d::Costmap2D &master_grid,
        int min_i, int min_j, int max_i, int max_j);

    virtual void reset()
    {
      return;
    }

    virtual void onFootprintChanged();

    virtual bool isClearable() { return true; }

    void allies_callback(const rostron_interfaces::msg::Robots::SharedPtr msg);
    void opponents_callback(const rostron_interfaces::msg::Robots::SharedPtr msg);

    double dist(const double x1, const double y1, const double x2, const double y2);

    void processMap(const nav_msgs::msg::OccupancyGrid &new_map);

    /**
   * @brief  Callback to update the costmap's map from the map_server
   * @param new_map The map to put into the costmap. The origin of the new
   * map along with its size will determine what parts of the costmap's
   * static map are overwritten.
   */
    void incomingMap(const nav_msgs::msg::OccupancyGrid::SharedPtr new_map);

  private:
    rclcpp::Subscription<rostron_interfaces::msg::Robots>::SharedPtr pub_allies_;
    rclcpp::Subscription<rostron_interfaces::msg::Robots>::SharedPtr pub_opponents_;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

    rostron_interfaces::msg::Robots allies_;
    rostron_interfaces::msg::Robots opponents_;

    uint32_t robot_id = 0;

    bool map_received_{false};
    tf2::Duration transform_tolerance_;
    std::string global_frame_; ///< @brief The global frame for the costmap
    std::string map_frame_;    /// @brief frame that map is located in
  };

} // namespace rostron_nav_costmap_plugin