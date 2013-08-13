#ifndef LOCAL_PLANNER_SIMULATOR_H_
#define LOCAL_PLANNER_SIMULATOR_H_

#include <vector>
#include <string>

#include <ros/ros.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <pluginlib/class_loader.h>

  class LocalPlannerSimulator {
    public:
      LocalPlannerSimulator(tf::TransformListener& tf);
      void setCostmap(nav_msgs::OccupancyGrid& grid); 

    private:
      tf::TransformListener& tf_;
      boost::shared_ptr<nav_core::BaseLocalPlanner> tc_;
      costmap_2d::Costmap2DROS* controller_costmap_ros_;

      pluginlib::ClassLoader<nav_core::BaseLocalPlanner> blp_loader_;
  };

#endif

