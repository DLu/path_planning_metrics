#include<ros/ros.h>
#include<local_planner_simulator/local_planner_simulator.h>

LocalPlannerSimulator::LocalPlannerSimulator(tf::TransformListener& tf) : tf_(tf),
     controller_costmap_ros_(NULL),
    blp_loader_("nav_core", "nav_core::BaseLocalPlanner") {
    ros::NodeHandle private_nh("~");
    
    std::string local_planner;
    private_nh.param("base_local_planner", local_planner, std::string("base_local_planner/TrajectoryPlannerROS"));
    controller_costmap_ros_ = new costmap_2d::Costmap2DROS("local_costmap", tf_);
    
        //create a local planner
    try {
      //check if a non fully qualified name has potentially been passed in
      if(!blp_loader_.isClassAvailable(local_planner)){
        std::vector<std::string> classes = blp_loader_.getDeclaredClasses();
        for(unsigned int i = 0; i < classes.size(); ++i){
          if(local_planner == blp_loader_.getName(classes[i])){
            //if we've found a match... we'll get the fully qualified name and break out of the loop
            ROS_WARN("Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                local_planner.c_str(), classes[i].c_str());
            local_planner = classes[i];
            break;
          }
        }
      }

      tc_ = blp_loader_.createInstance(local_planner);
      ROS_INFO("Created local_planner %s", local_planner.c_str());
      tc_->initialize(blp_loader_.getName(local_planner), &tf_, controller_costmap_ros_);
    } catch (const pluginlib::PluginlibException& ex)
    {
      ROS_FATAL("Failed to create the %s planner, are you sure it is properly registered and that the containing library is built? Exception: %s", local_planner.c_str(), ex.what());
      exit(1);
    }
}

        
void LocalPlannerSimulator::setCostmap(const nav_msgs::OccupancyGrid::ConstPtr& grid)
{
  controller_costmap_ros_->getCostmap()->resizeMap(grid->info.width, grid->info.height, grid->info.resolution, 
                                      grid->info.origin.position.x, grid->info.origin.position.y);
    for(unsigned int j=0;j<grid->info.height;j++){
        for(unsigned int i=0;i<grid->info.width;i++){
            int index = j * grid->info.width + i;
            controller_costmap_ros_->getCostmap()->setCost(i,j, grid->data[index]);
        }
    }
}

void LocalPlannerSimulator::setPath(const nav_msgs::Path::ConstPtr& msg)
{
    tc_->setPlan(msg->poses);
}

void LocalPlannerSimulator::plan()
{
    geometry_msgs::Twist cmd_vel;
    tc_->computeVelocityCommands(cmd_vel);
    ROS_INFO("COMMAND: %f %f : %f", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
    
}
    
