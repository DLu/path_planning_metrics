#include<local_planner_simulator/local_planner_simulator.h>
#include<ros/ros.h>
#include<nav_msgs/Path.h>
#include<nav_msgs/OccupancyGrid.h>

class LPS{
    public:
        LPS(){
            ros::NodeHandle n;
            ROS_INFO("CONS");
            lps = new LocalPlannerSimulator(tf);
            grid_sub = n.subscribe("costmap", 1, &LPS::gridCallback, this);
            path_sub = n.subscribe("path", 1, &LPS::pathCallback, this);
            ROS_INFO("SUBD");
            lps->plan();
            ROS_INFO("X");
        }

    private: 

        void gridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
            lps->setCostmap(msg);
        }

        void pathCallback(const nav_msgs::Path::ConstPtr& msg){
            lps->setPath(msg);
        }

        tf::TransformListener tf;
        LocalPlannerSimulator* lps;
        ros::Subscriber grid_sub, path_sub;
};

int main(int argc, char **argv)
{
ROS_INFO("A");
  ros::init(argc, argv, "lps_tester");
ROS_INFO("B");
  LPS* lpsa = new LPS();
ROS_INFO("C");
  ros::spin();  
    return 0;
}
