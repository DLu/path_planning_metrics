#include<local_planner_simulator/local_planner_simulator.h>
#include<ros/ros.h>
#include<nav_msgs/Path.h>
#include<nav_msgs/OccupancyGrid.h>

class LPS{
    public:
        LPS(){
            ros::NodeHandle n;

            lps = new LocalPlannerSimulator(tf);

            grid_sub = n.subscribe("costmap", 1, &LPS::gridCallback, this);
            path_sub = n.subscribe("/global_plan", 1, &LPS::pathCallback, this);
            lps->plan();
        }

    private: 

        void gridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg){
            lps->setCostmap(msg);
            lps->plan();
        }

        void pathCallback(const nav_msgs::Path::ConstPtr& msg){
            ROS_INFO("GOT PATH");
            lps->setPath(msg);
        }

        tf::TransformListener tf;
        LocalPlannerSimulator* lps;
        ros::Subscriber grid_sub, path_sub;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lps_tester");
  LPS* lpsa = new LPS();
  ros::spin();  
  delete lpsa;
  return 0;
}
