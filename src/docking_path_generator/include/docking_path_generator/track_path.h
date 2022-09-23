#ifndef DOCKING_PATH_GENERATOR_TRACK_PATH_H_
#define DOCKING_PATH_GENERATOR_TRACK_PATH_H_

#include <nav_msgs/Odometry.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <dynamic_reconfigure/server.h>
//#include <asr_ftc_local_planner/FTCPlannerConfig.h> //
#include <nav_core/base_local_planner.h>
//#include <ftc_local_planner/transform_global_plan.h> //
//#include <ftc_local_planner/join_costmap.h> //

namespace docking_path_generator
{
    class TrackPath
    {
        public:
            TrackPath();
            
            std::vector<geometry_msgs::PoseStamped> adjust_path(std::vector<geometry_msgs::PoseStamped> &input);

            ~TrackPath();
        
        private:
            int fac(int x);
            
            std::vector<geometry_msgs::PoseStamped> Bezier(double dt, std::vector<geometry_msgs::PoseStamped> &input);
            
    };
}

#endif