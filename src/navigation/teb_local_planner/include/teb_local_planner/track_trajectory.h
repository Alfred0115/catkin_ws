#ifndef TEB_LOCAL_PLANNER_TRACK_TRAJECTORY_H_
#define TEB_LOCAL_PLANNER_TRACK_TRAJECTORY_H_

#include <nav_msgs/Odometry.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <tf/transform_listener.h>
#include <Eigen/Core>
#include <dynamic_reconfigure/server.h>
//#include <asr_ftc_local_planner/FTCPlannerConfig.h> //
#include <nav_core/base_local_planner.h>
//#include <ftc_local_planner/transform_global_plan.h> //
//#include <ftc_local_planner/join_costmap.h> //

namespace teb_local_planner
{
    class TrackTrajectory
    {
        public:
            TrackTrajectory();

            /**
             * @brief Given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
             * 
             * @param cmd_vel 
             * @return true 
             * @return false 
             */
            bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);

            bool getRobotGoalPose(const tf::TransformListener& tf, const std::vector<geometry_msgs::PoseStamped>& global_plan,
                     const std::string& robot_frame, tf::Stamped<tf::Pose>& robot_goal_pose, int plan_point);
            
            void initialize(tf::TransformListener *tf, std::vector<geometry_msgs::PoseStamped> &global_plan, 
                            const std::string &global_frame, 
                            const double xy_goal_tolerance,
                            const double rotation_accuracy,
                            const tf::Stamped<tf::Pose> current_pose,
                            const double max_moveable_dis, const double acceleration_z,
                            const double acceleration_x);
            
            std::vector<geometry_msgs::PoseStamped> adjust_path(std::vector<geometry_msgs::PoseStamped> &input);

            bool goal_reached_;
            double angle_to_global_plan;

            ~TrackTrajectory();
        
        private:
            void joinMaps();

            int checkMaxDistance(tf::Stamped<tf::Pose> robot_current_pose);
            double calculateGlobalPlanAngle(tf::Stamped<tf::Pose> current_pose, const std::vector<geometry_msgs::PoseStamped> &plan, int point);
            int driveToward(tf::Stamped<tf::Pose> current_pose, geometry_msgs::Twist &cmd_vel);
            int checkMaxAngle(int points, tf::Stamped<tf::Pose> current_pose);
            bool rotateToOrientation(double angle, geometry_msgs::Twist& cmd_vel, double accuracy);
            int fac(int x);
            
            std::vector<geometry_msgs::PoseStamped> Bezier(double dt, std::vector<geometry_msgs::PoseStamped> &input);
            tf::TransformListener* tf_;
            std::vector<geometry_msgs::PoseStamped> transformed_global_plan_;
            std::vector<geometry_msgs::PoseStamped> global_plan_;
            std::string global_frame_;
            double xy_goal_tolerance_;
            double rotation_accuracy_;
            tf::Stamped<tf::Pose> robot_goal_pose_;      //global frame
            tf::Stamped<tf::Pose> robot_current_pose_;    //global frame
            tf::Stamped<tf::Pose> goal_pose_cp;
            
            double sim_time;
            double max_moveable_dis_;    //max_moveable_dis_ = max_x_vel * sim_time
            double max_rotate_angle_;
            bool rotate_to_global_plan_ = false;
            double cmd_vel_linear_x_;
            double cmd_vel_angular_z_;
            double max_linear_vel;
            
            double max_rotation_vel;
            double min_rotation_vel;
            double acceleration_z_;
            double acceleration_x_;
            double slow_down_factor;
            double cmd_vel_angular_z_rotate_;
            bool stand_at_goal_;
            geometry_msgs::Pose old_goal;
    };
}

#endif