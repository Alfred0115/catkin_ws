#ifndef DOCKING_PATH_GENERATOR_H_
#define DOCKING_PATH_GENERATOR_H_

#include <nav_core/base_local_planner.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <docking_path_generator/track_path.h>

namespace docking_path_generator
{
    struct Euler_angle
    {
        double roll;
        double pitch;
        double yaw;
    };
    class DockingPathGenerator
    {
    public:
        DockingPathGenerator();
        void init();
        void isGoalReached();
        void final_dock(const float final_dock_dist);
        bool turnAngle(const float yaw_angle, const float &angular_speed);
        void mainProcess(geometry_msgs::Twist &cmd_vel);
        bool ready_docking;
        
        ros::Time current_time;
        std::vector<geometry_msgs::PoseStamped> bezier_docking_path;
        ros::Publisher vel_pub;
        ~DockingPathGenerator();

    private:
        geometry_msgs::PoseStamped cam2base(geometry_msgs::PoseStamped* source_pose, std::string target_frame);
        Euler_angle getEulerAngle(geometry_msgs::Pose &msg);
        void tag_pose_cb(const apriltag_ros::AprilTagDetectionArray::ConstPtr &tag_msgs);
        void docking_prepared_cb(const std_msgs::Bool &msg);
        bool rotateAdjustment(geometry_msgs::Twist& cmd_vel, double yaw, bool &flag, const double &stop_threshold);
        double angle_remaping(double theta);
        void odom_cb(const nav_msgs::Odometry &odom_msgs);
        bool goStraightLine(geometry_msgs::Pose &start_pose, geometry_msgs::Twist &cmd_vel, const double distance);
        geometry_msgs::Pose cpPose(geometry_msgs::Pose &robot_pose);
        ros::Subscriber tag_msgs_sub_;
        ros::Subscriber docking_prepared_sub_;
        ros::Subscriber odom_sub;
        std::string tag_topic, docking_topic, robot_frame, goal_frame, odom_topic;
        bool docking_prepared;
        tf::TransformListener tf;
        geometry_msgs::PoseStamped goal_local_pose;
        double efficient_docking_dis;
        bool allowable_distance, legal_tag_info, robot_ready;
        double rx, ry, rt, rqz, rqw;
        ros::Publisher path_pub;
        std::vector<geometry_msgs::PoseStamped> docking_path;
        std::vector<geometry_msgs::PoseStamped> local_path;
    
        geometry_msgs::PoseStamped robot_global_pose, goal_global_pose;

        double xy_goal_tolerance_;
        bool goal_reached_;
        double rotation_accuracy_ ;
        std::string global_frame_;
        double max_moveable_dis_;
        double max_linear_vel_;
        double sim_time_;
        double max_rotation_vel_ ; //0.6
        double min_rotation_vel_;
        double max_rotate_angle_ ;
        double cmd_vel_linear_x_;
        double cmd_vel_angular_z_;
        double acceleration_z_ ;
        double acceleration_x_ ;
        double slow_down_factor_;
        bool rotate_to_global_plan_ = false;
        std::vector<geometry_msgs::PoseStamped> transformed_global_plan_;
        double cmd_vel_angular_z_rotate_;

    };
}
#endif