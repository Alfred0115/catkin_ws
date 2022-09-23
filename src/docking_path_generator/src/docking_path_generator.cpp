#include <iostream>
#include <ros/ros.h>
#include <signal.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>
#include <math.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <apriltag_ros/AprilTagDetectionArray.h>
#include <std_msgs/Float64.h>
#include <amr_msgs_srv/track_tag.h>
#include <amr_msgs_srv/dock_accurancy.h>


struct Euler_angle
{
    double roll;
    double pitch;
    double yaw;
};

class docking_planner
{
private:
    ros::NodeHandle nh_;
    ros::Subscriber pos_sub_;
    ros::Subscriber robot_sub_;
    ros::Subscriber dock_prepared_sub_;
    ros::Publisher cmd_pub_;
    ros::Publisher path_pub_;
    ros::Publisher goal_pub_;
    ros::Publisher docking_state;
    ros::Publisher docking_accurancy;
    geometry_msgs::PoseStamped poseGoal, last_msg;
    geometry_msgs::Twist cmd_vel;
    pcl::PointCloud<pcl::PointXYZ> path;
    tf::TransformListener listener;
    // kalman_filter filter;
    std::string goal_frame, robot_frame, fixed_frame, goal_topic, robot_topic, output_vel_topic, docking_path_topic;
    double vel;
    int ind;
    double refresh;          // counter for number of loop passed
    bool robot_ready;        // check if robot odometry is received
    bool goal_ready;         // check if goal data/ camera data is received or not
    bool final_adjust;       // state when the robot approaching the goal
    bool goal_reach;         // state when the robot reached the goal
    bool callback_firstloop; // check if the system runs on the first loop or not (for initializing purpose)
    double offset;           // offset according to robot+station size
    double rx, ry, rt;       // robot x,y,theta
    double gx, gy, gt;       // goal x,y, theta
    double tx, ty, tt;       // required x,y,theta to reach lookahead point
    double rx_old, ry_old, rt_old;
    double lh_dist; // lookahead distance
    double k, k1, k2, k3;       // turning coefficient转动系数
    double max_angular_speed;
    int point_num;  // number of point on the path
    double tag_dis_;
    double detected_angle_error = 0.05;
    tf::TransformListener tfListener;

    bool isCopyFinalErrors = false;
    bool isCopyErrors = false;
    bool adjust_yaw = false;
    double angle;
    double distance;
    ros::Time start_time;
    ros::Time lost_track_start_time;
    bool isCopyTrackLostTime = false;
    double end_time;
    double final_dock_dis;
    int counts;
    bool ifFianl_dock = false;
    bool goForward = false;
    bool goBackward = false;
    bool isSleep = false;
    double cp_angular_z;
    unsigned int tag_id;
    double stop_distance, yaw_deviation, y_deviation, yaw_accurancy;
    bool final_angle_adjustment = false;

public:
    docking_planner() : // Constructor
                        nh_("~"),
                        ind(0),
                        refresh(0),
                        final_adjust(false),
                        callback_firstloop(true),
                        offset(0.05),
                        point_num(1000),
                        goal_reach(false),
                        goal_ready(false)
    {
        nh_.param<double>("velocity", vel, -0.12);
        nh_.param<double>("k1", k1, 0.3); // 0.3
        nh_.param<double>("k2", k2, 0.3);
        nh_.param<double>("k3", k3, 0.3);
        nh_.param<double>("max_angular_speed", max_angular_speed, 0.15);
        nh_.param<double>("lookahead_distance", lh_dist, 0.1);
        nh_.param<std::string>("goal_frame", goal_frame, "goal");
        nh_.param<std::string>("robot_frame", robot_frame, "base_link");
        nh_.param<std::string>("fixed_frame", fixed_frame, "odom");
        nh_.param<std::string>("goal_topic", goal_topic, "/tag_detections");
        nh_.param<std::string>("robot_topic", robot_topic, "/odom_ekf");
        nh_.param<std::string>("output_vel_topic", output_vel_topic, "/amr/cmd_vel_ctrl");
        nh_.param<std::string>("docking_path_topic", docking_path_topic, "/docking_path");
        nh_.param<double>("tag_distance", tag_dis_, 0.0); // 0.8
        nh_.param<double>("stop_distance", stop_distance, 0.75);
        nh_.param<double>("yaw_deviation", yaw_deviation, 0.0);
        nh_.param<double>("yaw_accurancy", yaw_accurancy, 0.005);
        nh_.param<double>("y_deviation" , y_deviation, 0.0);

        pos_sub_ = nh_.subscribe(goal_topic, 1, &docking_planner::goalPose, this);
        robot_sub_ = nh_.subscribe(robot_topic, 1, &docking_planner::robotPose, this); // Sub to robot pose or odom?
        dock_prepared_sub_ = nh_.subscribe("/docking_prepared", 1, &docking_planner::dock_prepared_cb, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>(output_vel_topic, 1);
        path_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(docking_path_topic, 1);
        goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/docking_goal", 1);
        docking_state = nh_.advertise<std_msgs::Bool>("/docking_state", 1);
        docking_accurancy = nh_.advertise<amr_msgs_srv::dock_accurancy>("/docking_accurancy", 1);

    }
    void goalPose(const apriltag_ros::AprilTagDetectionArray::ConstPtr &tag_msgs);
    void robotPose(const nav_msgs::Odometry::ConstPtr &msg);
    bool init();
    void mainProcess();
    // void pathGenerate();
    void purePursuit();
    void stop();
    Euler_angle getEulerAngle(geometry_msgs::Pose &msg);
    geometry_msgs::PoseStamped cam2base(geometry_msgs::PoseStamped *source_pose, std::string target_frame);
    bool rotateAdjustment(geometry_msgs::Twist &cmd_vel, double yaw, bool &flag, const double &stop_threshold);
    double angle_remaping(double theta);
    void copyErrors(double &tx, double &ty, double &tt, double &rx_old_, double &ry_old_, double &rt_old_);
    bool goStraightLine(geometry_msgs::Twist &cmd_vel, const double distance, const int plus_minus, const double &stop_threshold);
    void dock_prepared_cb(const amr_msgs_srv::track_tag &msg);
    void pathGenerate();
    bool dock_prepared_;

    ~docking_planner();
};

void docking_planner::dock_prepared_cb(const amr_msgs_srv::track_tag &msg)
{
    if (msg.docking_prepareation_completed)
    {
        dock_prepared_ = true;
        ifFianl_dock = false;
        goForward = false;
        goBackward = false;
        isSleep = false;
    }
    else
    {
        dock_prepared_ = false;
        ifFianl_dock = false;
        goForward = false;
        goBackward = false;
        isSleep = false;
    }

    tag_id = msg.tag_id;

    return;
}

void docking_planner::goalPose(const apriltag_ros::AprilTagDetectionArray::ConstPtr &tag_msgs) // Callback to visp auto tracker's object position
{
    if (tag_msgs.get()->detections.size() <= 0)
    {
        // ROS_INFO("Receive no tags info..., tracking lost");
        goal_ready = false;
        cmd_vel.linear.x = 0;
        // cmd_vel.angular.z = 0;

        // final_dock(0.1);
        return;
    }

    int size = tag_msgs.get()->detections.size();
    for (int i = 0; i < size; i++)
    {
        if (tag_msgs.get()->detections[i].id[0] == tag_id)
        {

            // goal pose transform from cam frame to robot frame
            geometry_msgs::PoseStamped cam_goal_pose, robot_goal_pose;

            int size = tag_msgs.get()->detections.size();
            // printf("detection:  %d ...\n", tag_msgs->detections[size - 1].id.size());
            double distance = sqrt(pow(tag_msgs.get()->detections[size - 1].pose.pose.pose.position.z, 2) +
                                   pow(tag_msgs.get()->detections[size - 1].pose.pose.pose.position.x, 2));

            cam_goal_pose.header = tag_msgs.get()->detections[size - 1].pose.header;
            cam_goal_pose.pose = tag_msgs.get()->detections[size - 1].pose.pose.pose;
            // printf("tag pose x: %f, z: %f \n", cam_goal_pose.pose.position.x, cam_goal_pose.pose.position.z);

            geometry_msgs::Pose p = cam_goal_pose.pose;
            Euler_angle e = getEulerAngle(p);
            double pitch = e.pitch;
            double roll = e.roll;
            double yaw = e.yaw;
            // final goal
            // double tag_dis = control_point_dis[count]; // 0.4
            double tag_dis = tag_dis_; //最接近tag设置0.35

            double delta_z = cos(pitch) * tag_dis;
            double delta_x = sin(pitch) * tag_dis;

            // printf("delta_x: %f, delta_y: %f\n", delta_x, delta_y);

            geometry_msgs::PoseStamped tag_goal, previous_goal;

            tag_goal.header.frame_id = tag_msgs.get()->header.frame_id;
            tag_goal.header.stamp = ros::Time::now();
            tag_goal.pose.position.x = cam_goal_pose.pose.position.x - delta_x;
            tag_goal.pose.position.y = 0;
            tag_goal.pose.position.z = cam_goal_pose.pose.position.z - delta_z;

            // let goal rotate to tag
            geometry_msgs::Quaternion q;
            q = tf::createQuaternionMsgFromRollPitchYaw(roll, (pitch - M_PI_2 + yaw_deviation), yaw); // 0.07    //adjust yaw
            // q = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch + 3.14, yaw);
            tag_goal.pose.orientation = q;

            geometry_msgs::PoseStamped goal_local_pose = cam2base(&tag_goal, robot_frame);

            goal_local_pose.pose.position.y = goal_local_pose.pose.position.y + y_deviation; // adjust y
            
            goal_pub_.publish(goal_local_pose); // publish goal pose
            // previous_goal_pub.publish(previous_goal_pose);

            // geometry_msgs::PoseStamped s;
            // s.header.stamp = ros::Time::now();
            // s.header.frame_id = "base_link";
            // s.pose.position.x = 0;
            // s.pose.position.y = 0;
            // s.pose.position.z = 0;
            // geometry_msgs::Quaternion qu;
            // qu = tf::createQuaternionMsgFromRollPitchYaw(0, 0, 0);
            // s.pose.orientation = qu;

            // robot_pose_pub.publish(s);

            double angle = tf::getYaw(goal_local_pose.pose.orientation);
            // double current_th = tf::getYaw(s.pose.orientation);

            // printf("angle: %f, current_th: %f \n", angle, current_th);   //current_th = 0

            double xy_diff = sqrt(pow(goal_local_pose.pose.position.x, 2) + pow(goal_local_pose.pose.position.y, 2));
            // float yaw_diff_ = angles::shortest_angular_distance(current_th, angle);
            double yaw_diff = angle;
            // double yaw_angle = atan2(goal_local_pose.pose.position.y, goal_local_pose.pose.position.x);
            double y_dis = goal_local_pose.pose.position.y;
            double x_dis = goal_local_pose.pose.position.x;

            // if (y_dis > 0)
            // {
            //     gt = yaw_diff;
            // }
            // else
            // {
            //     gt = yaw_diff; //-0.008
            // }
            gt = yaw_diff;
            gx = x_dis;
            gy = y_dis; // adjust y -0.001

            if (gx == 0 && gy == 0 && gt == 0)
            {
                goal_ready = false;
                return;
            }

            goal_ready = true;
        }
    }

    return;
}

geometry_msgs::PoseStamped docking_planner::cam2base(geometry_msgs::PoseStamped *source_pose, std::string target_frame)
{
    geometry_msgs::PoseStamped robot_goal_pose;
    source_pose->header.stamp = ros::Time::now();
    try
    {
        tfListener.waitForTransform(target_frame, source_pose->header.frame_id,
                                    ros::Time::now(), ros::Duration(3));
        // ros::Time(0)
        tfListener.transformPose(target_frame, *source_pose, robot_goal_pose);
        robot_goal_pose.pose.position.z = 0;
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("Transform exception: %s", ex.what());
    }
    return robot_goal_pose;
}

void docking_planner::robotPose(const nav_msgs::Odometry::ConstPtr &msg)
{
    rx = msg->pose.pose.position.x;
    ry = msg->pose.pose.position.y;
    geometry_msgs::Pose temp = msg->pose.pose;

    Euler_angle e;
    e = getEulerAngle(temp);
    rt = e.yaw;
    robot_ready = true;
}

Euler_angle docking_planner::getEulerAngle(geometry_msgs::Pose &msg)
{
    tf::Quaternion q(
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    Euler_angle e;
    e.roll = roll;
    e.pitch = pitch;
    e.yaw = yaw;
    return e;
}

bool docking_planner::init() // First setup
{
    if (goal_ready && robot_ready)
    {
        std::cout << "Initialize" << std::endl;
        pathGenerate(); // Generate the Path
    }
    return goal_ready && robot_ready;
}

void docking_planner::pathGenerate()
{
    // printf("goal gx: %f, gy: %f, gt: %f \n", gx, gy, gt);
    // initialize path
    path.header.frame_id = fixed_frame;
    pcl_conversions::toPCL(ros::Time::now(), path.header.stamp);

    // initialize temporary path
    pcl::PointCloud<pcl::PointXYZ> temp;
    temp.header.frame_id = robot_frame;
    pcl_conversions::toPCL(ros::Time::now(), temp.header.stamp);

    // Cubic Bezier Curve
    temp.points.resize(point_num); // 1000
    path.points.resize(point_num);
    double x0, x1, x2, x3;
    double y0, y1, y2, y3;
    double x0_r, x1_r, x2_r, x3_r;
    double y0_r, y1_r, y2_r, y3_r;
    // In rotated frame
    x3_r = cos(gt) * gx + sin(gt) * gy + lh_dist;
    y3_r = -sin(gt) * gx + cos(gt) * gy;

    x2_r = x3_r * 0.5;
    y2_r = y3_r;

    x1_r = x3_r * 0.2; // control point 1
    y1_r = y3_r * 0.8;

    x0_r = 0; // start point
    y0_r = 0;

    // Convert to robot frame w/o rotation
    x3 = cos(gt) * x3_r - sin(gt) * y3_r;
    y3 = sin(gt) * x3_r + cos(gt) * y3_r;

    x2 = cos(gt) * x2_r - sin(gt) * y2_r;
    y2 = sin(gt) * x2_r + cos(gt) * y2_r;

    x1 = cos(gt) * x1_r - sin(gt) * y1_r;
    y1 = sin(gt) * x1_r + cos(gt) * y1_r;

    x0 = 0;
    y0 = 0;

    double d_PathPoint = 1.0 / (point_num - 1);
    double t = 0.0;
    for (int i = 0; i < point_num; i++)
    {
        temp.points[i].x = x0 + 3 * t * (x1 - x0) + 3 * t * t * (x0 + x2 - 2 * x1) + t * t * t * (x3 - x0 + 3 * x1 - 3 * x2);
        temp.points[i].y = y0 + 3 * t * (y1 - y0) + 3 * t * t * (y0 + y2 - 2 * y1) + t * t * t * (y3 - y0 + 3 * y1 - 3 * y2);
        temp.points[i].z = 0;
        path.points[i].x = cos(rt) * temp.points[i].x - sin(rt) * temp.points[i].y + rx;
        path.points[i].y = sin(rt) * temp.points[i].x + cos(rt) * temp.points[i].y + ry;
        path.points[i].z = 0;
        t = t + d_PathPoint;
        // printf("x: %f, y: %f \n", path.points[i].x, path.points[i].y);
    }
    ind = 0;

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(path, output);

    path_pub_.publish(output);
    return;
}

void docking_planner::mainProcess() // Main Process, will never come here before initialization
{
    // printf("goal gx: %f, gy: %f, gt: %f \n", gx, gy, gt);
    // printf("ifFinal_dock: %d\n", ifFianl_dock);
    //return;    //标定二维码时，直接return　否则就把return注掉
    
    bool D_state = true;
    docking_state.publish(D_state);
    
    if (ifFianl_dock) //是否进行最终的对接，前进，后退
    {
        final_angle_adjustment = false;  //终点旋转标志位恢复
        final_adjust = false;
        //printf("goal gx: %f, gy: %f, gt: %f \n", gx, gy, gt);
        
        //printf("goforward: %d, sleep: %d,gobackward: %d \n", goForward, isSleep, goBackward);
        if (!goBackward)
        {
            if (!isCopyErrors)
            {
                copyErrors(tx, ty, tt, rx_old, ry_old, rt_old);
                isCopyErrors = true;
            }
            double dis = sqrt(pow(rx - rx_old, 2) + pow(ry - ry_old, 2));
            double target_dis = 0.25;
            //printf("robot go %f meters\n", dis);

            if (goStraightLine(cmd_vel, (target_dis - dis), -1, 0.01))
            {
                goBackward = true;
                isCopyErrors = false;
                // sleep(5);
            }
            cmd_pub_.publish(cmd_vel);
            return;
        }

        if (!isSleep)
        {
            if (!isCopyErrors)
            {
                start_time = ros::Time::now();
                isCopyErrors = true;
            }

            double T = ros::Time::now().toSec() - start_time.toSec();
            //printf("stop time: %f\n", T);
            if (T < 5)
            {
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
            }
            else
            {
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
                isSleep = true;
                isCopyErrors = false;
            }
            cmd_pub_.publish(cmd_vel);
            return;
        }

        if (!goForward)
        {
            if (!isCopyErrors)
            {
                copyErrors(tx, ty, tt, rx_old, ry_old, rt_old);
                isCopyErrors = true;
            }
            double dis = sqrt(pow(rx - rx_old, 2) + pow(ry - ry_old, 2));
            double target_dis = 0.5;   //0.25

            if (goStraightLine(cmd_vel, (target_dis - dis), 1, 0.01))
            {
                goForward = true;
                isCopyErrors = false;
                // ifFianl_dock = false;
                goal_reach = false;
                dock_prepared_ = false;
                D_state = false;
                docking_state.publish(D_state);
            }
            cmd_pub_.publish(cmd_vel);
            return;
        }
        
        return;
        
    }
    

    if (!goal_ready) //运动过程中二维码丢失
    {
        if (!isCopyTrackLostTime)
        {
            lost_track_start_time = ros::Time::now();
            isCopyTrackLostTime = true;
            cp_angular_z = cmd_vel.angular.z;
            
        }

        double track_lost_time = ros::Time::now().toSec() - lost_track_start_time.toSec();

        cmd_vel.linear.x = 0;
        if (track_lost_time > 1) 
        {
            cmd_vel.angular.z = 0.0;
            dock_prepared_ = false;
            //printf("track lost over 1 seconds, please check the tag!\n");
            //cmd_vel.angular.z = cp_angular_z / fabs(cp_angular_z) * 0.1;
        }
        else
        {
            cmd_vel.angular.z = -cp_angular_z * 0.5;
            // if (cp_angular_z > 0)
            // {
            //     cmd_vel.angular.z = -0.05;
            // }
            // else
            // {
            //     cmd_vel.linear.z = 0.05;
            // }
        }

        cmd_pub_.publish(cmd_vel);

        return;
    }

    isCopyTrackLostTime = false;

    // return;
    if (gx > 1.5)
    {
        k = k1;  //0.3
    }
    else if (gx > 1.0)
    {
        k = k2;  //0.45
    }
    else
    {
        k = k3;   //0.5
    }

    //std::cout << "---------k: " << k << "----------\n";
    if (!goal_reach)
    {
        if (gx < 8 * lh_dist)    //3
        {
            final_adjust = true;
        }

        double temp_distance;
        double shortest_distance = 1000;
        double temp_dx, temp_dy;
        double L = 0.0;
        double dx, dy;

        pathGenerate();

        for (int i = 0; i < path.size(); i++)
        {
            temp_dx = path.points[i].x - rx;
            temp_dy = path.points[i].y - ry;
            temp_distance = sqrt(temp_dx * temp_dx + temp_dy * temp_dy);
            if (shortest_distance > temp_distance)
            {
                ind = i;
                shortest_distance = temp_distance;
            }
        }

        while (lh_dist > L && ind < path.size())
        {
            dx = path.points[ind + 1].x - path.points[ind].x;
            dy = path.points[ind + 1].y - path.points[ind].y;
            L += sqrt(dx * dx + dy * dy);
            ind++;
        }

        if (ind >= path.size() || gx <= stop_distance)
        {
            goal_reach = true;
        }
        else
        {
            tx = path.points[ind].x - rx;
            ty = path.points[ind].y - ry;
            // printf("tx: %f, ty: %f\n", tx, ty);
            // printf("atan2: %f, rt: %f\n", atan2(ty, tx), rt);
            tt = atan2(ty, tx) - rt;
        }

        if (!goal_reach)
        {
            if (!final_adjust)
            {
                cmd_vel.linear.x = vel;
                cmd_vel.angular.z = -2 * vel * sin(tt) * (1 / lh_dist) * k;
            }
            if (final_adjust)
            {
                cmd_vel.linear.x = vel / 2;    // 3
                cmd_vel.angular.z = -2 * vel / 2 * sin(tt) * (1 / lh_dist) * k;
            }

            //std::cout << "vel: " << vel << "\n"
            //          << "max_angular_speed: " << max_angular_speed << "\n";

            
            // 防止角速度过大
            double angular_threshold = max_angular_speed;
            if(cmd_vel.angular.z > angular_threshold)
            {
                cmd_vel.angular.z = angular_threshold;
            }
            else if(cmd_vel.angular.z < -angular_threshold)
            {
                cmd_vel.angular.z = -angular_threshold;
            }
            cmd_pub_.publish(cmd_vel);
        }
        else
        {
            // Goal reached, stop the robot
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            std::cout << "Goal reached" << std::endl;
            cmd_pub_.publish(cmd_vel);
        }
    }
    else
    {
        double gt_ = gt + yaw_deviation;
        // Mission already finished
        if (fabs(gt_) <= yaw_accurancy)    //0.005
        {
            //printf("no need to rotate\n");
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            ifFianl_dock = true;
            isCopyErrors = false; //标志位恢复
            amr_msgs_srv::dock_accurancy accurancy;
            accurancy.gy_accurancy = gy;
            accurancy.gt_accurancy = gt_;
            accurancy.angleAdjustment = final_angle_adjustment;
            docking_accurancy.publish(accurancy);
        }
        else
        {
            final_angle_adjustment = true;
            //printf("need to rotate...\n");
            if (!isCopyErrors)
            {
                // printf("start docking...\n");
                isCopyErrors = true;
                copyErrors(tx, ty, tt, rx_old, ry_old, rt_old);
                angle = tt;
                start_time = ros::Time::now();
            }

            end_time = ros::Time::now().toSec() - start_time.toSec();

            //if (!rotateAdjustment(cmd_vel, angle - (rt - rt_old), adjust_yaw, 0.01) || end_time > 20)
            if (!rotateAdjustment(cmd_vel, gt_, adjust_yaw, yaw_accurancy) || end_time > 20)   //0.005 -> 0.003
            {
                adjust_yaw = false;
                printf("adjust %f seconds \n", end_time);
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
                ifFianl_dock = true;
                isCopyErrors = false; //标志位恢复
                amr_msgs_srv::dock_accurancy accurancy;
                accurancy.gy_accurancy = gy;
                accurancy.gt_accurancy = gt_;
                accurancy.angleAdjustment = true;
                docking_accurancy.publish(accurancy);
            }
        }

        cmd_pub_.publish(cmd_vel);
    }
    
    return;
}

bool docking_planner::goStraightLine(geometry_msgs::Twist &cmd_vel, const double distance, const int plus_minus, const double &stop_threshold)
{

    double dis_level = fabs(distance);
    double dis_goal;

    if (dis_level <= 0.1)
    {
        dis_goal = fabs(distance);
        if (dis_goal > stop_threshold)
        {
            if (dis_goal > 0.01)
            {
                cmd_vel.linear.x = dis_goal * plus_minus;
            }
            else
            {
                cmd_vel.linear.x = 0.02;
            }

            cmd_vel.linear.z = 0;
        }
        else
        {
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            // cmd_pub_.publish(cmd_vel);
            return true;
        }
    }
    else
    {
        dis_goal = fabs(distance) - 0.02;
        if (dis_goal > stop_threshold)
        {
            cmd_vel.linear.x = 0.05 * plus_minus;
            cmd_vel.angular.z = 0;
        }
        else
        {
            cmd_vel.linear.x = 0;
            cmd_vel.angular.z = 0;
            // cmd_pub_.publish(cmd_vel);
            return true;
        }
    }

    return false;
}

void docking_planner::copyErrors(double &tx_, double &ty_, double &tt_,
                                 double &rx_old_, double &ry_old_, double &rt_old_)
{
    tx_ = gx;
    ty_ = gy;
    tt_ = gt; //记录当前时刻偏差

    rx_old_ = rx; //记录机器人当前时刻里程计数值
    ry_old_ = ry;
    rt_old_ = rt;

    return;
}

bool docking_planner::rotateAdjustment(geometry_msgs::Twist &cmd_vel, double yaw, bool &flag, const double &stop_threshold)
{
    yaw = angle_remaping(yaw);
    float abs_yaw = fabs(yaw);

    if (abs_yaw > 0.0001 && !flag)
    {
        //printf("yaw diff: %f \n", abs_yaw);
        float vel_tmp = 0.0f;

        if (abs_yaw > 0.1)
        {
            vel_tmp = abs_yaw;
        }
        else if (abs_yaw > fabs(stop_threshold))
        {
            vel_tmp = 0.05; // 0.05
        }
        else
        {
            vel_tmp = 0.0;
            flag = true;
        }
        vel_tmp = vel_tmp > 1.0 ? 1.0 : vel_tmp;

        if (yaw > 0)
            cmd_vel.angular.z = vel_tmp;
        else
            cmd_vel.angular.z = -vel_tmp;

        return true;
    }
    else
    {
        return false;
    }
}

double docking_planner::angle_remaping(double theta)
{
    if (theta >= -M_PI && theta < M_PI)
        return theta;

    double multiplier = std::floor(theta / (2 * M_PI));
    theta = theta - multiplier * 2 * M_PI;
    if (theta >= M_PI)
        theta -= 2 * M_PI;
    if (theta < -M_PI)
        theta += 2 * M_PI;

    return theta;
}

void docking_planner::stop()
{
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_pub_.publish(cmd_vel);
    return;
}

docking_planner *c_docking_planner;

void sig_handler(int sig)
{
    if (c_docking_planner != NULL)
    {
        c_docking_planner->stop();
    }
    ros::shutdown();
}

int main(int argc, char **argv)
{
    std::cout << "Docking Robot..." << std::endl;

    ros::init(argc, argv, "docking_path_genetor", ros::init_options::NoSigintHandler);
    // ros::init(argc, argv, "docking_planner");
    c_docking_planner = new docking_planner();
    signal(SIGINT, sig_handler);

    ros::Rate rate(10);
    ros::Rate init_rate(1);
    std::cout << "Waiting For Docking Station's Pose..." << std::endl;
    bool init_ok = false;

    // initialization loop
    while (!init_ok && ros::ok())
    {
        ros::spinOnce();
        init_ok = c_docking_planner->init();
        init_rate.sleep();
    }

    // main loop
    while (ros::ok())
    {
        ros::spinOnce();

        if (c_docking_planner->dock_prepared_)
        {
            c_docking_planner->mainProcess();
        }
        rate.sleep();
    }
    return 0;
}
