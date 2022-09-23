#include <ros/ros.h>
#include <teb_local_planner/track_trajectory.h>
#include <angles/angles.h>
//#include <tf/transform_datatypes.h>

namespace teb_local_planner
{
    TrackTrajectory::TrackTrajectory()
    {
    }

    void TrackTrajectory::initialize(tf::TransformListener *tf, std::vector<geometry_msgs::PoseStamped> &global_plan,
                                     const std::string &global_frame,
                                     const double xy_goal_tolerance,
                                     const double rotation_accuracy,
                                     const tf::Stamped<tf::Pose> current_pose,
                                     const double max_moveable_dis, const double acceleration_z,
                                     const double acceleration_x)
    {
        if (global_plan_.size() > 0)
        {
            global_plan_.clear();
        }
        tf_ = tf;
        global_plan_ = global_plan;
        xy_goal_tolerance_ = xy_goal_tolerance;
        //printf("xy_goal_tolerance: %f \n", xy_goal_tolerance_);
        //rotation_accuracy_ = rotation_accuracy;
        rotation_accuracy_ = 3.14;
        global_frame_ = global_frame;
        robot_current_pose_ = current_pose; //  frame is map
        //printf("+++++frame: %s :\n", robot_current_pose_.frame_id_.c_str());
        max_moveable_dis_ = max_moveable_dis;
        max_linear_vel = 0.2; //0.2
        //sim_time = max_moveable_dis_ / max_linear_vel;
        sim_time = 1.0;
        max_rotation_vel = 1.0; //0.6
        min_rotation_vel = 0.1;
        max_rotate_angle_ = max_rotation_vel * sim_time;
        rotate_to_global_plan_ = false;
        cmd_vel_linear_x_ = 0;
        cmd_vel_angular_z_ = 0;
        acceleration_z_ = acceleration_z;
        acceleration_x_ = acceleration_x;
        slow_down_factor = 2.0;
        cmd_vel_angular_z_rotate_ = 0.0;
        //stand_at_goal_ = false;
        goal_reached_ = false;

        //judge if get a new goal
        //printf("old goal x: %f, y: %f, z: %f \n", old_goal.position.x, old_goal.position.y, old_goal.orientation.z);
        if ( sqrt(pow((old_goal.position.x - global_plan_[global_plan_.size() - 1].pose.position.x),2) + 
             pow((old_goal.position.y - global_plan_[global_plan_.size() - 1].pose.position.y),2)) <= xy_goal_tolerance_ )
        {
            ROS_DEBUG("old goal = new goal!");
        }
        else
        {
            ROS_DEBUG("get new goal!");
            old_goal = global_plan_[global_plan_.size() - 1].pose;
            stand_at_goal_ = false;
        }

        //cp goal pose orientation
        getRobotGoalPose(*tf_, global_plan_, global_frame_, goal_pose_cp, global_plan_.size() - 1);

        if (global_plan_.size() > 10)
        {
            global_plan_[global_plan_.size() - 1].pose.orientation = global_plan_[global_plan_.size() - 10].pose.orientation;
        }
        else
        {
            global_plan_[global_plan_.size() - 1].pose.orientation = global_plan_[global_plan_.size() - 2].pose.orientation;
        }

        getRobotGoalPose(*tf_, global_plan_, global_frame_, robot_goal_pose_, global_plan_.size() - 1);
    }

    bool TrackTrajectory::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        ros::Time begin = ros::Time::now();
        //tf::Stamped<tf::Pose> current_pose = robot_current_pose;
        //joinMaps();

        int max_point = 0;
        if (rotate_to_global_plan_)
        {
            //printf("rotate to goal!++++++++++\n");
            double angle_to_global_plan = calculateGlobalPlanAngle(robot_current_pose_, global_plan_, checkMaxDistance(robot_current_pose_));
            rotate_to_global_plan_ = rotateToOrientation(angle_to_global_plan, cmd_vel, 0.1);
        }
        else
        {
            double distance = sqrt(pow((robot_goal_pose_.getOrigin().getX() - robot_current_pose_.getOrigin().getX()), 2) + pow((robot_goal_pose_.getOrigin().getY() - robot_current_pose_.getOrigin().getY()), 2));

            ROS_DEBUG("Goal distance: %f, stand_at_goal: %d \n", distance, stand_at_goal_);

            if (distance > xy_goal_tolerance_ && !stand_at_goal_)
            {
                if (fabs(calculateGlobalPlanAngle(robot_current_pose_, global_plan_, checkMaxDistance(robot_current_pose_)) > 1.2))
                {
                    ROS_INFO("Excessive deviation from global plan orientation. Start routine new.");
                    rotate_to_global_plan_ = true;
                }

                max_point = driveToward(robot_current_pose_, cmd_vel);

                //collision check
                // if (!checkCollision(max_point))
                // {
                //     return false;
                // }
            }
            else
            {
                if (!stand_at_goal_)
                {
                    ROS_INFO("Stand at goal. Rotate to goal orientation.");
                }
                stand_at_goal_ = true;

                angle_to_global_plan = angles::shortest_angular_distance(tf::getYaw(robot_current_pose_.getRotation()),
                                                                         tf::getYaw(goal_pose_cp.getRotation()));
                // if (!rotateToOrientation(angle_to_global_plan, cmd_vel, rotation_accuracy_))
                // {
                //     goal_reached_ = true;
                // }

                goal_reached_ = true;

                cmd_vel.linear.x = 0;
                cmd_vel.angular.z = 0;
                return true;
            }
        }
        // ros::Time end = ros::Time::now();
        // ros::Duration duration = end - begin;
        return true;
    }

    bool TrackTrajectory::getRobotGoalPose(const tf::TransformListener &tf, const std::vector<geometry_msgs::PoseStamped> &global_plan,
                                           const std::string &robot_frame, tf::Stamped<tf::Pose> &robot_goal_pose, int plan_point)
    {
        //printf("goal pose frame: %s \n", robot_frame.c_str());
        if (global_plan.empty())
        {
            ROS_ERROR("Received plan with zero length");
            return false;
        }
        if (plan_point >= (int)global_plan.size())
        {
            ROS_ERROR("Goal function: Plan_point %d to big. Plan size: %lu", plan_point, global_plan.size());
            return false;
        }

        const geometry_msgs::PoseStamped &plan_goal_pose = global_plan.at(plan_point);
        try
        {
            tf::StampedTransform transform;
            tf.waitForTransform(robot_frame, ros::Time::now(),
                                plan_goal_pose.header.frame_id, plan_goal_pose.header.stamp,
                                plan_goal_pose.header.frame_id, ros::Duration(0.5));
            tf.lookupTransform(robot_frame, ros::Time(),
                               plan_goal_pose.header.frame_id, plan_goal_pose.header.stamp,
                               plan_goal_pose.header.frame_id, transform);
            poseStampedMsgToTF(plan_goal_pose, robot_goal_pose);
            robot_goal_pose.setData(transform * robot_goal_pose);
            robot_goal_pose.stamp_ = transform.stamp_;
            robot_goal_pose.frame_id_ = robot_frame;
        }
        catch (tf::LookupException &ex)
        {
            ROS_ERROR("No Transform available Error: %s\n", ex.what());
            return false;
        }
        catch (tf::ConnectivityException &ex)
        {
            ROS_ERROR("Connectivity Error: %s\n", ex.what());
            return false;
        }
        catch (tf::ExtrapolationException &ex)
        {
            ROS_ERROR("Extrapolation Error: %s\n", ex.what());
            if (global_plan.size() > 0)
                ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", robot_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

            return false;
        }
        return true;
    }

    int TrackTrajectory::checkMaxDistance(tf::Stamped<tf::Pose> current_pose)
    {
        int max_point = 0;
        tf::Stamped<tf::Pose> x_pose;     //robot goal pose
        transformed_global_plan_.clear();
        for (unsigned int i = 0; i < global_plan_.size(); i++)
        {
            //tf::TransformListener listener;
            getRobotGoalPose(*tf_, global_plan_, global_frame_, x_pose, i);
            double distance = sqrt(pow((x_pose.getOrigin().getX() - current_pose.getOrigin().getX()), 2) + pow((x_pose.getOrigin().getY() - current_pose.getOrigin().getY()), 2));
            tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(x_pose,
                                                            ros::Time::now(),
                                                            global_frame_);
            geometry_msgs::PoseStamped pose;
            tf::poseStampedTFToMsg(p, pose);
            transformed_global_plan_.push_back(pose);

            max_point = i - 1;
            //max_point = i;

            if (distance > max_moveable_dis_)
            {

                break;
            }
        }
        if (max_point < 0)
        {
            max_point = 0;
        }
        //printf("global plan size1: %d\n", global_plan_.size());
        //printf("check max distance point: %d \n", max_point);
        return max_point;
    }


    double TrackTrajectory::calculateGlobalPlanAngle(tf::Stamped<tf::Pose> current_pose, const std::vector<geometry_msgs::PoseStamped> &plan, int point)
    {
        if (point >= (int)plan.size())
        {
            point = plan.size() - 1;
        }
        //printf("global plan size2: %d\n", plan.size());
        double angle = 0;
        double current_th = tf::getYaw(current_pose.getRotation());
        for (int i = 0; i <= point; i++)
        {
            geometry_msgs::PoseStamped x_pose;
            x_pose = transformed_global_plan_.at(i);
            double angle_to_goal = atan2(x_pose.pose.position.y - current_pose.getOrigin().getY(),
                                         x_pose.pose.position.x - current_pose.getOrigin().getX());
            angle += angle_to_goal;
        }
        //average
        angle = angle / (point + 1);

        //test
        // geometry_msgs::PoseStamped x_pose;
        // x_pose = transformed_global_plan_.at(point);
        // angle = atan2(x_pose.pose.position.y - current_pose.getOrigin().getY(),
        //               x_pose.pose.position.x - current_pose.getOrigin().getX());

        return angles::shortest_angular_distance(current_th, angle);
    }

    int TrackTrajectory::driveToward(tf::Stamped<tf::Pose> current_pose, geometry_msgs::Twist &cmd_vel)
    {
        double distance = 0;
        double angle = 0;
        double plan_frequence = 5;
        int max_point = 0;

        //Search for max achievable point on global plan.
        max_point = checkMaxDistance(current_pose);
        max_point = checkMaxAngle(max_point, current_pose);
        //max_point = transformed_global_plan_.size() - 1;

        //printf("global plan size: %d \n", (int)global_plan_.size());
        //printf("max_point: %d \n", (int)max_point);
        double cmd_vel_linear_x_old = cmd_vel_linear_x_;
        double cmd_vel_angular_z_old = cmd_vel_angular_z_;

        geometry_msgs::PoseStamped x_pose;
        x_pose = transformed_global_plan_.at(max_point);

        distance = sqrt(pow((x_pose.pose.position.x - current_pose.getOrigin().getX()), 2) +
                        pow((x_pose.pose.position.y - current_pose.getOrigin().getY()), 2));
        angle = calculateGlobalPlanAngle(current_pose, global_plan_, max_point);

        //printf("drivetoword distance: %f\n", distance);
        //printf("drivetoward angle: %f\n", angle);

        if ((distance / sim_time) > max_linear_vel) //max linear vel = 0.2
        {
            cmd_vel_linear_x_ = max_linear_vel;
        }
        else
        {
            cmd_vel_linear_x_ = (distance / sim_time);
            // if (cmd_vel_linear_x_ < 0.1)
            // {
            //     cmd_vel_linear_x_ = 0.1;
            // }
        }

        if (fabs(angle / sim_time) > max_rotation_vel)
        {
            cmd_vel_angular_z_ = max_rotation_vel;
        }
        else
        {
            cmd_vel_angular_z_ = (angle / sim_time);
        }

        if (cmd_vel_linear_x_ > cmd_vel_linear_x_old + acceleration_x_ / plan_frequence)
        {
            cmd_vel_linear_x_ = cmd_vel_linear_x_old + acceleration_x_ / plan_frequence;
        }
        else
        {
            if (cmd_vel_linear_x_ < cmd_vel_linear_x_old - acceleration_x_ / plan_frequence)
            {
                cmd_vel_linear_x_ = cmd_vel_linear_x_old - acceleration_x_ / plan_frequence;
            }
            else
            {
                //printf("new_cmd_vel_x: %f\n", cmd_vel_linear_x_);
                //printf("old_cmd_vel_x: %f\n", cmd_vel_linear_x_old);
                //cmd_vel_linear_x_ = cmd_vel_linear_x_old;
            }
        }

        if (fabs(cmd_vel_angular_z_) > fabs(cmd_vel_angular_z_old) + fabs(acceleration_z_ / plan_frequence))
        {
            if (cmd_vel_angular_z_ < 0)
            {
                cmd_vel_angular_z_ = cmd_vel_angular_z_old - acceleration_z_ / plan_frequence;
            }
            else
            {
                cmd_vel_angular_z_ = cmd_vel_angular_z_old + acceleration_z_ / plan_frequence;
            }
        }

        if (cmd_vel_angular_z_ < 0 && cmd_vel_angular_z_old > 0)
        {
            if (fabs(cmd_vel_angular_z_ - cmd_vel_angular_z_old) > fabs(acceleration_z_ / plan_frequence))
            {
                cmd_vel_angular_z_ = cmd_vel_angular_z_old - acceleration_z_ / plan_frequence;
            }
        }

        if (cmd_vel_angular_z_ > 0 && cmd_vel_angular_z_old < 0)
        {
            if (fabs(cmd_vel_angular_z_ - cmd_vel_angular_z_old) > fabs(acceleration_z_ / plan_frequence))
            {
                cmd_vel_angular_z_ = cmd_vel_angular_z_old + acceleration_z_ / plan_frequence;
            }
        }

        //Check at last if velocity is to high.
        if (cmd_vel_angular_z_ > max_rotation_vel)
        {
            cmd_vel_angular_z_ = max_rotation_vel;
        }
        if (cmd_vel_angular_z_ < -max_rotation_vel)
        {
            cmd_vel_angular_z_ = (-max_rotation_vel);
        }
        if (cmd_vel_linear_x_ > max_linear_vel)
        {
            cmd_vel_linear_x_ = max_linear_vel;
        }
        //Push velocity to cmd_vel for driving.
        cmd_vel.linear.x = cmd_vel_linear_x_;
        cmd_vel.angular.z = cmd_vel_angular_z_;
        cmd_vel_angular_z_rotate_ = cmd_vel_angular_z_;
        //ROS_INFO("max_point: %d, distance: %f, x_vel: %f, rot_vel: %f, angle: %f", max_point, distance, cmd_vel.linear.x, cmd_vel.angular.z, angle);

        return max_point;
    }

    int TrackTrajectory::checkMaxAngle(int points, tf::Stamped<tf::Pose> current_pose)
    {
        int max_point = points;
        double angle = 0;
        for (int i = max_point; i >= 0; i--)
        {
            angle = calculateGlobalPlanAngle(current_pose, global_plan_, i);
            max_point = i;
            if (fabs(angle) < max_rotate_angle_)
            {
                break;
            }
        }
        //printf("check max angle point: %d \n", max_point);
        return max_point;
    }

    bool TrackTrajectory::rotateToOrientation(double angle, geometry_msgs::Twist &cmd_vel, double accuracy)
    {
        //printf("rotateToOrientation!\n");
        double plan_frequence = 5;

        if ((cmd_vel_linear_x_ - 0.1) >= 0)
        {
            //cmd_vel.linear.x = cmd_vel_linear_x_ - 0.1;
            //cmd_vel_linear_x_ = cmd_vel_linear_x_ - 0.1;
            cmd_vel.linear.x = 0.0;
            cmd_vel_linear_x_ = 0.0;
        }
        if (fabs(angle) > accuracy)
        {
            //slow down
            if (max_rotation_vel >= fabs(angle) * (acceleration_z_ + slow_down_factor))
            {
                //printf("slow down.\n");

                if (angle < 0)
                {
                    if (cmd_vel_angular_z_rotate_ >= -min_rotation_vel)
                    {
                        cmd_vel_angular_z_rotate_ = -min_rotation_vel;
                        cmd_vel.angular.z = cmd_vel_angular_z_rotate_;
                    }
                    else
                    {
                        cmd_vel_angular_z_rotate_ = cmd_vel_angular_z_rotate_ + acceleration_z_ / plan_frequence;
                        cmd_vel.angular.z = cmd_vel_angular_z_rotate_;
                    }
                }
                if (angle > 0)
                {
                    if (cmd_vel_angular_z_rotate_ <= min_rotation_vel)
                    {
                        cmd_vel_angular_z_rotate_ = min_rotation_vel;
                        cmd_vel.angular.z = cmd_vel_angular_z_rotate_;
                    }
                    else
                    {
                        cmd_vel_angular_z_rotate_ = cmd_vel_angular_z_rotate_ - acceleration_z_ / plan_frequence;
                        cmd_vel.angular.z = cmd_vel_angular_z_rotate_;
                    }
                }
            }
            else
            {
                // speed up
                //printf("speed up\n");
                if (fabs(cmd_vel_angular_z_rotate_) < max_rotation_vel)
                {
                    //printf("angle: %f\n", angle);
                    if (angle < 0)
                    {
                        // cmd_vel_angular_z_rotate_ = cmd_vel_angular_z_rotate_ - acceleration_z_ / plan_frequence;
                        // if (fabs(cmd_vel_angular_z_rotate_) > max_rotation_vel)
                        // {
                        //     cmd_vel_angular_z_rotate_ = -max_rotation_vel;
                        // }

                        cmd_vel_angular_z_rotate_ = -1.0; // force speed up

                        cmd_vel.angular.z = cmd_vel_angular_z_rotate_;
                    }
                    if (angle > 0)
                    {
                        // cmd_vel_angular_z_rotate_ = cmd_vel_angular_z_rotate_ + acceleration_z_ / plan_frequence;
                        // if (fabs(cmd_vel_angular_z_rotate_) > max_rotation_vel)
                        // {
                        //     cmd_vel_angular_z_rotate_ = max_rotation_vel;
                        // }

                        cmd_vel_angular_z_rotate_ = 1.0; // force speed up

                        cmd_vel.angular.z = cmd_vel_angular_z_rotate_;
                    }
                }
                else
                {
                    cmd_vel.angular.z = cmd_vel_angular_z_rotate_;
                }
            }
            //ROS_INFO("cmd_vel.z: %f, angle: %f", cmd_vel.angular.z, angle);
            return false;
        }
        else
        {
            cmd_vel_angular_z_rotate_ = 0;
            cmd_vel.angular.z = 0;
            return false;
        }
    }

    std::vector<geometry_msgs::PoseStamped> TrackTrajectory::adjust_path(std::vector<geometry_msgs::PoseStamped> &input)
    {
        std::vector<geometry_msgs::PoseStamped> output;
        double t;
        t = 0.1;
        geometry_msgs::PoseStamped goal_point = input[input.size() - 1];
        output = Bezier(t, input);

        //std::cout << "input plan size: " << input.size() << "\n";
        //std::cout << "output plan size: " << output.size() << "\n";
        output.push_back(goal_point);
        return output;
    }

    std::vector<geometry_msgs::PoseStamped> TrackTrajectory::Bezier(double dt, std::vector<geometry_msgs::PoseStamped> &input)
    {
        std::vector<geometry_msgs::PoseStamped> output;
        if (input.size() == 3)
        {
            input.erase(input.begin() + 1);
        }

        double t = 0;
        while (t < 1) //t <= 1
        {
            geometry_msgs::PoseStamped p;
            double x_sum = 0.0;
            double y_sum = 0.0;
            int i = 0;
            int n = input.size() - 1;
            //int n = 3;
            while (i <= n)
            {
                double k = fac(n) / (fac(i) * fac(n - i)) * pow(t, i) * pow(1 - t, n - i);
                x_sum += k * input[i].pose.position.x;
                y_sum += k * input[i].pose.position.y;
                i++;
            }

            p.pose.position.x = x_sum;
            p.pose.position.y = y_sum;
            p.header = input[0].header;
            output.push_back(p);
            t += dt;
        }

        //publishBezierCruvePlan(output);

        return output;
    }

    int TrackTrajectory::fac(int x)
    {
        int f = 1;
        if (x == 0)
        {
            return f;
        }
        else
        {
            for (int i = 1; i <= x; i++)
            {
                f *= i;
            }

            return f;
        }
    }

    void TrackTrajectory::joinMaps()
    {
    }
}
