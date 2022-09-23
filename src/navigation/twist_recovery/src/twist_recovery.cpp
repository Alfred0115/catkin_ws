/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/**
 * \file 
 * 
 * \author Bhaskara Marthi
 * 
 */

#include <twist_recovery/twist_recovery.h>
#include <pluginlib/class_list_macros.hpp>
#include <tf/transform_datatypes.h>
#include <iostream>

// register as a RecoveryBehavior plugin
PLUGINLIB_EXPORT_CLASS(twist_recovery::TwistRecovery, nav_core::RecoveryBehavior)

namespace gm = geometry_msgs;
namespace cmap = costmap_2d;
namespace blp = base_local_planner;
using std::max;
using std::vector;

namespace twist_recovery
{

  TwistRecovery::TwistRecovery() : global_costmap_(NULL), local_costmap_(NULL), tf_(NULL), initialized_(false)
  {
  }

  TwistRecovery::~TwistRecovery()
  {
    delete world_model_;
  }

  void TwistRecovery::initialize(std::string name, tf::TransformListener *tf,
                                 cmap::Costmap2DROS *global_cmap, cmap::Costmap2DROS *local_cmap)
  {
    ROS_ASSERT(!initialized_);
    name_ = name;
    std::cout << "name: " << name_ << std::endl;
    tf_ = tf;
    local_costmap_ = local_cmap;
    global_costmap_ = global_cmap;
    world_model_ = new blp::CostmapModel(*local_costmap_->getCostmap());

    pub_ = nh_.advertise<gm::Twist>("cmd_vel", 10);
    ros::NodeHandle private_nh("~/" + name);

    {
      bool found = true;
      found = found && private_nh.getParam("linear_x", base_frame_twist_.linear.x);
      found = found && private_nh.getParam("linear_y", base_frame_twist_.linear.y);
      found = found && private_nh.getParam("angular_z", base_frame_twist_.angular.z);
      if (!found)
      {
        ROS_FATAL_STREAM("Didn't find twist parameters in " << private_nh.getNamespace());
        ros::shutdown();
      }
    }
    private_nh.param("scan_topic", scan_topic_, std::string("/scan"));
    private_nh.param("plan_time", plan_time_, 0.5);

    private_nh.param("detect_distance", detect_distance_, 100.0);
    private_nh.param("duration", duration_, 1.0);
    private_nh.param("linear_speed_limit", linear_speed_limit_, 0.3);
    private_nh.param("angular_speed_limit", angular_speed_limit_, 1.0);
    private_nh.param("linear_acceleration_limit", linear_acceleration_limit_, 4.0);
    private_nh.param("angular_acceleration_limit", angular_acceleration_limit_, 3.2);
    private_nh.param("controller_frequency", controller_frequency_, 20.0);
    private_nh.param("simulation_inc", simulation_inc_, 1 / controller_frequency_);

    //ROS_INFO_STREAM_NAMED("top", "Initialized twist recovery with twist " << base_frame_twist_ << " and duration " << duration_);

    initialized_ = true;
  }

  gm::Twist scaleTwist(const gm::Twist &twist, const double scale)
  {
    gm::Twist t;
    t.linear.x = twist.linear.x * scale;
    t.linear.y = twist.linear.y * scale;
    t.angular.z = twist.angular.z * scale;
    return t;
  }

  gm::Pose2D forwardSimulate(const gm::Pose2D &p, const gm::Twist &twist, const double t = 1.0)
  {
    gm::Pose2D p2;
    p2.x = p.x + twist.linear.x * t;
    p2.y = p.y + twist.linear.y * t;
    p2.theta = p.theta + twist.angular.z * t;
    return p2;
  }

  /// Return the cost of a pose, modified so that -1 does not equal infinity; instead 1e9 does.
  double TwistRecovery::normalizedPoseCost(const gm::Pose2D &pose) const
  {
    //std::vector<gm::Point> padded_footprint = local_costmap_->getRobotFootprint();
    // std::cout << "footprint size: " << padded_footprint.size() << std::endl;
    // for(int i = 0; i < padded_footprint.size(); ++i) {
    //   printf("point %d (%.3f, %.3f, %.3f)\n", i, padded_footprint[i].x, padded_footprint[i].y, padded_footprint[i].z);
    // }

    const double c = world_model_->footprintCost(pose.x, pose.y, pose.theta, local_costmap_->getRobotFootprint(), 0.0, 0.0);
    //std::cout << "c: " << c << std::endl; //find robot footprint cost
    return c < 0 ? 1e9 : c;
  }

  /// Return the maximum d <= duration_ such that starting at the current pose, the cost is nonincreasing for
  /// d seconds if we follow twist
  /// It might also be good to have a threshold such that we're allowed to have lethal cost for at most
  /// the first k of those d seconds, but this is not done
  double TwistRecovery::nonincreasingCostInterval(const gm::Pose2D &current, const gm::Twist &twist) const
  {
    double cost = normalizedPoseCost(current);
    double t; // Will hold the first time that is invalid
    for (t = simulation_inc_; t <= duration_; t += simulation_inc_)
    {
      const double next_cost = normalizedPoseCost(forwardSimulate(current, twist, t));
      if (next_cost > cost)
      {
        ROS_DEBUG_STREAM_NAMED("cost", "Cost at " << t << " and pose " << forwardSimulate(current, twist, t)
                                                  << " is " << next_cost << " which is greater than previous cost " << cost);
        break;
      }
      cost = next_cost;
    }

    return t - simulation_inc_;
  }

  double linearSpeed(const gm::Twist &twist)
  {
    return sqrt(twist.linear.x * twist.linear.x + twist.linear.y * twist.linear.y);
  }

  double angularSpeed(const gm::Twist &twist)
  {
    return fabs(twist.angular.z);
  }

  // Scale twist so we can stop in the given time, and so it's within the max velocity
  gm::Twist TwistRecovery::scaleGivenAccelerationLimits(const gm::Twist &twist, const double time_remaining) const
  {
    const double linear_speed = linearSpeed(twist);
    const double angular_speed = angularSpeed(twist);
    const double linear_acc_scaling = linear_speed / (time_remaining * linear_acceleration_limit_);
    const double angular_acc_scaling = angular_speed / (time_remaining * angular_acceleration_limit_);
    const double acc_scaling = max(linear_acc_scaling, angular_acc_scaling);
    const double linear_vel_scaling = linear_speed / linear_speed_limit_;
    const double angular_vel_scaling = angular_speed / angular_speed_limit_;
    const double vel_scaling = max(linear_vel_scaling, angular_vel_scaling);
    return scaleTwist(twist, max(1.0, max(acc_scaling, vel_scaling)));
  }

  // Get pose in local costmap frame
  gm::Pose2D TwistRecovery::getCurrentLocalPose() const
  {
    tf::Stamped<tf::Pose> p;
    local_costmap_->getRobotPose(p);
    gm::Pose2D pose;
    pose.x = p.getOrigin().x();
    pose.y = p.getOrigin().y();
    pose.theta = tf::getYaw(p.getRotation());
    return pose;
  }

  void TwistRecovery::runBehavior()
  {
    printf("run twist recovery!");
    ROS_ASSERT(initialized_);
    int turnCount = 0;

    while (turnCount < 10)
    {
      // Figure out how long we can safely run the behavior
      const gm::Pose2D &current = getCurrentLocalPose();

      const double d = nonincreasingCostInterval(current, base_frame_twist_);
      ros::Rate r(controller_frequency_);
      ros::NodeHandle scan_nh;
      ros::Subscriber scan_sub;
     

      if (d > plan_time_)
      {
        turnCount = 0;
        // back (below 7) 
        // for (double t = 0; t < d; t += 1 / controller_frequency_)
        // {
        //   pub_.publish(scaleGivenAccelerationLimits(base_frame_twist_, d - t));
        //   // scan_sub = scan_nh.subscribe<sensor_msgs::LaserScan>(scan_topic_, 1, boost::bind(&TwistRecovery::scanCallback, this, _1));
        //   // ros::spinOnce();
        //   r.sleep();
        // }
        
        scan_sub = scan_nh.subscribe<sensor_msgs::LaserScan>(scan_topic_, 1, boost::bind(&TwistRecovery::scanCallback, this, _1));
        gm::Twist stop_vel;
        stop_vel.linear.x = 0;
        stop_vel.linear.y = 0;
        stop_vel.angular.z = 0;
        pub_.publish(stop_vel);
        sleep(1);

        if (obstacle_direction_ == "Unobstructed path!")
        {
          ROS_INFO("There is no obstacle within one meter ahead!");
        }
        else if (obstacle_direction_ == "RightLow")
        {
          turnAngle(15); //50
        }
        else if (obstacle_direction_ == "RightHigh")
        {
          turnAngle(15); //35
        }
        else if (obstacle_direction_ == "LeftLow")
        {
          turnAngle(-15); //-50
        }
        else if (obstacle_direction_ == "LeftHigh")
        {
          turnAngle(-15); //-35
        }
        ROS_INFO("Twist recovery end");
        turnCount++;
        return;
      }
      else
      {
        // ROS_INFO("turn and try again");
        // bool turnResult = turnAngle(90);
        // if (turnResult) {
                // turnCount++;
        // } else {
          ROS_ERROR("I can't turn my body!");
        // }
        
      }
    }
    ROS_ERROR("---I have tried many times and recover over---");
    turnCount = 0;
    return;
  }

  //转弯
  bool TwistRecovery::turnAngle(const double angle)
  {
    ROS_INFO("angle: %.2f , direction: %s", angle, obstacle_direction_.c_str());
    ros::Rate r(controller_frequency_);
    ros::NodeHandle nv;
    ros::Publisher vel_pub_ = nv.advertise<gm::Twist>("cmd_vel", 10);
    double angle_speed;
    if (angle > 0)
    {
      angle_speed = 0.5;
    }
    else
    {
      angle_speed = -0.5;
    }

    double angle_duration = (angle / 360 * 2 * 3.14) / angle_speed;
    int ticks = int(angle_duration * controller_frequency_);
    abs(ticks);
    gm::Twist cmd_vel;
    cmd_vel.linear.x = 0.0;
    cmd_vel.linear.y = 0.0;
    while (ticks > 0)
    {
      ticks--;
      cmd_vel.angular.z = angle_speed;
      vel_pub_.publish(cmd_vel);
      r.sleep();
    }
    cmd_vel.angular.z = 0.0;
    vel_pub_.publish(cmd_vel);
    r.sleep();
    return true;
  }


  void TwistRecovery::scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
  {
    int scan_size = msg->ranges.size();
    int min_range_index = 0;
    float min_range_ = 100;

    if (scan_size == 0)
    {
      ROS_INFO("Cannot get laser data!");
      obstacle_direction_ = "Unobstructed path!";
      return;
    }
    //寻找离机器人最近的障碍物距离
    for (int i = 0; i < scan_size; i++)
    {
      if (msg->ranges[i] < min_range_ && std::isfinite(msg->ranges[i]))
      {
        min_range_ = msg->ranges[i];
        min_range_index = i;
      }
    }

    if (min_range_ > (detect_distance_ / 100))
    {
      obstacle_direction_ = "Unobstructed path!";
      return;
    }
    else
    {
      //ROS_INFO("The distance between the robot and the nearest obstacle is %.2f m", min_range_);
      //计算最近的障碍物激光数据的角度
      float obstacle_angle = min_range_index * msg->angle_increment / 3.14 * 180;
      //ROS_INFO("The included angle between the obstacle point and the laser initial point is %.2f", obstacle_angle);

      //利用激光的夹角推测障碍物的方位，这里默认激光角度为 -1.57 ~ 1.57
      if (obstacle_angle < 45)
      {
        obstacle_direction_ = "RightLow";
        //turnAngle(40);
      }
      else if (obstacle_angle < 90)
      {
        obstacle_direction_ = "RightHigh";
        //turnAngle(22.5);
      }
      else if (obstacle_angle < 135)
      {
        obstacle_direction_ = "LeftHigh";
        //turnAngle(-22.5);
      }
      else
      {
        obstacle_direction_ = "LeftLow";
        //turnAngle(-40);
      }

      return;
    }
  }

} // namespace twist_recovery
