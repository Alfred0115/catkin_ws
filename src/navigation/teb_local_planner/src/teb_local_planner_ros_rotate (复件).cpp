/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016,
 *  TU Dortmund - Institute of Control Theory and Systems Engineering.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the institute nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Christoph R閺嬫mann
 *********************************************************************/

#include <teb_local_planner/teb_local_planner_ros.h>

#include <tf_conversions/tf_eigen.h>

#include <boost/algorithm/string.hpp>

// pluginlib macros
#include <pluginlib/class_list_macros.h>

#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/csparse/linear_solver_csparse.h"
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"

// register this planner as a BaseLocalPlanner plugin
PLUGINLIB_EXPORT_CLASS(teb_local_planner::TebLocalPlannerROS, nav_core::BaseLocalPlanner)

namespace teb_local_planner
{
  static const float MY_PI = 3.1415926f;
  static bool vel_first_plan_flag = false;//lyy
  tf::Stamped<tf::Pose> global_charging_pile_start;//lyy
  bool my_start_rotate = false, my_mid_rotate = false,my_end_rotate = false;//lyy
  float temp_goal_theta_raw = 0.0f,temp_goal_theta = 0.0f;
  bool goal_reached_once = false;
  geometry_msgs::PoseStamped my_global_goal;
  double final_th = 0.0;
  bool final_th_init = false;
  double init_abs_sqrt_dist = 100.0;
  bool no_more_teb_plan = false;
  float last_vel_x = 0.0;
  bool lpf_init_flag = false;
  bool final_goal_check = false;

  double angle_remapping(double theta){
    if (theta >= -MY_PI && theta < MY_PI)
      return theta;
    
    double multiplier = std::floor(theta / (2*MY_PI));
    theta = theta - multiplier*2*MY_PI;
    if (theta >= MY_PI)
      theta -= 2*MY_PI;
    if (theta < -MY_PI)
      theta += 2*MY_PI;

    return theta;
  }

  double quaternion_to_yaw(const double &x, const double &y, const double &z, const double &w){
    double t1 = 2.0 * (w * z + x * y);
    double t2 = 1.0 - 2.0 * (y * y + z * z);
    double yaw = atan2(t1, t2);

    return yaw;
  }

  float vel_lpf(float vel, float &last_vel, float ratio){
    ratio = ratio > 1.0 ? 1.0 : ratio;
    ratio = ratio < 0.0 ? 0.0 : ratio;

    last_vel = ratio*last_vel + (1.0-ratio)*vel;

    return last_vel;
  }

  bool my_rotate_adjustment(double in, bool &flag, geometry_msgs::Twist &my_cmd_vel,
                            const double &stop_threshold){
    in = angle_remapping(in);
    float abs_in = fabs(in);
    if(abs_in > 0.0001 && !flag){
      ROS_INFO("~~~~~~~~in: %f",in);
      // my_cmd_vel.linear.x = 0;
      // my_cmd_vel.linear.y = 0;
      float vel_tmp = 0.0f;
      if(abs_in > 0.1){
        vel_tmp = abs_in;
      }else if(abs_in > fabs(stop_threshold)){
        vel_tmp = 0.05;
      }else{
        vel_tmp = 0.0;
        flag = true;
      }
      vel_tmp = vel_tmp > 1.0 ? 1.0 : vel_tmp;
      if(in > 0) my_cmd_vel.angular.z = vel_tmp;
      else       my_cmd_vel.angular.z = -vel_tmp;

      return true;    
    }else{
      return false;
    }
  }

  TebLocalPlannerROS::TebLocalPlannerROS() : costmap_ros_(NULL), tf_(NULL), costmap_model_(NULL),
                                             costmap_converter_loader_("costmap_converter", "costmap_converter::BaseCostmapToPolygons"),
                                             dynamic_recfg_(NULL), custom_via_points_active_(false), goal_reached_(false), no_infeasible_plans_(0),
                                             last_preferred_rotdir_(RotType::none), initialized_(false)
  {
  }

  TebLocalPlannerROS::~TebLocalPlannerROS()
  {
  }

  void TebLocalPlannerROS::reconfigureCB(TebLocalPlannerReconfigureConfig &config, uint32_t level)
  {
    cfg_.reconfigure(config);
  }
  ros::NodeHandle n("~TebLocalPlannerROS");

  void TebLocalPlannerROS::initialize(std::string name, tf::TransformListener *tf, costmap_2d::Costmap2DROS *costmap_ros)
  {
    // check if the plugin is already initialized
    if (!initialized_)
    {
      // create Node Handle with name of plugin (as used in move_base for loading)
      ros::NodeHandle nh("~/" + name);

      //printf("name: %s \n", name.c_str());

      // get parameters of TebConfig via the nodehandle and override the default config
      cfg_.loadRosParamFromNodeHandle(nh);

      // reserve some memory for obstacles
      obstacles_.reserve(500);

      // create visualization instance
      visualization_ = TebVisualizationPtr(new TebVisualization(nh, cfg_));

      // create robot footprint/contour model for optimization
      RobotFootprintModelPtr robot_model = getRobotFootprintFromParamServer(nh);

      // create the planner instance
      if (cfg_.hcp.enable_homotopy_class_planning)
      {
        planner_ = PlannerInterfacePtr(new HomotopyClassPlanner(cfg_, &obstacles_, robot_model, visualization_, &via_points_));
        ROS_INFO("Parallel planning in distinctive topologies enabled.");
      }
      else
      {
        planner_ = PlannerInterfacePtr(new TebOptimalPlanner(cfg_, &obstacles_, robot_model, visualization_, &via_points_));
        ROS_INFO("Parallel planning in distinctive topologies disabled.");
      }

      // init other variables
      tf_ = tf;
      costmap_ros_ = costmap_ros;
      costmap_ = costmap_ros_->getCostmap(); // locking should be done in MoveBase.

      costmap_model_ = boost::make_shared<base_local_planner::CostmapModel>(*costmap_);

      global_frame_ = costmap_ros_->getGlobalFrameID();
      cfg_.map_frame = global_frame_; // TODO
      robot_base_frame_ = costmap_ros_->getBaseFrameID();

      //Initialize a costmap to polygon converter
      if (!cfg_.obstacles.costmap_converter_plugin.empty())
      {
        try
        {
          costmap_converter_ = costmap_converter_loader_.createInstance(cfg_.obstacles.costmap_converter_plugin);
          std::string converter_name = costmap_converter_loader_.getName(cfg_.obstacles.costmap_converter_plugin);
          // replace '::' by '/' to convert the c++ namespace to a NodeHandle namespace
          boost::replace_all(converter_name, "::", "/");
          costmap_converter_->setOdomTopic(cfg_.odom_topic);
          costmap_converter_->initialize(ros::NodeHandle(nh, "costmap_converter/" + converter_name));
          costmap_converter_->setCostmap2D(costmap_);

          costmap_converter_->startWorker(ros::Rate(cfg_.obstacles.costmap_converter_rate), costmap_, cfg_.obstacles.costmap_converter_spin_thread);
          ROS_INFO_STREAM("Costmap conversion plugin " << cfg_.obstacles.costmap_converter_plugin << " loaded.");
        }
        catch (pluginlib::PluginlibException &ex)
        {
          ROS_WARN("The specified costmap converter plugin cannot be loaded. All occupied costmap cells are treaten as point obstacles. Error message: %s", ex.what());
          costmap_converter_.reset();
        }
      }
      else
        ROS_INFO("No costmap conversion plugin specified. All occupied costmap cells are treaten as point obstacles.");

      // Get footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
      footprint_spec_ = costmap_ros_->getRobotFootprint();
      costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);

      // init the odom helper to receive the robot's velocity from odom messages
      odom_helper_.setOdomTopic(cfg_.odom_topic);

      // setup dynamic reconfigure
      dynamic_recfg_ = boost::make_shared<dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>>(nh);
      dynamic_reconfigure::Server<TebLocalPlannerReconfigureConfig>::CallbackType cb = boost::bind(&TebLocalPlannerROS::reconfigureCB, this, _1, _2);
      dynamic_recfg_->setCallback(cb);

      // validate optimization footprint and costmap footprint
      validateFootprints(robot_model->getInscribedRadius(), robot_inscribed_radius_, cfg_.obstacles.min_obstacle_dist);

      // setup callback for custom obstacles
      custom_obst_sub_ = nh.subscribe("obstacles", 1, &TebLocalPlannerROS::customObstacleCB, this);

      // setup callback for custom via-points
      via_points_sub_ = nh.subscribe("via_points", 1, &TebLocalPlannerROS::customViaPointsCB, this);

      //lyy: publish local planner status to amcl to use pure odom localization
      approaching_goal_pub_ = nh.advertise<std_msgs::Bool>("approaching_goal", 2, false);

      // initialize failure detector
      ros::NodeHandle nh_move_base("~");
      double controller_frequency = 5;
      nh_move_base.param("controller_frequency", controller_frequency, controller_frequency);
      failure_detector_.setBufferLength(std::round(cfg_.recovery.oscillation_filter_duration * controller_frequency));

      //load crucial points list from yaml
      std::string params = "crucial_points";
      parseCrucialPointSuccess = parseCrucialPointsListFromYaml(&nh_move_base, params);
      if (!parseCrucialPointSuccess)
      {
        ROS_ERROR_STREAM("Reading crucial points list form ' " << nh_move_base.getNamespace() << "/" << params << " ' failed!");
      }

      //nh_move_base.param("crucial_points", crucial_points_, crucial_points_);
      // set initialized flag
      initialized_ = true;
      
      //lyy: backup vel and acc parameters
      max_vel_x_backup_ = cfg_.robot.max_vel_x;
      max_vel_theta_backup_ = cfg_.robot.max_vel_theta;
      acc_lim_x_backup_ = cfg_.robot.acc_lim_x;
      acc_lim_theta_backup_ = cfg_.robot.acc_lim_theta;
      ROS_DEBUG("teb_local_planner plugin initialized.");
    }
    else
    {
      ROS_WARN("teb_local_planner has already been initialized, doing nothing.");
    }
  }

  //load crucial points list out of the rosparam server
  bool TebLocalPlannerROS::parseCrucialPointsListFromYaml(ros::NodeHandle *nhandle, const std::string &param)
  {
    //printf("erase init\n");
    std::lock_guard<std::mutex> l(_data_mutex);
    XmlRpc::XmlRpcValue param_yaml;
    bool ret_val = true;

    if (nhandle->getParam(param, param_yaml))
    {
      if (param_yaml.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        for (int i = 0; i < param_yaml.size(); ++i)
        {
          std::vector<double> cp;
          cp.push_back(static_cast<double>(param_yaml[i][0][0]));
          cp.push_back(static_cast<double>(param_yaml[i][0][1]));
          crucialPointFromYaml.push_back(cp);
          //printf("crucial point size: %d \n",crucialPointFromYaml.size());
          std::vector<double> vp;
          vp.push_back(static_cast<double>(param_yaml[i][1][0]));
          vp.push_back(static_cast<double>(param_yaml[i][1][1]));
          viaPointFromYaml.push_back(vp);
          //printf("viaPoint size: %d \n", viaPointFromYaml.size());
        }
      }
      else
      {
        ROS_ERROR_STREAM("Crucial points list: " << param << " struct is not correct.");
        ret_val = false;
      }
    }
    else
    {
      ROS_ERROR_STREAM("Crucial points list: Cannot read " << param << " from parameter server");
      ret_val = false;
    }
    return ret_val;
  }

  bool TebLocalPlannerROS::setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan)
  {
    ROS_INFO("setPlannnnnnnnnnnnn\n");

    // check if plugin is initialized
    if (!initialized_)
    {
      ROS_ERROR("teb_local_planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    //lyy
    if(goal_reached_){
      vel_first_plan_flag = false;
      init_abs_sqrt_dist = 100.0;//lyy
      no_more_teb_plan = false;//lyy
      lpf_init_flag = false;//lyy
      final_goal_check = false;
      final_th_init = false;
    }

    // store the global plan
    global_plan_.clear();
    int size = orig_global_plan.size();

    std::vector<geometry_msgs::PoseStamped> global_plan_need_change;
    //global_plan_need_change.clear();
    global_plan_need_change = orig_global_plan;

    //lyy
    my_global_goal = global_plan_need_change[size - 1];

    if (size > 20)
    {
      global_plan_need_change[size - 1].pose.orientation.x = orig_global_plan[size - 20].pose.orientation.x;
      global_plan_need_change[size - 1].pose.orientation.y = orig_global_plan[size - 20].pose.orientation.y;
      global_plan_need_change[size - 1].pose.orientation.z = orig_global_plan[size - 20].pose.orientation.z;
      global_plan_need_change[size - 1].pose.orientation.w = orig_global_plan[size - 20].pose.orientation.w;
    }
    else
    {
      global_plan_need_change[size - 1].pose.orientation.x = orig_global_plan[1].pose.orientation.x;
      global_plan_need_change[size - 1].pose.orientation.y = orig_global_plan[1].pose.orientation.y;
      global_plan_need_change[size - 1].pose.orientation.z = orig_global_plan[1].pose.orientation.z;
      global_plan_need_change[size - 1].pose.orientation.w = orig_global_plan[1].pose.orientation.w;
    }

    global_plan_ = global_plan_need_change;
    // global_plan_ = orig_global_plan;
    

    if (parseCrucialPointSuccess)
    {

      via_point.poses.clear();      //婵炴挸鎳樺▍宸歩a_point闁轰焦澹嗙划锟�
      crucialPointContiner.clear(); //婵炴挸鎳樺▍宸唕ucial_points闁轰焦澹嗙划锟�

      //闁告帇鍊栭弻鍥礂閵娿儳婀伴悹渚灠缁剁偤寮伴姘剨缂備礁绻楃换鍐偋鐟欏嫮鏆曠€殿噮鍨舵禍楣冩倷閻у摜绀夋鐐舵硾閻ㄣ垻鎹勯姘辩獮闁绘劙鈧稓鐟愰柣銊ュia point濞ｅ洦绻傞悺銊╁礆閻ㄥ患a_point闁轰焦澹嗙划锟�
      isPassThroughCrucialPoint = passThroughCrucialPoint(global_plan_);

      if (isPassThroughCrucialPoint)
      {
        via_points_.clear();
        pub_via_point = n.advertise<nav_msgs::Path>("via_points", 1, true);
        if (via_point.poses.size() == 1)
        {
          pub_via_point.publish(via_point);
        }
        else
        {
          nav_msgs::Path first_via;
          geometry_msgs::PoseStamped first_point = via_point.poses[0]; //閻犱警鍨扮欢鐐寸▔婵犲偆娲ら柡瀣矋濠€浣瑰緞濮橆偊鍤媣ia point 闁稿繐鐗嗚ぐ鍌涚▔閳ь剚绋夐搴ｇ闂侇剙鐏濋崢銈夋倷闁稓顓洪梻鍌氼嚟濞村绂嶉幒鎴濐殯闁跨噦鎷�?
          first_via.poses.push_back(first_point);

          pub_via_point.publish(first_via);
        }
      }
    }

    //via_point.poses.clear();

    // we do not clear the local planner here, since setPlan is called frequently whenever the global planner updates the plan.
    // the local planner checks whether it is required to reinitialize the trajectory or not within each velocity computation step.

    // reset goal_reached_ flag
    goal_reached_ = false;    

    return true;
  }

  bool TebLocalPlannerROS::passThroughCrucialPoint(const std::vector<geometry_msgs::PoseStamped> &global_path)
  {
    if (crucialPointFromYaml.size() == 0 || global_path.size() == 0)
    {
      return false;
    }

    int via_points_count = 0;

    for (int i = 0; i < global_path.size(); ++i)
    {
      for (int j = 0; j < crucialPointFromYaml.size(); ++j)
      {
        double xDiff = global_path[i].pose.position.x - crucialPointFromYaml[j][0];
        double yDiff = global_path[i].pose.position.y - crucialPointFromYaml[j][1];
        double dist = xDiff * xDiff + yDiff * yDiff;
        if (dist < 1)
        {
          geometry_msgs::PoseStamped p;
          p.pose.position.x = viaPointFromYaml[j][0];
          p.pose.position.y = viaPointFromYaml[j][1];
          via_point.poses.push_back(p);
          via_points_count++;

          std::vector<double> crucial_;
          crucial_.push_back(crucialPointFromYaml[j][0]); //x
          crucial_.push_back(crucialPointFromYaml[j][1]); //y
          crucialPointContiner.push_back(crucial_);
        }
        else
        {
          //printf("path not via crucial point scene dist is %.2f \n", dist);
          continue;
        }
      }
    }
    if (via_points_count > 0)
    {
      //map ->闁靛棌鍋搊dom
      tf::TransformListener listener;
      geometry_msgs::PointStamped pose_map;

      pose_map.header.frame_id = "map";
      pose_map.header.stamp = ros::Time();

      for (int i = 0; i < via_point.poses.size(); ++i)
      {
        pose_map.point.x = via_point.poses[i].pose.position.x;
        pose_map.point.y = via_point.poses[i].pose.position.y;
        pose_map.point.z = 0;
        geometry_msgs::PointStamped pose_odom;

        try
        {
          printf("transform map -> odom");
          listener.waitForTransform("odom", "map", ros::Time(0), ros::Duration(3));
          listener.transformPoint("odom", pose_map, pose_odom);
          via_point.poses[i].pose.position.x = pose_odom.point.x;
          via_point.poses[i].pose.position.y = pose_odom.point.y;
          
      
          //first_via.poses[0].header.frame_id = pose_odom.header.frame_id;
          ROS_INFO("map: (%.2f, %.2f. %.2f) -----> odom: (%.2f, %.2f, %.2f) at time %.2f",
                   pose_map.point.x, pose_map.point.y, pose_map.point.z,
                   pose_odom.point.x, pose_odom.point.y, pose_odom.point.z, pose_odom.header.stamp.toSec());
        }
        catch (tf::TransformException ex)
        {
          ROS_WARN("transfrom exception : %s", ex.what());
        }
      }

      return true;
    }
    return false;
  }

  bool TebLocalPlannerROS::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
  {
    // check if plugin initialized
    if (!initialized_)
    {
      ROS_ERROR("teb_local_planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    cmd_vel.linear.x = 0;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = 0;
    goal_reached_ = false;

    // Get robot pose
    tf::Stamped<tf::Pose> robot_pose;
    costmap_ros_->getRobotPose(robot_pose);
    robot_pose_ = PoseSE2(robot_pose);

    //lyy
    double my_yaw = tf::getYaw(robot_pose.getRotation());

    // Get robot velocity
    tf::Stamped<tf::Pose> robot_vel_tf;
    odom_helper_.getRobotVel(robot_vel_tf);
    robot_vel_.linear.x = robot_vel_tf.getOrigin().getX();
    robot_vel_.linear.y = robot_vel_tf.getOrigin().getY();
    robot_vel_.angular.z = tf::getYaw(robot_vel_tf.getRotation());

    // prune global plan to cut off parts of the past (spatially before the robot)
    pruneGlobalPlan(*tf_, robot_pose, global_plan_, cfg_.trajectory.global_plan_prune_distance);

    // Transform global plan to the frame of interest (w.r.t. the local costmap)
    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    int goal_idx;
    tf::StampedTransform tf_plan_to_global;
    if (!transformGlobalPlan(*tf_, global_plan_, robot_pose, *costmap_, global_frame_, cfg_.trajectory.max_global_plan_lookahead_dist,
                             transformed_plan, &goal_idx, &tf_plan_to_global))
    {
      ROS_WARN("Could not transform the global plan to the frame of the controller");
      return false;
    }

    //lyy
    std_msgs::Bool approaching_msg;
    int current_global_plan_size = int(global_plan_.size()) - 1;
    printf("current goal_idx: %d\nglobal_plan_size: %d\n",goal_idx,current_global_plan_size);
    if(goal_idx == current_global_plan_size){//enter final local plan
      double my_yaw2 = 0.5*my_yaw;
      transformed_plan.back().pose.orientation.x = 0.0;
      transformed_plan.back().pose.orientation.y = 0.0;
      transformed_plan.back().pose.orientation.z = sin(my_yaw2);
      transformed_plan.back().pose.orientation.w = cos(my_yaw2);
    }

    // update via-points container
    if (!custom_via_points_active_)
    {
      updateViaPointsContainer(transformed_plan, cfg_.trajectory.global_plan_viapoint_sep);
    }

    // check if global goal is reached
    tf::Stamped<tf::Pose> global_goal;
    
    //lyy: get start point
    if(!vel_first_plan_flag){
      vel_first_plan_flag = true;
      tf::poseStampedMsgToTF(global_plan_.front(), global_charging_pile_start);
      global_charging_pile_start.setData( tf_plan_to_global * global_charging_pile_start);
    }    
    tf::poseStampedMsgToTF(global_plan_.back(), global_goal);
    ROS_INFO("global final goal x:%f,y:%f",global_goal.getOrigin().getX(),global_goal.getOrigin().getY());
    global_goal.setData(tf_plan_to_global * global_goal);
    double dx = global_goal.getOrigin().getX() - robot_pose_.x();
    double dy = global_goal.getOrigin().getY() - robot_pose_.y();
    double delta_orient = g2o::normalize_theta(tf::getYaw(global_goal.getRotation()) - robot_pose_.theta());


    tf::Stamped<tf::Pose> tmp_pose1;
    tf::poseStampedMsgToTF(my_global_goal, tmp_pose1);//lyy
    tmp_pose1.setData(tf_plan_to_global * tmp_pose1);
    temp_goal_theta_raw = tf::getYaw(tmp_pose1.getRotation());

    temp_goal_theta = g2o::normalize_theta(temp_goal_theta_raw - robot_pose_.theta());//lyy
    double d_dist = fabs(std::sqrt(dx*dx+dy*dy));
    printf("delta_orient is %f\n",delta_orient);
    
    //lyy: get start point delta dist
    double dx_s = global_charging_pile_start.getOrigin().getX() - robot_pose_.x();
    double dy_s = global_charging_pile_start.getOrigin().getY() - robot_pose_.y();
    double ds_orient = g2o::normalize_theta( tf::getYaw(global_charging_pile_start.getRotation()) - robot_pose_.theta());
    double ds_dist = fabs(std::sqrt(dx_s*dx_s+dy_s*dy_s));
  
    
    if (custom_via_points_active_ && fabs(std::sqrt(dx * dx + dy * dy)) < 2 * cfg_.goal_tolerance.xy_goal_tolerance && fabs(delta_orient) < 2 * cfg_.goal_tolerance.yaw_goal_tolerance)
    {
      via_points_.clear();
    }

    //lyy: start point and end point move slowly!!
    if(ds_dist < 0.5){
        cmd_vel.linear.x = 0;
        cmd_vel.linear.y = 0;
        cmd_vel.angular.z = 0;
        // //lyy: TODO ......
        // double footprint_cost = costmap_model_.get()->footprintCost(robot_pose_.x(), robot_pose_.y(), robot_pose_.theta(),
        //     footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);
        
        // ROS_INFO("footprint_cost: %f,robot_inscribed_radius_: %f,robot_circumscribed_radius: %f",
        //                 footprint_cost, robot_inscribed_radius_, robot_circumscribed_radius);
        // if(footprint_cost < 254){//free
              
          if(my_rotate_adjustment(ds_orient, my_start_rotate, cmd_vel, 0.1)){
            return true;    
          }else{
            cfg_.robot.max_vel_x = 0.3;
            cfg_.robot.max_vel_theta = 0.5;
            cfg_.robot.acc_lim_x = 0.3;
            cfg_.robot.acc_lim_theta = 0.5;
          }
        // }else{//not free
        //   cmd_vel.linear.x = 0;
        //   cmd_vel.linear.y = 0;
        //   cmd_vel.angular.z = 0;

        //   ROS_WARN("Current robot footprint not free, don't start firstly!!");
        //   return true;
        // }
    }else if(d_dist < 0.5){
        cfg_.robot.max_vel_x = 0.3;
        cfg_.robot.max_vel_theta = 0.5;
        cfg_.robot.acc_lim_x = 0.3;
        cfg_.robot.acc_lim_theta = 0.5;
    }else{
      cfg_.robot.max_vel_x = max_vel_x_backup_;
      cfg_.robot.max_vel_theta = max_vel_theta_backup_;
      cfg_.robot.acc_lim_x = acc_lim_x_backup_;
      cfg_.robot.acc_lim_theta = acc_lim_theta_backup_;   
    }

    double abs_sqrt_dist = fabs(std::sqrt(dx * dx + dy * dy));
    ROS_INFO("abs_sqrt_dist: %f, init_abs_sqrt_dist: %f\n",abs_sqrt_dist, init_abs_sqrt_dist);
    if(abs_sqrt_dist < init_abs_sqrt_dist){
      init_abs_sqrt_dist = abs_sqrt_dist;
    }

    if (abs_sqrt_dist < cfg_.goal_tolerance.xy_goal_tolerance || (init_abs_sqrt_dist<0.2 && abs_sqrt_dist>0.3)/*(final_goal_check 
        && (abs_sqrt_dist - init_abs_sqrt_dist) > 0)*/ //&&
        //fabs(dx) < cfg_.goal_tolerance.xy_goal_tolerance && fabs(dy) < 0.02 && //zy
        // fabs(delta_orient) < cfg_.goal_tolerance.yaw_goal_tolerance && 
        // (!cfg_.goal_tolerance.complete_global_plan || via_points_.size() == 0)
      )
    {
      // planner_->clearPlanner();
      printf("+++++++++++ wilson: robot is arriving at position, waiting for angle complete!\n");
      goal_reached_once = true;//lyy
      ROS_INFO("abs_sqrt_dist goal_reached: %f, init_abs_sqrt_dist: %f\n",
                                                      abs_sqrt_dist,init_abs_sqrt_dist);

      // goal_reached_ = true;
      // my_start_rotate = false;//lyy
      // my_end_rotate = false;//lyy
      
      // return true;
    }

    //lyy
    if(goal_reached_once){
      cmd_vel.linear.x = 0;
      cmd_vel.linear.y = 0;
      cmd_vel.angular.z = 0;

      // double footprint_cost = costmap_model_.get()->footprintCost(robot_pose_.x(), robot_pose_.y(), robot_pose_.theta(),
      //     footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);
      // ROS_INFO("footprint_cost: %f,robot_inscribed_radius_: %f,robot_circumscribed_radius: %f",
      //                 footprint_cost, robot_inscribed_radius_, robot_circumscribed_radius);
      // if(footprint_cost < 254)
      // {

        if(!my_rotate_adjustment(temp_goal_theta, my_end_rotate, cmd_vel, 0.01)){
          goal_reached_ = true;
          my_start_rotate = false;
          my_end_rotate = false;
          goal_reached_once = false;
          my_mid_rotate = false;
          final_goal_check = false;
          std_msgs::Bool tmp;
          tmp.data = false;
          approaching_goal_pub_.publish(tmp);
          printf("+++++++++++ robot complete angle!\n");
        }
      // }else{
      //   cmd_vel.linear.x = 0;
      //   cmd_vel.linear.y = 0;
      //   cmd_vel.angular.z = 0;
      //   ROS_WARN("Current robot footprint not free, don't start finally!!");
      //   return true;
      // }

      return true;
    }
  

    if(goal_idx == current_global_plan_size){//enter final local plan
      approaching_msg.data = true;
      // approaching_goal_pub_.publish(approaching_msg);
      /* //adjust transformed_plan to fit diff model
      geometry_msgs::PoseStamped current_pose;
      double tmp_x, tmp_y, tmp_th;
      double tmp_dx, tmp_dy, current_angle, next_angle;
      double coefficient, sin_coef, cos_coef;
      for(int i=0; i<transformed_plan.size()-1; i++){
        
        geometry_msgs::PoseStamped &next_pose = transformed_plan.at(i+1);
        current_pose = transformed_plan.at(i);
        current_angle = quaternion_to_yaw(current_pose.pose.orientation.x,current_pose.pose.orientation.y,
                                          current_pose.pose.orientation.z,current_pose.pose.orientation.w);
        next_angle = quaternion_to_yaw(next_pose.pose.orientation.x,next_pose.pose.orientation.y,
                                          next_pose.pose.orientation.z,next_pose.pose.orientation.w);
        tmp_dx = next_pose.pose.position.x - current_pose.pose.position.x;
        tmp_dy = next_pose.pose.position.y - current_pose.pose.position.y;
        sin_coef = sin(current_angle) + sin(next_angle);
        cos_coef = cos(current_angle) + cos(next_angle);
        if(cos_coef != 0){
          coefficient = sin_coef / cos_coef;
        }

        if(coefficient != 0){
          //tmp_x = current_pose.pose.position.x + tmp_dy * 1.0/coefficient;
          tmp_x = current_pose.pose.position.x + tmp_dx;
          tmp_y = current_pose.pose.position.y + tmp_dx * coefficient;
          next_pose.pose.position.x = tmp_x;
          next_pose.pose.position.y = tmp_y;
        }
      }
      */
     int points_num = 15;
     int plan_size = transformed_plan.size();
     if((plan_size < points_num && plan_size > 1) || no_more_teb_plan){
       no_more_teb_plan = true;
       ROS_INFO("no_more_teb_plan: %d",no_more_teb_plan);
      //  planner_->clearPlanner();//lyy
      //  geometry_msgs::PoseStamped front_pose = transformed_plan.front();
       geometry_msgs::PoseStamped back_pose = transformed_plan.back();
       double final_deltax = back_pose.pose.position.x - robot_pose_.x();
       double final_deltay = back_pose.pose.position.y - robot_pose_.y();
      //  if(!final_th_init)
       {
         final_th = atan2(final_deltay,final_deltax);
        //  final_th_init = true;
       }
      /* //local plan points interpolation
       transformed_plan.clear();
       transformed_plan.push_back(front_pose);
       geometry_msgs::PoseStamped tmp;
       for(int i=1; i<plan_size; i++){
         tmp.pose.position.x = front_pose.pose.position.x + final_deltax*i/(plan_size-1);
         tmp.pose.position.y = front_pose.pose.position.y + final_deltay*i/(plan_size-1);
         tmp.pose.orientation.x = 0.0;
         tmp.pose.orientation.y = 0.0;
         tmp.pose.orientation.z = sin(0.5*final_th);
         tmp.pose.orientation.w = cos(0.5*final_th);
        //  transformed_plan.at(i) = tmp;
         transformed_plan.push_back(tmp);
         front_pose = tmp;
         printf("x: %f,y: %f\n",transformed_plan.at(i).pose.position.x,
                                transformed_plan.at(i).pose.position.y);
       }
       */
        
        cmd_vel.linear.x = 0.0;
        cmd_vel.linear.y = 0.0;
        cmd_vel.angular.z = 0.0;
        double robot_yaw = my_yaw/*tf::getYaw(robot_pose.getRotation())*/;
        double final_delta_angle = final_th-robot_yaw;
        ROS_INFO("final_delta_angle: %f, bx: %f, by: %f, rx: %f, ry: %f, rz: %f",
                  final_delta_angle, back_pose.pose.position.x, back_pose.pose.position.y,
                  robot_pose_.x(), robot_pose_.y(), robot_yaw);
        
        if(!my_rotate_adjustment(final_delta_angle, my_mid_rotate, cmd_vel, 0.002) || my_mid_rotate){
          // double footprint_cost = costmap_model_.get()->footprintCost(robot_pose_.x(), robot_pose_.y(), robot_pose_.theta(),
          //     footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);
          // ROS_INFO("footprint_cost: %f,robot_inscribed_radius_: %f,robot_circumscribed_radius: %f",
          //                 footprint_cost, robot_inscribed_radius_, robot_circumscribed_radius);
          // if(footprint_cost < 254)
          {
            //target point is free now, start moving
            cmd_vel.linear.x = 0.1;
            cmd_vel.linear.y = 0.0;
            cmd_vel.angular.z = 0.0;
            final_goal_check = true;
          }
          // else{
          //   //target point is occupied, stop now
          //   cmd_vel.linear.x = 0.0;
          //   cmd_vel.linear.y = 0.0;
          //   cmd_vel.angular.z = 0.0;
          //   ROS_WARN("Current robot footprint not free, don't start middlely!!");
          //   return true;
          // }
          ROS_INFO("finish fffffffffffinal_delta_angle: %f, plan_size: %d\n",final_delta_angle,plan_size);
        }else{
          ROS_INFO("not finish ffffffffffffinal_delta_angle: %f, plan_size: %d\n",final_delta_angle,plan_size);
        }
        
        return true;
     }
    }else{
      approaching_msg.data = false;
      approaching_goal_pub_.publish(approaching_msg);
    }

    // check if we should enter any backup mode and apply settings
    configureBackupModes(transformed_plan, goal_idx);

    // Return false if the transformed global plan is empty
    if (transformed_plan.empty())
    {
      ROS_WARN("Transformed plan is empty. Cannot determine a local plan.");
      return false;
    }

    // Get current goal point (last point of the transformed plan)
    tf::Stamped<tf::Pose> goal_point;
    tf::poseStampedMsgToTF(transformed_plan.back(), goal_point);
    robot_goal_.x() = goal_point.getOrigin().getX();
    robot_goal_.y() = goal_point.getOrigin().getY();
    if (cfg_.trajectory.global_plan_overwrite_orientation)
    {
      robot_goal_.theta() = estimateLocalGoalOrientation(global_plan_, goal_point, goal_idx, tf_plan_to_global);
      // overwrite/update goal orientation of the transformed plan with the actual goal (enable using the plan as initialization)
      transformed_plan.back().pose.orientation = tf::createQuaternionMsgFromYaw(robot_goal_.theta());
    }
    else
    {
      robot_goal_.theta() = tf::getYaw(goal_point.getRotation());
    }

    // overwrite/update start of the transformed plan with the actual robot position (allows using the plan as initial trajectory)
    if (transformed_plan.size() == 1) // plan only contains the goal
    {
      transformed_plan.insert(transformed_plan.begin(), geometry_msgs::PoseStamped()); // insert start (not yet initialized)
    }
    tf::poseTFToMsg(robot_pose, transformed_plan.front().pose); // update start;

    // clear currently existing obstacles
    obstacles_.clear();

    // Update obstacle container with costmap information or polygons provided by a costmap_converter plugin
    if (costmap_converter_)
      updateObstacleContainerWithCostmapConverter();
    else
      updateObstacleContainerWithCostmap();

    // also consider custom obstacles (must be called after other updates, since the container is not cleared)
    updateObstacleContainerWithCustomObstacles();

    // Do not allow config changes during the following optimization step
    boost::mutex::scoped_lock cfg_lock(cfg_.configMutex());

    // Now perform the actual planning
    //   bool success = planner_->plan(robot_pose_, robot_goal_, robot_vel_, cfg_.goal_tolerance.free_goal_vel); // straight line init
    bool success = planner_->plan(transformed_plan, &robot_vel_, cfg_.goal_tolerance.free_goal_vel);
    if (!success)
    {
      planner_->clearPlanner(); // force reinitialization for next time
      ROS_WARN("teb_local_planner was not able to obtain a local plan for the current setting.");

      ++no_infeasible_plans_; // increase number of infeasible solutions in a row
      time_last_infeasible_plan_ = ros::Time::now();
      last_cmd_ = cmd_vel;
      return false;
    }

    // Check feasibility (but within the first few states only)
    if (cfg_.robot.is_footprint_dynamic)
    {
      // Update footprint of the robot and minimum and maximum distance from the center of the robot to its footprint vertices.
      footprint_spec_ = costmap_ros_->getRobotFootprint();
      costmap_2d::calculateMinAndMaxDistances(footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius);
    }

    bool feasible = planner_->isTrajectoryFeasible(costmap_model_.get(), footprint_spec_, robot_inscribed_radius_, robot_circumscribed_radius, cfg_.trajectory.feasibility_check_no_poses);
    if (!feasible)
    {
      cmd_vel.linear.x = 0;
      cmd_vel.linear.y = 0;
      cmd_vel.angular.z = 0;

      // now we reset everything to start again with the initialization of new trajectories.
      planner_->clearPlanner();
      ROS_WARN("TebLocalPlannerROS: trajectory is not feasible. Resetting planner...");

      ++no_infeasible_plans_; // increase number of infeasible solutions in a row
      time_last_infeasible_plan_ = ros::Time::now();
      last_cmd_ = cmd_vel;
      return false;
    }

    // Get the velocity command for this sampling interval
    if (!planner_->getVelocityCommand(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z, cfg_.trajectory.control_look_ahead_poses))
    {
      planner_->clearPlanner();
      ROS_WARN("TebLocalPlannerROS: velocity command invalid. Resetting planner...");
      ++no_infeasible_plans_; // increase number of infeasible solutions in a row
      time_last_infeasible_plan_ = ros::Time::now();
      last_cmd_ = cmd_vel;
      return false;
    }

    // Saturate velocity, if the optimization results violates the constraints (could be possible due to soft constraints).
    saturateVelocity(cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z, cfg_.robot.max_vel_x, cfg_.robot.max_vel_y,
                     cfg_.robot.max_vel_theta, cfg_.robot.max_vel_x_backwards);

    // convert rot-vel to steering angle if desired (carlike robot).
    // The min_turning_radius is allowed to be slighly smaller since it is a soft-constraint
    // and opposed to the other constraints not affected by penalty_epsilon. The user might add a safety margin to the parameter itself.
    if (cfg_.robot.cmd_angle_instead_rotvel)
    {
      cmd_vel.angular.z = convertTransRotVelToSteeringAngle(cmd_vel.linear.x, cmd_vel.angular.z, cfg_.robot.wheelbase, 0.95 * cfg_.robot.min_turning_radius);
      if (!std::isfinite(cmd_vel.angular.z))
      {
        cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.angular.z = 0;
        last_cmd_ = cmd_vel;
        planner_->clearPlanner();
        ROS_WARN("TebLocalPlannerROS: Resulting steering angle is not finite. Resetting planner...");
        ++no_infeasible_plans_; // increase number of infeasible solutions in a row
        time_last_infeasible_plan_ = ros::Time::now();
        return false;
      }
    }

    // a feasible solution should be found, reset counter
    no_infeasible_plans_ = 0;

    // store last command (for recovery analysis etc.)
    last_cmd_ = cmd_vel;

    // Now visualize everything
    planner_->visualize();
    visualization_->publishObstacles(obstacles_);
    visualization_->publishViaPoints(via_points_);
    visualization_->publishGlobalPlan(global_plan_);

    //lyy
    if(goal_idx != current_global_plan_size){
      float vel = cmd_vel.linear.x;
      if(!lpf_init_flag){
        lpf_init_flag = true;
        last_vel_x = vel;
      }
      cmd_vel.linear.x = vel_lpf(vel, last_vel_x, 0.5);
    }

    return true;
  }
  
  //zy
  void TebLocalPlannerROS::adjust_transformed_plan(std::vector<geometry_msgs::PoseStamped> &transformed_plan, float robot_current_angle)
  {
    // printf("transformed plan size: %d\n", transformed_plan.size());
    int s = transformed_plan.size();
    double delta_y = transformed_plan[s-1].pose.position.y - transformed_plan[0].pose.position.y;
    double delta_x = transformed_plan[s-1].pose.position.x - transformed_plan[0].pose.position.x;
    double yaw_diff = atan2(delta_y, delta_x);
    // double yaw_diff = robot_current_angle;
    
    printf("yaw_diff: %f degree\n", yaw_diff*57.3f);
    double pitch_diff = 0.0;
    double roll_diff = 0.0;
    tf::Quaternion q;
    q.setRPY(roll_diff, pitch_diff, yaw_diff);
    // printf("q.x: %f, y: %f, z: %f, w: %f\n", q.x(), q.y(), q.z(), q.w());

    transformed_plan[0].pose.orientation.x = q.x();
    transformed_plan[0].pose.orientation.y = q.y();
    transformed_plan[0].pose.orientation.z = q.z();
    transformed_plan[0].pose.orientation.w = q.w();

    transformed_plan[s-1].pose.orientation.x = q.x();
    transformed_plan[s-1].pose.orientation.y = q.y();
    transformed_plan[s-1].pose.orientation.z = q.z();
    transformed_plan[s-1].pose.orientation.w = q.w();

  }

  bool TebLocalPlannerROS::isGoalReached()
  {
    if (goal_reached_)
    {
      ROS_INFO("GOAL Reached!");
      planner_->clearPlanner();
      return true;
    }
    return false;
  }

  void TebLocalPlannerROS::updateObstacleContainerWithCostmap()
  {
    // Add costmap obstacles if desired
    if (cfg_.obstacles.include_costmap_obstacles)
    {
      Eigen::Vector2d robot_orient = robot_pose_.orientationUnitVec();

      for (unsigned int i = 0; i < costmap_->getSizeInCellsX() - 1; ++i)
      {
        for (unsigned int j = 0; j < costmap_->getSizeInCellsY() - 1; ++j)
        {
          if (costmap_->getCost(i, j) == costmap_2d::LETHAL_OBSTACLE)
          {
            Eigen::Vector2d obs;
            costmap_->mapToWorld(i, j, obs.coeffRef(0), obs.coeffRef(1));

            // check if obstacle is interesting (e.g. not far behind the robot)
            Eigen::Vector2d obs_dir = obs - robot_pose_.position();
            if (obs_dir.dot(robot_orient) < 0 && obs_dir.norm() > cfg_.obstacles.costmap_obstacles_behind_robot_dist)
              continue;

            obstacles_.push_back(ObstaclePtr(new PointObstacle(obs)));
          }
        }
      }
    }
  }

  void TebLocalPlannerROS::updateObstacleContainerWithCostmapConverter()
  {
    if (!costmap_converter_)
      return;

    //Get obstacles from costmap converter
    costmap_converter::ObstacleArrayConstPtr obstacles = costmap_converter_->getObstacles();
    if (!obstacles)
      return;

    for (std::size_t i = 0; i < obstacles->obstacles.size(); ++i)
    {
      const costmap_converter::ObstacleMsg *obstacle = &obstacles->obstacles.at(i);
      const geometry_msgs::Polygon *polygon = &obstacle->polygon;

      if (polygon->points.size() == 1 && obstacle->radius > 0) // Circle
      {
        obstacles_.push_back(ObstaclePtr(new CircularObstacle(polygon->points[0].x, polygon->points[0].y, obstacle->radius)));
      }
      else if (polygon->points.size() == 1) // Point
      {
        obstacles_.push_back(ObstaclePtr(new PointObstacle(polygon->points[0].x, polygon->points[0].y)));
      }
      else if (polygon->points.size() == 2) // Line
      {
        obstacles_.push_back(ObstaclePtr(new LineObstacle(polygon->points[0].x, polygon->points[0].y,
                                                          polygon->points[1].x, polygon->points[1].y)));
      }
      else if (polygon->points.size() > 2) // Real polygon
      {
        PolygonObstacle *polyobst = new PolygonObstacle;
        for (std::size_t j = 0; j < polygon->points.size(); ++j)
        {
          polyobst->pushBackVertex(polygon->points[j].x, polygon->points[j].y);
        }
        polyobst->finalizePolygon();
        obstacles_.push_back(ObstaclePtr(polyobst));
      }

      // Set velocity, if obstacle is moving
      if (!obstacles_.empty())
        obstacles_.back()->setCentroidVelocity(obstacles->obstacles[i].velocities, obstacles->obstacles[i].orientation);
    }
  }

  void TebLocalPlannerROS::updateObstacleContainerWithCustomObstacles()
  {
    // Add custom obstacles obtained via message
    boost::mutex::scoped_lock l(custom_obst_mutex_);

    if (!custom_obstacle_msg_.obstacles.empty())
    {
      // We only use the global header to specify the obstacle coordinate system instead of individual ones
      Eigen::Affine3d obstacle_to_map_eig;
      try
      {
        tf::StampedTransform obstacle_to_map;
        tf_->waitForTransform(global_frame_, ros::Time(0),
                              custom_obstacle_msg_.header.frame_id, ros::Time(0),
                              custom_obstacle_msg_.header.frame_id, ros::Duration(0.5));
        tf_->lookupTransform(global_frame_, ros::Time(0),
                             custom_obstacle_msg_.header.frame_id, ros::Time(0),
                             custom_obstacle_msg_.header.frame_id, obstacle_to_map);
        tf::transformTFToEigen(obstacle_to_map, obstacle_to_map_eig);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s", ex.what());
        obstacle_to_map_eig.setIdentity();
      }

      for (size_t i = 0; i < custom_obstacle_msg_.obstacles.size(); ++i)
      {
        if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1 && custom_obstacle_msg_.obstacles.at(i).radius > 0) // circle
        {
          Eigen::Vector3d pos(custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                              custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                              custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z);
          obstacles_.push_back(ObstaclePtr(new CircularObstacle((obstacle_to_map_eig * pos).head(2), custom_obstacle_msg_.obstacles.at(i).radius)));
        }
        else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 1) // point
        {
          Eigen::Vector3d pos(custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                              custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                              custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z);
          obstacles_.push_back(ObstaclePtr(new PointObstacle((obstacle_to_map_eig * pos).head(2))));
        }
        else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.size() == 2) // line
        {
          Eigen::Vector3d line_start(custom_obstacle_msg_.obstacles.at(i).polygon.points.front().x,
                                     custom_obstacle_msg_.obstacles.at(i).polygon.points.front().y,
                                     custom_obstacle_msg_.obstacles.at(i).polygon.points.front().z);
          Eigen::Vector3d line_end(custom_obstacle_msg_.obstacles.at(i).polygon.points.back().x,
                                   custom_obstacle_msg_.obstacles.at(i).polygon.points.back().y,
                                   custom_obstacle_msg_.obstacles.at(i).polygon.points.back().z);
          obstacles_.push_back(ObstaclePtr(new LineObstacle((obstacle_to_map_eig * line_start).head(2),
                                                            (obstacle_to_map_eig * line_end).head(2))));
        }
        else if (custom_obstacle_msg_.obstacles.at(i).polygon.points.empty())
        {
          ROS_WARN("Invalid custom obstacle received. List of polygon vertices is empty. Skipping...");
          continue;
        }
        else // polygon
        {
          PolygonObstacle *polyobst = new PolygonObstacle;
          for (size_t j = 0; j < custom_obstacle_msg_.obstacles.at(i).polygon.points.size(); ++j)
          {
            Eigen::Vector3d pos(custom_obstacle_msg_.obstacles.at(i).polygon.points[j].x,
                                custom_obstacle_msg_.obstacles.at(i).polygon.points[j].y,
                                custom_obstacle_msg_.obstacles.at(i).polygon.points[j].z);
            polyobst->pushBackVertex((obstacle_to_map_eig * pos).head(2));
          }
          polyobst->finalizePolygon();
          obstacles_.push_back(ObstaclePtr(polyobst));
        }

        // Set velocity, if obstacle is moving
        if (!obstacles_.empty())
          obstacles_.back()->setCentroidVelocity(custom_obstacle_msg_.obstacles[i].velocities, custom_obstacle_msg_.obstacles[i].orientation);
      }
    }
  }

  void TebLocalPlannerROS::updateViaPointsContainer(const std::vector<geometry_msgs::PoseStamped> &transformed_plan, double min_separation)
  {
    via_points_.clear();

    if (min_separation <= 0)
      // printf("global_plan_via_point_seq: %.2f \n", min_separation);
      return;

    std::size_t prev_idx = 0;
    for (std::size_t i = 1; i < transformed_plan.size(); ++i) // skip first one, since we do not need any point before the first min_separation [m]
    {
      // check separation to the previous via-point inserted
      if (distance_points2d(transformed_plan[prev_idx].pose.position, transformed_plan[i].pose.position) < min_separation)
        continue;

      // add via-point
      via_points_.push_back(Eigen::Vector2d(transformed_plan[i].pose.position.x, transformed_plan[i].pose.position.y));
      prev_idx = i;
    }
  }

  Eigen::Vector2d TebLocalPlannerROS::tfPoseToEigenVector2dTransRot(const tf::Pose &tf_vel)
  {
    Eigen::Vector2d vel;
    vel.coeffRef(0) = std::sqrt(tf_vel.getOrigin().getX() * tf_vel.getOrigin().getX() + tf_vel.getOrigin().getY() * tf_vel.getOrigin().getY());
    vel.coeffRef(1) = tf::getYaw(tf_vel.getRotation());
    return vel;
  }

  bool TebLocalPlannerROS::pruneGlobalPlan(const tf::TransformListener &tf, const tf::Stamped<tf::Pose> &global_pose, 
                                           std::vector<geometry_msgs::PoseStamped> &global_plan, double dist_behind_robot)
  {
    if (global_plan.empty())
      return true;

    try
    {
      // transform robot pose into the plan frame (we do not wait here, since pruning not crucial, if missed a few times)
      tf::StampedTransform global_to_plan_transform;
      tf.lookupTransform(global_plan.front().header.frame_id, global_pose.frame_id_, ros::Time(0), global_to_plan_transform);
      tf::Stamped<tf::Pose> robot;
      robot.setData(global_to_plan_transform * global_pose);

      double dist_thresh_sq = dist_behind_robot * dist_behind_robot;

      // iterate plan until a pose close the robot is found
      std::vector<geometry_msgs::PoseStamped>::iterator it = global_plan.begin();
      std::vector<geometry_msgs::PoseStamped>::iterator erase_end = it;
      while (it != global_plan.end())
      {
        double dx = robot.getOrigin().x() - it->pose.position.x;
        double dy = robot.getOrigin().y() - it->pose.position.y;
        double dist_sq = dx * dx + dy * dy;
        if (dist_sq < dist_thresh_sq)
        {
          erase_end = it;
          break;
        }
        ++it;
      }
      if (erase_end == global_plan.end())
        return false;

      if (erase_end != global_plan.begin())
        global_plan.erase(global_plan.begin(), erase_end);

      // //copy erased plan
      // std::vector<geometry_msgs::PoseStamped> erase_plan;
      // erase_plan.assign(global_plan.begin(), erase_end);
      if (parseCrucialPointSuccess)
      {
        for (int i = 0; i < crucialPointContiner.size() && crucialPointContiner.size() > 0; ++i)
        {
          double x_diff = global_plan[0].pose.position.x - crucialPointContiner[i][0];
          double y_diff = global_plan[0].pose.position.y - crucialPointContiner[i][1];
          double dist = x_diff * x_diff + y_diff * y_diff;

          if (dist < 1) 
          {
            if ( via_point.poses.size() > 1)
            {
            //printf("pass crucial point, clean the via point! \n");
            //printf("%d crucial point \n", i);
            //erase_via_point = i;
            via_points_.clear();
            via_point.poses.erase(via_point.poses.begin()); //闁告帞濞€濞呭骸顔忛懠顒傜梾闁告瑦鍨电粩鐑芥儍閸曨偅鍎х€殿喗娲滈崑锟�
            crucialPointContiner.erase(crucialPointContiner.begin());
            pub_via_point = n.advertise<nav_msgs::Path>("via_points", 1, true);
            nav_msgs::Path first_via;
            geometry_msgs::PoseStamped first_point = via_point.poses[0]; //閻犱警鍨扮欢鐐寸▔婵犲偆娲ら柡瀣矋濠€浣瑰緞濮橆偊鍤媣ia point 闁稿繐鐗嗚ぐ鍌涚▔閳ь剚绋夐搴ｇ闂侇剙鐏濋崢銈夋倷闁稓顓洪梻鍌氼嚟濞村绂嶉幒鎴濐殯闁跨噦鎷�?            first_via.poses.push_back(first_point);
            
            pub_via_point.publish(first_via);
          }
          else
          {
           via_points_.clear();
          }
        }
       }
      }
    }
    catch (const tf::TransformException &ex)
    {
      ROS_DEBUG("Cannot prune path since no transform is available: %s\n", ex.what());
      return false;
    }
    return true;
  }

  bool TebLocalPlannerROS::transformGlobalPlan(const tf::TransformListener &tf, const std::vector<geometry_msgs::PoseStamped> &global_plan,
                                               const tf::Stamped<tf::Pose> &global_pose, const costmap_2d::Costmap2D &costmap, const std::string &global_frame, double max_plan_length,
                                               std::vector<geometry_msgs::PoseStamped> &transformed_plan, int *current_goal_idx, tf::StampedTransform *tf_plan_to_global) const
  {
    // this method is a slightly modified version of base_local_planner/goal_functions.h

    const geometry_msgs::PoseStamped &plan_pose = global_plan[0];

    transformed_plan.clear();

    try
    {
      if (global_plan.empty())
      {
        ROS_ERROR("Received plan with zero length");
        *current_goal_idx = 0;
        return false;
      }

      // get plan_to_global_transform from plan frame to global_frame
      tf::StampedTransform plan_to_global_transform;
      tf.waitForTransform(global_frame, ros::Time::now(),
                          plan_pose.header.frame_id, plan_pose.header.stamp,
                          plan_pose.header.frame_id, ros::Duration(0.5));
      tf.lookupTransform(global_frame, ros::Time(),
                         plan_pose.header.frame_id, plan_pose.header.stamp,
                         plan_pose.header.frame_id, plan_to_global_transform);

      //let's get the pose of the robot in the frame of the plan
      tf::Stamped<tf::Pose> robot_pose;
      tf.transformPose(plan_pose.header.frame_id, global_pose, robot_pose);

      //we'll discard points on the plan that are outside the local costmap
      double dist_threshold = std::max(costmap.getSizeInCellsX() * costmap.getResolution() / 2.0,
                                       costmap.getSizeInCellsY() * costmap.getResolution() / 2.0);
      dist_threshold *= 0.85; // just consider 85% of the costmap size to better incorporate point obstacle that are
                              // located on the border of the local costmap

      int i = 0;
      double sq_dist_threshold = dist_threshold * dist_threshold;
      double sq_dist = 1e10;

      //we need to loop to a point on the plan that is within a certain distance of the robot
      for (int j = 0; j < (int)global_plan.size(); ++j)
      {
        double x_diff = robot_pose.getOrigin().x() - global_plan[j].pose.position.x;
        double y_diff = robot_pose.getOrigin().y() - global_plan[j].pose.position.y;
        double new_sq_dist = x_diff * x_diff + y_diff * y_diff;
        if (new_sq_dist > sq_dist_threshold)
          break; // force stop if we have reached the costmap border

        if (new_sq_dist < sq_dist) // find closest distance
        {
          sq_dist = new_sq_dist;
          i = j;
        }
      }

      tf::Stamped<tf::Pose> tf_pose;
      geometry_msgs::PoseStamped newer_pose;

      double plan_length = 0; // check cumulative Euclidean distance along the plan

      //now we'll transform until points are outside of our distance threshold
      while (i < (int)global_plan.size() && sq_dist <= sq_dist_threshold && (max_plan_length <= 0 || plan_length <= max_plan_length))
      {
        const geometry_msgs::PoseStamped &pose = global_plan[i];
        tf::poseStampedMsgToTF(pose, tf_pose);
        tf_pose.setData(plan_to_global_transform * tf_pose);
        tf_pose.stamp_ = plan_to_global_transform.stamp_;
        tf_pose.frame_id_ = global_frame;
        tf::poseStampedTFToMsg(tf_pose, newer_pose);

        transformed_plan.push_back(newer_pose);

        double x_diff = robot_pose.getOrigin().x() - global_plan[i].pose.position.x;
        double y_diff = robot_pose.getOrigin().y() - global_plan[i].pose.position.y;
        sq_dist = x_diff * x_diff + y_diff * y_diff;

        // caclulate distance to previous pose
        if (i > 0 && max_plan_length > 0)
          plan_length += distance_points2d(global_plan[i - 1].pose.position, global_plan[i].pose.position);

        ++i;
      }

      // if we are really close to the goal (<sq_dist_threshold) and the goal is not yet reached (e.g. orientation error >>0)
      // the resulting transformed plan can be empty. In that case we explicitly inject the global goal.
      if (transformed_plan.empty())
      {
        tf::poseStampedMsgToTF(global_plan.back(), tf_pose);
        tf_pose.setData(plan_to_global_transform * tf_pose);
        tf_pose.stamp_ = plan_to_global_transform.stamp_;
        tf_pose.frame_id_ = global_frame;
        tf::poseStampedTFToMsg(tf_pose, newer_pose);

        transformed_plan.push_back(newer_pose);

        // Return the index of the current goal point (inside the distance threshold)
        if (current_goal_idx)
          *current_goal_idx = int(global_plan.size()) - 1;
      }
      else
      {
        // Return the index of the current goal point (inside the distance threshold)
        if (current_goal_idx)
          *current_goal_idx = i - 1; // subtract 1, since i was increased once before leaving the loop
      }

      // Return the transformation from the global plan to the global planning frame if desired
      if (tf_plan_to_global)
        *tf_plan_to_global = plan_to_global_transform;
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
        ROS_ERROR("Global Frame: %s Plan Frame size %d: %s\n", global_frame.c_str(), (unsigned int)global_plan.size(), global_plan[0].header.frame_id.c_str());

      return false;
    }

    return true;
  }

  double TebLocalPlannerROS::estimateLocalGoalOrientation(const std::vector<geometry_msgs::PoseStamped> &global_plan, const tf::Stamped<tf::Pose> &local_goal,
                                                          int current_goal_idx, const tf::StampedTransform &tf_plan_to_global, int moving_average_length) const
  {
    int n = (int)global_plan.size();

    // check if we are near the global goal already
    if (current_goal_idx > n - moving_average_length - 2)
    {
      if (current_goal_idx >= n - 1) // we've exactly reached the goal
      {
        return tf::getYaw(local_goal.getRotation());
      }
      else
      {
        tf::Quaternion global_orientation;
        tf::quaternionMsgToTF(global_plan.back().pose.orientation, global_orientation);
        return tf::getYaw(tf_plan_to_global.getRotation() * global_orientation);
      }
    }

    // reduce number of poses taken into account if the desired number of poses is not available
    moving_average_length = std::min(moving_average_length, n - current_goal_idx - 1); // maybe redundant, since we have checked the vicinity of the goal before

    std::vector<double> candidates;
    tf::Stamped<tf::Pose> tf_pose_k = local_goal;
    tf::Stamped<tf::Pose> tf_pose_kp1;

    int range_end = current_goal_idx + moving_average_length;
    for (int i = current_goal_idx; i < range_end; ++i)
    {
      // Transform pose of the global plan to the planning frame
      const geometry_msgs::PoseStamped &pose = global_plan.at(i + 1);
      tf::poseStampedMsgToTF(pose, tf_pose_kp1);
      tf_pose_kp1.setData(tf_plan_to_global * tf_pose_kp1);

      // calculate yaw angle
      candidates.push_back(std::atan2(tf_pose_kp1.getOrigin().getY() - tf_pose_k.getOrigin().getY(),
                                      tf_pose_kp1.getOrigin().getX() - tf_pose_k.getOrigin().getX()));

      if (i < range_end - 1)
        tf_pose_k = tf_pose_kp1;
    }
    return average_angles(candidates);
  }

  void TebLocalPlannerROS::saturateVelocity(double &vx, double &vy, double &omega, double max_vel_x, double max_vel_y, double max_vel_theta, double max_vel_x_backwards) const
  {
    // Limit translational velocity for forward driving
    if (vx > max_vel_x)
      vx = max_vel_x;

    // limit strafing velocity
    if (vy > max_vel_y)
      vy = max_vel_y;
    else if (vy < -max_vel_y)
      vy = -max_vel_y;

    // Limit angular velocity
    if (omega > max_vel_theta)
      omega = max_vel_theta;
    else if (omega < -max_vel_theta)
      omega = -max_vel_theta;

    // Limit backwards velocity
    if (max_vel_x_backwards <= 0)
    {
      ROS_WARN_ONCE("TebLocalPlannerROS(): Do not choose max_vel_x_backwards to be <=0. Disable backwards driving by increasing the optimization weight for penalyzing backwards driving.");
    }
    else if (vx < -max_vel_x_backwards)
      vx = -max_vel_x_backwards;
      
    if(fabs(omega) < 0.12) omega = 0.0;//lyy 
  }

  double TebLocalPlannerROS::convertTransRotVelToSteeringAngle(double v, double omega, double wheelbase, double min_turning_radius) const
  {
    if (omega == 0 || v == 0)
      return 0;

    double radius = v / omega;

    if (fabs(radius) < min_turning_radius)
      radius = double(g2o::sign(radius)) * min_turning_radius;

    return std::atan(wheelbase / radius);
  }

  void TebLocalPlannerROS::validateFootprints(double opt_inscribed_radius, double costmap_inscribed_radius, double min_obst_dist)
  {
    ROS_WARN_COND(opt_inscribed_radius + min_obst_dist < costmap_inscribed_radius,
                  "The inscribed radius of the footprint specified for TEB optimization (%f) + min_obstacle_dist (%f) are smaller "
                  "than the inscribed radius of the robot's footprint in the costmap parameters (%f, including 'footprint_padding'). "
                  "Infeasible optimziation results might occur frequently!",
                  opt_inscribed_radius, min_obst_dist, costmap_inscribed_radius);
  }

  void TebLocalPlannerROS::configureBackupModes(std::vector<geometry_msgs::PoseStamped> &transformed_plan, int &goal_idx)
  {
    ros::Time current_time = ros::Time::now();

    // reduced horizon backup mode
    if (cfg_.recovery.shrink_horizon_backup &&
        goal_idx < (int)transformed_plan.size() - 1 &&                                                                                 // we do not reduce if the goal is already selected (because the orientation might change -> can introduce oscillations)
        (no_infeasible_plans_ > 0 || (current_time - time_last_infeasible_plan_).toSec() < cfg_.recovery.shrink_horizon_min_duration)) // keep short horizon for at least a few seconds
    {
      ROS_INFO_COND(no_infeasible_plans_ == 1, "Activating reduced horizon backup mode for at least %.2f sec (infeasible trajectory detected).", cfg_.recovery.shrink_horizon_min_duration);

      // Shorten horizon if requested
      // reduce to 50 percent:
      int horizon_reduction = goal_idx / 2;

      if (no_infeasible_plans_ > 9)
      {
        ROS_INFO_COND(no_infeasible_plans_ == 10, "Infeasible trajectory detected 10 times in a row: further reducing horizon...");
        horizon_reduction /= 2;
      }

      // we have a small overhead here, since we already transformed 50% more of the trajectory.
      // But that's ok for now, since we do not need to make transformGlobalPlan more complex
      // and a reduced horizon should occur just rarely.
      int new_goal_idx_transformed_plan = int(transformed_plan.size()) - horizon_reduction - 1;
      goal_idx -= horizon_reduction;
      if (new_goal_idx_transformed_plan > 0 && goal_idx >= 0)
        transformed_plan.erase(transformed_plan.begin() + new_goal_idx_transformed_plan, transformed_plan.end());
      else
        goal_idx += horizon_reduction; // this should not happen, but safety first ;-)
    }

    // detect and resolve oscillations
    if (cfg_.recovery.oscillation_recovery)
    {
      double max_vel_theta;
      double max_vel_current = last_cmd_.linear.x >= 0 ? cfg_.robot.max_vel_x : cfg_.robot.max_vel_x_backwards;
      if (cfg_.robot.min_turning_radius != 0 && max_vel_current > 0)
        max_vel_theta = std::max(max_vel_current / std::abs(cfg_.robot.min_turning_radius), cfg_.robot.max_vel_theta);
      else
        max_vel_theta = cfg_.robot.max_vel_theta;

      failure_detector_.update(last_cmd_, cfg_.robot.max_vel_x, cfg_.robot.max_vel_x_backwards, max_vel_theta,
                               cfg_.recovery.oscillation_v_eps, cfg_.recovery.oscillation_omega_eps);

      bool oscillating = failure_detector_.isOscillating();
      bool recently_oscillated = (ros::Time::now() - time_last_oscillation_).toSec() < cfg_.recovery.oscillation_recovery_min_duration; // check if we have already detected an oscillation recently

      if (oscillating)
      {
        if (!recently_oscillated)
        {
          // save current turning direction
          if (robot_vel_.angular.z > 0)
            last_preferred_rotdir_ = RotType::left;
          else
            last_preferred_rotdir_ = RotType::right;
          ROS_WARN("TebLocalPlannerROS: possible oscillation (of the robot or its local plan) detected. Activating recovery strategy (prefer current turning direction during optimization).");
        }
        time_last_oscillation_ = ros::Time::now();
        planner_->setPreferredTurningDir(last_preferred_rotdir_);
      }
      else if (!recently_oscillated && last_preferred_rotdir_ != RotType::none) // clear recovery behavior
      {
        last_preferred_rotdir_ = RotType::none;
        planner_->setPreferredTurningDir(last_preferred_rotdir_);
        ROS_INFO("TebLocalPlannerROS: oscillation recovery disabled/expired.");
      }
    }
  }

  void TebLocalPlannerROS::customObstacleCB(const costmap_converter::ObstacleArrayMsg::ConstPtr &obst_msg)
  {
    boost::mutex::scoped_lock l(custom_obst_mutex_);
    custom_obstacle_msg_ = *obst_msg;
  }

  //copy custom via points for planning
  void TebLocalPlannerROS::customViaPointsCB(const nav_msgs::Path::ConstPtr &via_points_msg)
  {
    ROS_INFO_ONCE("Via-points received. This message is printed once!");

    if (cfg_.trajectory.global_plan_viapoint_sep > 0)
    {
      ROS_WARN("Via-points are already obtained from the global plan (global_plan_viapoint_sep>0)."
               "Ignoring custom via-points.");
      custom_via_points_active_ = false;
      return;
    }

    boost::mutex::scoped_lock l(via_point_mutex_);
    via_points_.clear();
    for (const geometry_msgs::PoseStamped &pose : via_points_msg->poses)
    {
      via_points_.emplace_back(pose.pose.position.x, pose.pose.position.y);
      //printf("via_point x: %.2f \n via_point y: %.2f \n", pose.pose.position.x, pose.pose.position.y);
    }
    custom_via_points_active_ = !via_points_.empty();
  }

  RobotFootprintModelPtr TebLocalPlannerROS::getRobotFootprintFromParamServer(const ros::NodeHandle &nh)
  {
    std::string model_name;
    if (!nh.getParam("footprint_model/type", model_name))
    {
      ROS_INFO("No robot footprint model specified for trajectory optimization. Using point-shaped model.");
      return boost::make_shared<PointRobotFootprint>();
    }

    // point
    if (model_name.compare("point") == 0)
    {
      ROS_INFO("Footprint model 'point' loaded for trajectory optimization.");
      return boost::make_shared<PointRobotFootprint>();
    }

    // circular
    if (model_name.compare("circular") == 0)
    {
      // get radius
      double radius;
      if (!nh.getParam("footprint_model/radius", radius))
      {
        ROS_ERROR_STREAM("Footprint model 'circular' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                                                                                                                  << "/footprint_model/radius' does not exist. Using point-model instead.");
        return boost::make_shared<PointRobotFootprint>();
      }
      ROS_INFO_STREAM("Footprint model 'circular' (radius: " << radius << "m) loaded for trajectory optimization.");
      return boost::make_shared<CircularRobotFootprint>(radius);
    }

    // line
    if (model_name.compare("line") == 0)
    {
      // check parameters
      if (!nh.hasParam("footprint_model/line_start") || !nh.hasParam("footprint_model/line_end"))
      {
        ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                                                                                                              << "/footprint_model/line_start' and/or '.../line_end' do not exist. Using point-model instead.");
        return boost::make_shared<PointRobotFootprint>();
      }
      // get line coordinates
      std::vector<double> line_start, line_end;
      nh.getParam("footprint_model/line_start", line_start);
      nh.getParam("footprint_model/line_end", line_end);
      if (line_start.size() != 2 || line_end.size() != 2)
      {
        ROS_ERROR_STREAM("Footprint model 'line' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                                                                                                              << "/footprint_model/line_start' and/or '.../line_end' do not contain x and y coordinates (2D). Using point-model instead.");
        return boost::make_shared<PointRobotFootprint>();
      }

      ROS_INFO_STREAM("Footprint model 'line' (line_start: [" << line_start[0] << "," << line_start[1] << "]m, line_end: ["
                                                              << line_end[0] << "," << line_end[1] << "]m) loaded for trajectory optimization.");
      return boost::make_shared<LineRobotFootprint>(Eigen::Map<const Eigen::Vector2d>(line_start.data()), Eigen::Map<const Eigen::Vector2d>(line_end.data()));
    }

    // two circles
    if (model_name.compare("two_circles") == 0)
    {
      // check parameters
      if (!nh.hasParam("footprint_model/front_offset") || !nh.hasParam("footprint_model/front_radius") || !nh.hasParam("footprint_model/rear_offset") || !nh.hasParam("footprint_model/rear_radius"))
      {
        ROS_ERROR_STREAM("Footprint model 'two_circles' cannot be loaded for trajectory optimization, since params '" << nh.getNamespace()
                                                                                                                      << "/footprint_model/front_offset', '.../front_radius', '.../rear_offset' and '.../rear_radius' do not exist. Using point-model instead.");
        return boost::make_shared<PointRobotFootprint>();
      }
      double front_offset, front_radius, rear_offset, rear_radius;
      nh.getParam("footprint_model/front_offset", front_offset);
      nh.getParam("footprint_model/front_radius", front_radius);
      nh.getParam("footprint_model/rear_offset", rear_offset);
      nh.getParam("footprint_model/rear_radius", rear_radius);
      ROS_INFO_STREAM("Footprint model 'two_circles' (front_offset: " << front_offset << "m, front_radius: " << front_radius
                                                                      << "m, rear_offset: " << rear_offset << "m, rear_radius: " << rear_radius << "m) loaded for trajectory optimization.");
      return boost::make_shared<TwoCirclesRobotFootprint>(front_offset, front_radius, rear_offset, rear_radius);
    }

    // polygon
    if (model_name.compare("polygon") == 0)
    {

      // check parameters
      XmlRpc::XmlRpcValue footprint_xmlrpc;
      if (!nh.getParam("footprint_model/vertices", footprint_xmlrpc))
      {
        ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                                                                                                                 << "/footprint_model/vertices' does not exist. Using point-model instead.");
        return boost::make_shared<PointRobotFootprint>();
      }
      // get vertices
      if (footprint_xmlrpc.getType() == XmlRpc::XmlRpcValue::TypeArray)
      {
        try
        {
          Point2dContainer polygon = makeFootprintFromXMLRPC(footprint_xmlrpc, "/footprint_model/vertices");
          ROS_INFO_STREAM("Footprint model 'polygon' loaded for trajectory optimization.");
          return boost::make_shared<PolygonRobotFootprint>(polygon);
        }
        catch (const std::exception &ex)
        {
          ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization: " << ex.what() << ". Using point-model instead.");
          return boost::make_shared<PointRobotFootprint>();
        }
      }
      else
      {
        ROS_ERROR_STREAM("Footprint model 'polygon' cannot be loaded for trajectory optimization, since param '" << nh.getNamespace()
                                                                                                                 << "/footprint_model/vertices' does not define an array of coordinates. Using point-model instead.");
        return boost::make_shared<PointRobotFootprint>();
      }
    }

    // otherwise
    ROS_WARN_STREAM("Unknown robot footprint model specified with parameter '" << nh.getNamespace() << "/footprint_model/type'. Using point model instead.");
    return boost::make_shared<PointRobotFootprint>();
  }

  Point2dContainer TebLocalPlannerROS::makeFootprintFromXMLRPC(XmlRpc::XmlRpcValue &footprint_xmlrpc, const std::string &full_param_name)
  {
    // Make sure we have an array of at least 3 elements.
    if (footprint_xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray ||
        footprint_xmlrpc.size() < 3)
    {
      ROS_FATAL("The footprint must be specified as list of lists on the parameter server, %s was specified as %s",
                full_param_name.c_str(), std::string(footprint_xmlrpc).c_str());
      throw std::runtime_error("The footprint must be specified as list of lists on the parameter server with at least "
                               "3 points eg: [[x1, y1], [x2, y2], ..., [xn, yn]]");
    }

    Point2dContainer footprint;
    Eigen::Vector2d pt;

    for (int i = 0; i < footprint_xmlrpc.size(); ++i)
    {
      // Make sure each element of the list is an array of size 2. (x and y coordinates)
      XmlRpc::XmlRpcValue point = footprint_xmlrpc[i];
      if (point.getType() != XmlRpc::XmlRpcValue::TypeArray ||
          point.size() != 2)
      {
        ROS_FATAL("The footprint (parameter %s) must be specified as list of lists on the parameter server eg: "
                  "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form.",
                  full_param_name.c_str());
        throw std::runtime_error("The footprint must be specified as list of lists on the parameter server eg: "
                                 "[[x1, y1], [x2, y2], ..., [xn, yn]], but this spec is not of that form");
      }

      pt.x() = getNumberFromXMLRPC(point[0], full_param_name);
      pt.y() = getNumberFromXMLRPC(point[1], full_param_name);

      footprint.push_back(pt);
    }
    return footprint;
  }

  double TebLocalPlannerROS::getNumberFromXMLRPC(XmlRpc::XmlRpcValue &value, const std::string &full_param_name)
  {
    // Make sure that the value we're looking at is either a double or an int.
    if (value.getType() != XmlRpc::XmlRpcValue::TypeInt &&
        value.getType() != XmlRpc::XmlRpcValue::TypeDouble)
    {
      std::string &value_string = value;
      ROS_FATAL("Values in the footprint specification (param %s) must be numbers. Found value %s.",
                full_param_name.c_str(), value_string.c_str());
      throw std::runtime_error("Values in the footprint specification must be numbers");
    }
    return value.getType() == XmlRpc::XmlRpcValue::TypeInt ? (int)(value) : (double)(value);
  }

} // end namespace teb_local_planner
