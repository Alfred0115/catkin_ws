/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
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
*   * Neither the name of the Willow Garage nor the names of its
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
*********************************************************************/
#ifndef DEPTH_IMAGE_PROC_DEPTH_CONVERSIONS
#define DEPTH_IMAGE_PROC_DEPTH_CONVERSIONS

#include <sensor_msgs/Image.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <image_geometry/pinhole_camera_model.h>
#include <depth_image_proc/depth_traits.h>

#include <limits>

#include <tf/transform_listener.h>
#include <geometry_msgs/PointStamped.h>


namespace depth_image_proc {

typedef sensor_msgs::PointCloud2 PointCloud;

// Handles float or uint16 depths
template<typename T>
void convert(
    const sensor_msgs::ImageConstPtr& depth_msg,
    PointCloud::Ptr& cloud_msg,
    const image_geometry::PinholeCameraModel& model,
    double range_max = 0.0)
{
  // Use correct principal point from calibration
  float center_x = model.cx();
  float center_y = model.cy();

  // Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
  double unit_scaling = DepthTraits<T>::toMeters( T(1) );
  float constant_x = unit_scaling / model.fx();
  float constant_y = unit_scaling / model.fy();
  float bad_point = std::numeric_limits<float>::quiet_NaN();

  sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
  const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
  int row_step = depth_msg->step / sizeof(T);
  //printf("framd id is: %s", cloud_msg->header.frame_id.c_str());
  for (int v = 0; v < (int)cloud_msg->height; ++v, depth_row += row_step)
  {
    for (int u = 0; u < (int)cloud_msg->width; ++u, ++iter_x, ++iter_y, ++iter_z)
    {
      T depth = depth_row[u];

      double pc_x = (u - center_x) * depth * constant_x;
      double pc_y = (v - center_y) * depth * constant_y;
      double pc_z = DepthTraits<T>::toMeters(depth);

      //当相机朝下看的时候，需要增加一个坐标变换, 主要是为了下面的直通滤波服务
      //用的时候更新两个frame名称即可
      // tf::TransformListener listener;
      // geometry_msgs::PointStamped pose_before;
      // geometry_msgs::PointStamped pose_after;

      // std::string old_frame = "old_frame"; // old frame 实际用的时候要灵活替换成相机向下看的frame
      // std::string new_frame = "new_frame"; // new frame 实际用的时候要灵活替换成水平的frame
      // pose_before.header.frame_id = old_frame;   
      // pose_before.header.stamp = ros::Time();

      // pose_before.point.x = pc_x;
      // pose_before.point.y = pc_y;
      // pose_before.point.z = pc_z;

      // try
      // {
      //   listener.waitForTransform(new_frame, old_frame, ros::Time(0), ros::Duration(3));
      //   listener.transformPoint(new_frame, pose_before, pose_after);
      //   pc_x = pose_after.point.x;
      //   pc_y = pose_after.point.y;
      //   pc_z = pose_after.point.z;
      // }
      // catch(tf::TransformException ex)
      // {
      //   ROS_WARN("transfrom exception : %s", ex.what());
      // }

      if(pc_z > 3.0 || pc_y > 0.5 || pc_z < 0.3 || pc_y < -1.1)   //zy delete points too far or too low   vertex->y 0.26
      {
        continue;
      }


      //std::cout << "pc_y: " << pc_y << std::endl;

      // Missing points denoted by NaNs
      if (!DepthTraits<T>::valid(depth))
      {
        if (range_max != 0.0)
        {
          depth = DepthTraits<T>::fromMeters(range_max);
        }
        else
        {
          *iter_x = *iter_y = *iter_z = bad_point;
          continue;
        }
      }

      // Fill in XYZ
      *iter_x = (u - center_x) * depth * constant_x;
      *iter_y = (v - center_y) * depth * constant_y;
      *iter_z = DepthTraits<T>::toMeters(depth);
    }
  }
}

} // namespace depth_image_proc

#endif
