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
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/pinhole_camera_model.h>
#include <boost/thread.hpp>
#include <depth_image_proc/depth_conversions.h>

#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>

//zy
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/range_image/range_image.h>
#include <pcl/keypoints/narf_keypoint.h>
#include <pcl/features/range_image_border_extractor.h>
#include <pcl/io/pcd_io.h>


namespace depth_image_proc
{

  namespace enc = sensor_msgs::image_encodings;

  class PointCloudXyzNodelet : public nodelet::Nodelet
  {
    // Subscriptions
    boost::shared_ptr<image_transport::ImageTransport> it_;
    image_transport::CameraSubscriber sub_depth_;
    int queue_size_;

    // Publications
    boost::mutex connect_mutex_;
    typedef sensor_msgs::PointCloud2 PointCloud;
    ros::Publisher pub_point_cloud_;

    image_geometry::PinholeCameraModel model_;

    virtual void onInit();

    void connectCb();

    void depthCb(const sensor_msgs::ImageConstPtr &depth_msg,
                 const sensor_msgs::CameraInfoConstPtr &info_msg);

    sensor_msgs::PointCloud2 downSample(const sensor_msgs::PointCloud2ConstPtr &cloud_in);    //zy
    sensor_msgs::PointCloud2 keyPointsExtraction(const sensor_msgs::PointCloud2ConstPtr &cloud_in); //zy
  };

  void PointCloudXyzNodelet::onInit()
  {
    ros::NodeHandle &nh = getNodeHandle();
    ros::NodeHandle &private_nh = getPrivateNodeHandle();
    it_.reset(new image_transport::ImageTransport(nh));

    // Read parameters
    private_nh.param("queue_size", queue_size_, 5);

    // Monitor whether anyone is subscribed to the output
    ros::SubscriberStatusCallback connect_cb = boost::bind(&PointCloudXyzNodelet::connectCb, this);
    // Make sure we don't enter connectCb() between advertising and assigning to pub_point_cloud_
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    pub_point_cloud_ = nh.advertise<PointCloud>("points", 1, connect_cb, connect_cb);
  }

  // Handles (un)subscribing when clients (un)subscribe
  void PointCloudXyzNodelet::connectCb()
  {
    boost::lock_guard<boost::mutex> lock(connect_mutex_);
    if (pub_point_cloud_.getNumSubscribers() == 0)
    {
      sub_depth_.shutdown();
    }
    else if (!sub_depth_)
    {
      image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
      sub_depth_ = it_->subscribeCamera("image_rect", queue_size_, &PointCloudXyzNodelet::depthCb, this, hints);
    }
  }

  void PointCloudXyzNodelet::depthCb(const sensor_msgs::ImageConstPtr &depth_msg,
                                     const sensor_msgs::CameraInfoConstPtr &info_msg)
  {
    PointCloud::Ptr cloud_msg(new PointCloud);
    cloud_msg->header = depth_msg->header;
    cloud_msg->height = depth_msg->height;
    cloud_msg->width = depth_msg->width;
    cloud_msg->is_dense = false;
    cloud_msg->is_bigendian = false;

    sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
    pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

    // Update camera model
    model_.fromCameraInfo(info_msg);

    if (depth_msg->encoding == enc::TYPE_16UC1)
    {
      convert<uint16_t>(depth_msg, cloud_msg, model_);
    }
    else if (depth_msg->encoding == enc::TYPE_32FC1)
    {
      convert<float>(depth_msg, cloud_msg, model_);
    }
    else
    {
      NODELET_ERROR_THROTTLE(5, "Depth image has unsupported encoding [%s]", depth_msg->encoding.c_str());
      return;
    }

    //down sample
    sensor_msgs::PointCloud2ConstPtr cloud_in = boost::make_shared<sensor_msgs::PointCloud2>(*cloud_msg);
    sensor_msgs::PointCloud2 cloud_downSample = downSample(cloud_in); // zy
    //extract key points
    sensor_msgs::PointCloud2ConstPtr cloud_keyPoint = boost::make_shared<sensor_msgs::PointCloud2>(cloud_downSample);
    //std::cout << "keypoint frame id is: " << cloud_keyPoint->header.frame_id << std::endl;
    sensor_msgs::PointCloud2 cloud_output = keyPointsExtraction(cloud_keyPoint);
    pub_point_cloud_.publish(cloud_output);
  }

  sensor_msgs::PointCloud2 PointCloudXyzNodelet::downSample(const sensor_msgs::PointCloud2ConstPtr &cloud_in)
  {
    pcl::PCLPointCloud2 *cloud = new pcl::PCLPointCloud2; //原始的点云的数据格式
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
    pcl::PCLPointCloud2 cloud_filtered;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_no_ground(new pcl::PointCloud<pcl::PointXYZ>);

    // 转化为PCL中的点云的数据格式
    pcl_conversions::toPCL(*cloud_in, *cloud);
    //std::cout << "pc size before downSample filter is : " << cloud->data.size() << std::endl;
    // 进行一个滤波处理
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloudPtr);
    sor.setLeafSize(/*0.03f, 0.05f, 0.05f*/ 0.03f, 0.03f, 0.03f); //设置体素网格的大小  xyz  one :0.04 0.05 0.05
    //sor.setLeafSize (_filter_resolution, _filter_resolution, _filter_resolution);
    sor.filter(cloud_filtered);

    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud_with_ground(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(cloud_filtered, *temp_cloud_with_ground);

    //半径滤波
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_radiusOutlierRemoval(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::RadiusOutlierRemoval<pcl::PointXYZ> pcFilter;
    pcFilter.setInputCloud(temp_cloud_with_ground);
    pcFilter.setRadiusSearch(0.04);      //0.06
    pcFilter.setMinNeighborsInRadius(8); //8
    pcFilter.setNegative(false);
    pcFilter.filter(*cloud_after_radiusOutlierRemoval);

    //save pcd file just for test
    // pcl::io::savePCDFileASCII ("/home/maxtang/test_pcd.pcd", *cloud_after_radiusOutlierRemoval);
    // std::cerr << "Saved " << cloud_after_radiusOutlierRemoval->points.size() << " data points to test_pcd.pcd." << std::endl;

    sensor_msgs::PointCloud2 output; //声明的输出的点云的格式
    pcl::toROSMsg(*cloud_after_radiusOutlierRemoval, output);
    //std::cout << "output.data.size: " <<output.data.size() << std::endl;

    return output;
  }

  sensor_msgs::PointCloud2 PointCloudXyzNodelet::keyPointsExtraction(const sensor_msgs::PointCloud2ConstPtr &cloud_in)
  {
    pcl::PCLPointCloud2 cloud;
    pcl_conversions::toPCL(*cloud_in, cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(cloud, *tmp_point_cloud);
    pcl::PointCloud<pcl::PointXYZ>& point_cloud = *tmp_point_cloud;
    //std::cout << "point cloud in: " << point_cloud.points.size() << " points \n";
    float support_size = 0.05f;
    float noise_level = 0.0;
    float min_range = 0.0f;
    int border_size = 1;
    float angular_resolution = pcl::deg2rad(0.5f);   //0.5
    Eigen::Affine3f scene_sensor_pose (Eigen::Affine3f::Identity ());
    scene_sensor_pose = Eigen::Affine3f(Eigen::Translation3f(point_cloud.sensor_origin_[0],
                                                             point_cloud.sensor_origin_[1],
                                                             point_cloud.sensor_origin_[2])) *
                                        Eigen::Affine3f(point_cloud.sensor_orientation_);
    // Eigen::Affine3f scene_sensor_pose = (Eigen::Affine3f)Eigen::Translation3f(0.0f, 0.0f, 0.0f);
    
    pcl::RangeImage::CoordinateFrame coordinate_frame = pcl::RangeImage::CAMERA_FRAME; //CAMERA_FRAME = 0, LASER_FRAME  = 1
    float max_angle_width = pcl::deg2rad(90.0f);
    float max_angle_height = pcl::deg2rad(180.0f);

    pcl::RangeImage::Ptr range_image_ptr(new pcl::RangeImage);
    pcl::RangeImage& range_image = *range_image_ptr;
    
    range_image.createFromPointCloud (point_cloud, angular_resolution, max_angle_width, max_angle_height,
                                     scene_sensor_pose, coordinate_frame, noise_level, min_range, border_size);
    range_image.setUnseenToMaxRange ();

    pcl::RangeImageBorderExtractor range_image_border_extractor;
    pcl::NarfKeypoint narf_keypoint_detector(&range_image_border_extractor);
    narf_keypoint_detector.setRangeImage(&range_image);
    narf_keypoint_detector.getParameters().support_size = support_size;

    pcl::PointCloud<int> keypoint_indices;
    narf_keypoint_detector.compute(keypoint_indices);
    //std::cout << "Found " << keypoint_indices.size() << " key points.\n";

    pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> &keypoints = *keypoints_ptr;
    keypoints.points.resize(keypoint_indices.size() + 100);
    
    for (std::size_t i = 0; i < keypoint_indices.size(); ++i)
    {
      //double pc_x = range_image[keypoint_indices[i]].getVector3fMap().x();
      double pc_y = range_image[keypoint_indices[i]].getVector3fMap().y();
      //double pc_z = range_image[keypoint_indices[i]].getVector3fMap().z();
      if (pc_y > 0.27)   // || pc_z < 0.3
      {
        Eigen::Vector3f point;
        point[0] = 0.0;
        point[1] = 0.0;
        point[2] = 10.0;
        keypoints[i].getVector3fMap() = point;
        
        continue;
      }
      keypoints[i].getVector3fMap() = range_image[keypoint_indices[i]].getVector3fMap();
      // std::cout << "x: " << range_image[keypoint_indices[i]].getVector3fMap().x() 
      //           << "y: " << range_image[keypoint_indices[i]].getVector3fMap().y()
      //           << "z: " << range_image[keypoint_indices[i]].getVector3fMap().z()
      //           << std::endl;
    }

    for (int i = keypoint_indices.size(); i < (keypoint_indices.size() + 100); i++)
    {
      Eigen::Vector3f point;
      point[0] = -3.47 + (i - keypoint_indices.size()) * 0.0347 * 2;
      point[1] = 0.0;
      point[2] = 3.5;
      keypoints[i].getVector3fMap() = point; 
    }
    keypoints.header.frame_id = cloud_in->header.frame_id;
    //pcl_conversions::toPCL(ros::Time::now(), keypoints.header.stamp);
    pcl_conversions::toPCL(cloud_in->header.stamp, keypoints.header.stamp);
    sensor_msgs::PointCloud2 output; //声明的输出的点云的格式
    pcl::toROSMsg(keypoints, output);
    return output;
  }

} // namespace depth_image_proc

// Register as nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(depth_image_proc::PointCloudXyzNodelet, nodelet::Nodelet);
