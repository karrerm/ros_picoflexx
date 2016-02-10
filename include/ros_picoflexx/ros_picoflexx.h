/*
 * Copyright (c) 2015, Mina  Kamel, ASL, ETH Zurich, Switzerland
 * You can contact the author at <mina.kamel@mavt.ethz.ch>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * created on Oct 1, 2015 7:54:43 AM
 */
#ifndef INCLUDE_ROS_PICOFLEXX_ROS_PICOFLEXX_H_
#define INCLUDE_ROS_PICOFLEXX_ROS_PICOFLEXX_H_

#include <royale.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <thread>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros_picoflexx/PicoFlexxConfig.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

#define DEPTH_AMPLIFICATION 5000.0
class PicoFlexxCamera
{
 public:
  PicoFlexxCamera(royale::CameraManager& _manager, std::string _camera_name);
  ~PicoFlexxCamera();

  void Initialize();

  bool startAcquisition();

  //setters
  void setExposureTime(int _exposure_time);
  void setExposureTimeLimits(int _minimum_exposure_time, int _maximum_exposure_time);
  void setAutoExposureTime(bool _auto_exposure_time);
  void setFrameRate(int _frame_rate);
  void setOperationMode(royale::OperationMode _operation_mode);

 private:
  royale::CameraManager& manager_;
  std::unique_ptr<royale::ICameraDevice> camera_device_;

  std::string camera_id_;
  std::string camera_name_;
  int exposure_time_;
  bool auto_exposure_time_;
  std::string frame_id_;
  std::string operation_mode_;

  ros::NodeHandle nh_;

  void updateOperationMode();
  void updateExposureSettings();


  class DepthDataListener : public royale::IDepthDataListener
  {
   public:
    DepthDataListener(std::string _camera_name)
    {
      ros::NodeHandle nh(_camera_name);

      nh.param<std::string>("topic_name", topic_name_,  "pcl");
      nh.param<std::string>("frame_id", frame_id_, "cam_link");
      nh.param<std::string>("topic_depth", topic_depth_,  "depth/image_raw");
      nh.param<std::string>("topic_confidence",topic_confidence_,"confidence/image_raw");
      nh.param<std::string>("topic_amplitude",topic_amplitude_,"amplitude/image_raw");
      nh.param<std::string>("topic_noise", topic_noise_, "noise/image_raw");

      pcl_publisher_ = nh.advertise<sensor_msgs::PointCloud2>(topic_name_, 1);
      image_transport::ImageTransport image_transport(nh);
      depth_publisher_ = image_transport.advertise(topic_depth_, 1);
      confidence_publisher_ = image_transport.advertise(topic_confidence_,1);
      amplitude_publisher_ = image_transport.advertise(topic_amplitude_,1);
      noise_publisher_ = image_transport.advertise(topic_noise_,1);
      seq_ = 0;
      cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }
    void onNewData(const royale::DepthData *data)
    {

      cloud_->is_dense = false;
      cloud_->height = data->height;
      cloud_->width = data->width;
      cloud_->points.resize(cloud_->height * cloud_->width);
      cv::Mat depth_mat(data->height,data->width,CV_16UC1);
      cv::Mat ampl_mat(data->height, data->width,CV_16UC1);
      cv::Mat conf_mat(data->height,data->width,CV_8UC1);
      cv::Mat noise_mat(data->height, data->width, CV_32FC1);
      unsigned char* conf_mat_ptr;
      unsigned short* ampl_mat_ptr;
      unsigned short* depth_mat_ptr;
      float* noise_mat_ptr;
      int cloud_indx = 0;
      int width = data->width;
      for (int v = 0; v < data->height; v++) {
          depth_mat_ptr = depth_mat.ptr<unsigned short>(v);
          conf_mat_ptr = conf_mat.ptr<unsigned char>(v);
          ampl_mat_ptr = ampl_mat.ptr<unsigned short>(v);
          noise_mat_ptr = noise_mat.ptr<float>(v);
          for (int u = 0; u < data->width; u++) {
              cloud_indx = v*width + u;
		      pcl::PointXYZ& pt = cloud_->points[cloud_indx];
              pt.x = data->points[cloud_indx].x;
              pt.y = data->points[cloud_indx].y;
              pt.z = data->points[cloud_indx].z;
              depth_mat_ptr[u] = (unsigned short)DEPTH_AMPLIFICATION*data->points[cloud_indx].z;
              conf_mat_ptr[u] =  (unsigned char)data->points[cloud_indx].depthConfidence;
              ampl_mat_ptr[u] =  (unsigned short)data->points[cloud_indx].grayValue;
              noise_mat_ptr[u] = (float)data->points[cloud_indx].noise;
	  	  }
      }
      pcl::toROSMsg(*cloud_, pcl2_msg_);

      pcl2_msg_.header.seq = seq_;
      pcl2_msg_.header.frame_id = frame_id_;
      pcl2_msg_.header.stamp.sec = (int) (data->timeStamp.count() / 1000);
      pcl2_msg_.header.stamp.nsec = (data->timeStamp.count()
          - (int) (data->timeStamp.count() / 1000) * 1000) * 1000000;

      pcl_publisher_.publish(pcl2_msg_);
      
	  // Depth Image
      depth_img_.header.seq = seq_;
      depth_img_.header.frame_id = frame_id_;
	  depth_img_.header.stamp = pcl2_msg_.header.stamp;
	  depth_img_.width = depth_mat.cols;
	  depth_img_.height = depth_mat.rows;
	  depth_img_.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
	  depth_img_.is_bigendian = 0;
	  int step = sizeof(unsigned short) * depth_img_.width;
	  int size = step * depth_img_.height;
	  depth_img_.step = step;
	  depth_img_.data.resize(size);
	  memcpy(&depth_img_.data[0], depth_mat.data, size);
      // Confidence Image
      confidence_img_.header = depth_img_.header;
      confidence_img_.width = conf_mat.cols;
      confidence_img_.height = conf_mat.rows;
      confidence_img_.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
      confidence_img_.is_bigendian = 0;
      step = sizeof(unsigned char) * confidence_img_.width;
      size = step * confidence_img_.height;
      confidence_img_.step = step;
      confidence_img_.data.resize(size);
      memcpy(&confidence_img_.data[0], conf_mat.data,size);
      // Amplitude Image
      amplitude_img_.header = depth_img_.header;
      amplitude_img_.width = ampl_mat.cols;
      amplitude_img_.height = ampl_mat.rows;
      amplitude_img_.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
      amplitude_img_.is_bigendian = 0;
      step = sizeof(unsigned short) * amplitude_img_.width;
      size = step * amplitude_img_.height;
      amplitude_img_.step = step;
      amplitude_img_.data.resize(size);
      memcpy(&amplitude_img_.data[0], ampl_mat.data,size);
      // Noise Imag
      noise_img_.header = depth_img_.header;    
      noise_img_.width = noise_mat.cols;
      noise_img_.height = noise_mat.rows;
      noise_img_.encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      noise_img_.is_bigendian = 0;
      step = sizeof(float) * noise_img_.width;
      size = step * noise_img_.height;
      noise_img_.step = step;
      noise_img_.data.resize(size);
      memcpy(&noise_img_.data[0], noise_mat.data,size);
      // Publish Data
	  depth_publisher_.publish(depth_img_); 
      confidence_publisher_.publish(confidence_img_);
      amplitude_publisher_.publish(amplitude_img_);
      noise_publisher_.publish(noise_img_);
      seq_++;
    }
   private:
    sensor_msgs::PointCloud2 pcl2_msg_;
    sensor_msgs::Image depth_img_;
    sensor_msgs::Image confidence_img_;
    sensor_msgs::Image amplitude_img_;
    sensor_msgs::Image noise_img_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_;
    int seq_;
    std::string topic_name_;
    std::string topic_depth_;
    std::string topic_confidence_;
    std::string topic_amplitude_;
    std::string topic_noise_;
    std::string frame_id_;
    ros::Publisher pcl_publisher_;
    image_transport::Publisher depth_publisher_;
    image_transport::Publisher confidence_publisher_;
    image_transport::Publisher amplitude_publisher_;
    image_transport::Publisher noise_publisher_;
  };
  std::thread thread_;
  ros::ServiceServer config_service_;
  bool configCameraCallback(ros_picoflexx::PicoFlexxConfig::Request  &req,
                            ros_picoflexx::PicoFlexxConfig::Response &res);

};

#endif /* INCLUDE_ROS_PICOFLEXX_ROS_PICOFLEXX_H_ */
