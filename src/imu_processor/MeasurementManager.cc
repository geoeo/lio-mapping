/**
* This file is part of LIO-mapping.
* 
* Copyright (C) 2019 Haoyang Ye <hy.ye at connect dot ust dot hk>,
* Robotics and Multiperception Lab (RAM-LAB <https://ram-lab.com>),
* The Hong Kong University of Science and Technology
* 
* For more information please see <https://ram-lab.com/file/hyye/lio-mapping>
* or <https://sites.google.com/view/lio-mapping>.
* If you use this code, please cite the respective publications as
* listed on the above websites.
* 
* LIO-mapping is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* LIO-mapping is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
* 
* You should have received a copy of the GNU General Public License
* along with LIO-mapping.  If not, see <http://www.gnu.org/licenses/>.
*/

//
// Created by hyye on 3/27/18.
//

#include "imu_processor/MeasurementManager.h"

namespace lio {

void MeasurementManager::SetupRos(ros::NodeHandle &nh) {
  is_ros_setup_ = true;

  nh_ = nh;

  sub_imu_ = nh_.subscribe<sensor_msgs::Imu>(mm_config_.imu_topic,
                                             1000,
                                             &MeasurementManager::ImuHandler,
                                             this,
                                             ros::TransportHints().tcpNoDelay());

  sub_compact_data_ = nh_.subscribe<sensor_msgs::PointCloud2>(mm_config_.compact_data_topic,
                                                              10,
                                                              &MeasurementManager::CompactDataHandler,
                                                              this);

    sub_imu_accel_ = nh_.subscribe<geometry_msgs::AccelStamped>(mm_config_.imu_topic_accel,1000,&MeasurementManager::ImuAccelHandler, this);
    sub_imu_odom_ = nh_.subscribe<nav_msgs::Odometry>(mm_config_.imu_topic_odom,1000,&MeasurementManager::ImuOdomHandler, this);

//  sub_laser_odom_ =
//      nh_.subscribe<nav_msgs::Odometry>(mm_config_.laser_odom_topic, 10, &MeasurementManager::LaserOdomHandler, this);
}

PairMeasurements MeasurementManager::GetMeasurements() {

  PairMeasurements measurements;

  while (true) {

    if (mm_config_.enable_imu) {


      if (imu_buf_.empty() || compact_data_buf_.empty()) {
        return measurements;
      }

      if (imu_buf_.back()->header.stamp.toSec()
          <= compact_data_buf_.front()->header.stamp.toSec() + mm_config_.msg_time_delay) {
        //ROS_DEBUG("wait for imu, only should happen at the beginning");
        // Count for waiting time
        return measurements;
      }

      if (imu_buf_.front()->header.stamp.toSec()
          >= compact_data_buf_.front()->header.stamp.toSec() + mm_config_.msg_time_delay) {
        ROS_DEBUG("throw compact_data, only should happen at the beginning");
        compact_data_buf_.pop();
        continue;
      }
      CompactDataConstPtr compact_data_msg = compact_data_buf_.front();
      compact_data_buf_.pop();

      vector<sensor_msgs::ImuConstPtr> imu_measurements;
      while (imu_buf_.front()->header.stamp.toSec()
          < compact_data_msg->header.stamp.toSec() + mm_config_.msg_time_delay) {
        imu_measurements.emplace_back(imu_buf_.front());
          imu_buf_.pop();
      }

        // NOTE: one message after laser odom msg
        imu_measurements.emplace_back(imu_buf_.front());


      if (imu_measurements.empty()) {
        ROS_DEBUG("no imu between two image");
      }
      measurements.emplace_back(imu_measurements, compact_data_msg);
    } else {
      vector<sensor_msgs::ImuConstPtr> imu_measurements;
      if (compact_data_buf_.empty()) {
        return measurements;
      }
      CompactDataConstPtr compact_data_msg = compact_data_buf_.front();
      compact_data_buf_.pop();
      measurements.emplace_back(imu_measurements, compact_data_msg);
    }

  }

}

void MeasurementManager::ImuHandler(const sensor_msgs::ImuConstPtr &raw_imu_msg) {
  if (raw_imu_msg->header.stamp.toSec() <= imu_last_time_) {
    LOG(ERROR) << ("imu message in disorder!");
    return;
  }

  imu_last_time_ = raw_imu_msg->header.stamp.toSec();

  buf_mutex_.lock();
  imu_buf_.push(raw_imu_msg);
  buf_mutex_.unlock();

  con_.notify_one();

  {
    lock_guard<mutex> lg(state_mutex_);
    // TODO: is it necessary to do predict here?
//    Predict(raw_imu_msg);

//    std_msgs::Header header = imu_msg->header;
//    header.frame_id = "world";
//    if (flag == INITIALIZED)
//      PubLatestOdometry(states, header);
  }
} // ImuHandler

void MeasurementManager::ImuAccelHandler(const geometry_msgs::AccelStampedConstPtr &accel_msg){
    imu_last_time_ = accel_msg->header.stamp.toSec();

    auto imu_msg = new sensor_msgs::Imu();
    odom_mutex_.lock();
    auto odom_msg = odom_msg_recent_;
    odom_mutex_.unlock();

    imu_msg->header = accel_msg->header;

    geometry_msgs::Vector3 linear_accel = accel_msg->accel.linear;
    linear_accel.x *= 9.81;
    linear_accel.y *= 9.81;
    linear_accel.z *= 9.81;
    auto twist = odom_msg->twist;
    double linear_accel_covariance[9] = {twist.covariance[0], twist.covariance[1], twist.covariance[2],
                                         twist.covariance[6],twist.covariance[7], twist.covariance[8],
                                         twist.covariance[12], twist.covariance[13],twist.covariance[14]};

    geometry_msgs::Vector3 angular_accel = accel_msg->accel.angular;
    //convert to radians
    angular_accel.x = angular_accel.x* M_PIf32 / 180;
    angular_accel.y = angular_accel.y* M_PIf32 / 180;
    angular_accel.z = angular_accel.z* M_PIf32 / 180;
    double angular_accel_covariance[9] = {twist.covariance[21], twist.covariance[22], twist.covariance[23],
                                          twist.covariance[27],twist.covariance[28], twist.covariance[29],
                                          twist.covariance[33], twist.covariance[34],twist.covariance[35]};



    geometry_msgs::Quaternion orientation = odom_msg->pose.pose.orientation;
    auto pose = odom_msg->pose;
    double orientation_covariance[9] = {pose.covariance[21], pose.covariance[22], pose.covariance[23],
                                        pose.covariance[27],pose.covariance[28], pose.covariance[29],
                                        pose.covariance[33], pose.covariance[34],pose.covariance[35]};

    imu_msg->orientation = orientation;

    imu_msg->linear_acceleration = linear_accel;
    imu_msg->angular_velocity = angular_accel;

    for(int i = 0; i < 9; i++){
        imu_msg->orientation_covariance[i] = orientation_covariance[i];
        imu_msg->linear_acceleration_covariance[i] = linear_accel_covariance[i];
        imu_msg->angular_velocity_covariance[i] = angular_accel_covariance[i];
    }

    auto img_msg_ptr = boost::shared_ptr<sensor_msgs::Imu const>(imu_msg);
    buf_mutex_.lock();
    imu_buf_.push(img_msg_ptr);
    buf_mutex_.unlock();

    con_.notify_one();

}

// When I have one odom measurement I should have multiple imu measurements
void MeasurementManager::ImuOdomHandler(const nav_msgs::OdometryConstPtr &odom_data_msg){

//    buf_mutex_.lock();
//    imu_odom_buf_.push(odom_data_msg);
//    buf_mutex_.unlock();
//
//    con_.notify_one();
      odom_mutex_.lock();
      odom_msg_recent_ = odom_data_msg;
      odom_mutex_.unlock();

}

void MeasurementManager::LaserOdomHandler(const nav_msgs::OdometryConstPtr &laser_odom_msg) {
  buf_mutex_.lock();
  laser_odom_buf_.push(laser_odom_msg);
  buf_mutex_.unlock();

  con_.notify_one();
}

void MeasurementManager::CompactDataHandler(const sensor_msgs::PointCloud2ConstPtr &compact_data_msg) {
  buf_mutex_.lock();
  compact_data_buf_.push(compact_data_msg);
  buf_mutex_.unlock();

  con_.notify_one();
}

}