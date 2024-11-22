/*********************************************************************
 * Software License Agreement (BSD 3-Clause License)
 * 
 *  Copyright (c) Rui P. Rocha (2024),
 *                Ioan A. Sucan - Willow Garage Inc. (2008)
 *  All rights reserved.
 * 
 *  Version 2.1.0, Nov. 20, 2024
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

/** \author Rui P. Rocha */

/**

  @mainpage

  @htmlinclude manifest.html

  @b odom_localization Takes in ground truth pose information for a robot
  base (e.g., from a simulator or motion capture system) and republishes
  it as if a localization system were in use.

  <hr>

  @section usage Usage
  @verbatim
  $ fake_localization
  @endverbatim

  <hr>

  @section topic ROS topics

  Subscribes to (name/type):
  - @b "base_pose_ground_truth" nav_msgs/Odometry : robot's odometric pose.  Only the position information is used (velocity is ignored).

  Publishes to (name / type):
  - @b "amcl_pose" geometry_msgs/PoseWithCovarianceStamped : robot's estimated pose in the map, with covariance
  - @b "particlecloud" geometry_msgs/PoseArray : fake set of poses being maintained by the filter (one paricle only).

  <hr>

  @section parameters ROS parameters

  - "~odom_frame_id" (string) : The odometry frame to be used, default: "odom"

 **/

#ifndef CLASS_CCorrOdomOffset
#define CLASS_CCorrOdomOffset


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>

namespace fake_localization_ros2{

  class FakeOdomNode : public rclcpp::Node
  {
    private:
      rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_posePub;
      rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr m_particlecloudPub;
      std::unique_ptr<tf2_ros::TransformBroadcaster> m_tfServer;
      std::shared_ptr<tf2_ros::TransformListener> m_tfListener;
      std::shared_ptr<tf2_ros::Buffer> m_tfBuffer;
      std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PoseWithCovarianceStamped>> m_initPoseFilter;
      std::shared_ptr<tf2_ros::MessageFilter<nav_msgs::msg::Odometry>> filter_;
      rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr stuff_sub_; 
      message_filters::Subscriber<nav_msgs::msg::Odometry> filter_sub_;
      message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped> m_initPoseSub;

      double delta_x_, delta_y_, delta_yaw_;
      bool   m_base_pos_received;
      double transform_tolerance_;

      geometry_msgs::msg::PoseArray                  m_particleCloud;
      geometry_msgs::msg::PoseWithCovarianceStamped  m_currentPos;
      tf2::Transform m_offsetTf;

      //parameter for what odom to use
      std::string odom_frame_id_;
      std::string base_frame_id_;
      std::string global_frame_id_;
      std::string odom_topic_; // new parameter in v2.0.0

    public:
      FakeOdomNode(const rclcpp::NodeOptions & options);
      void stuffFilter(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
      void update(const nav_msgs::msg::Odometry::SharedPtr message);
      void initPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
  };

}

#endif