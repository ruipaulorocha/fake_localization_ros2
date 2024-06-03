/*********************************************************************
 * Software License Agreement (BSD 3-Clause License)
 * 
 *  Copyright (c) Rui P. Rocha (2024),
 *                Ioan A. Sucan - Willow Garage Inc. (2008)
 *  All rights reserved.
 * 
 *  Version 2.0.0, May 24, 2024
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

#include <rclcpp/rclcpp.hpp>
//#include <ros/ros.h>
//#include <ros/time.h>

#include <nav_msgs/msg/odometry.hpp>
//#include <nav_msgs/Odometry.h>
#include <geometry_msgs/msg/pose_array.hpp>
//#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
//#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <angles/angles.h>
//#include <angles/angles.h>

//#include "ros/console.h"

#include <tf2/LinearMath/Transform.h>
//#include <tf2/LinearMath/Transform.h>
//#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/create_timer_ros.h>
//#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
//#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
//#include <tf2_ros/transform_listener.h>
#include <tf2_ros/message_filter.h>
//#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
//#include "message_filters/subscriber.h"

class FakeOdomNode : public rclcpp::Node
{
  private:
    //std::shared_ptr<rclcpp::Node> m_nh;
    //ros::NodeHandle m_nh;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr m_posePub;
    //ros::Publisher m_posePub;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr m_particlecloudPub;
    //ros::Publisher m_particlecloudPub;
    std::unique_ptr<tf2_ros::TransformBroadcaster> m_tfServer;
    //tf2_ros::TransformBroadcaster       *m_tfServer;
    std::shared_ptr<tf2_ros::TransformListener> m_tfListener;
    //tf2_ros::TransformListener          *m_tfListener;
    std::shared_ptr<tf2_ros::Buffer> m_tfBuffer;
    //tf2_ros::Buffer                     *m_tfBuffer;
    std::shared_ptr<tf2_ros::MessageFilter<geometry_msgs::msg::PoseWithCovarianceStamped>> m_initPoseFilter;
    //tf2_ros::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>* m_initPoseFilter;
    std::shared_ptr<tf2_ros::MessageFilter<nav_msgs::msg::Odometry>> filter_;
    //tf2_ros::MessageFilter<nav_msgs::Odometry>* filter_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr stuff_sub_;
    //ros::Subscriber stuff_sub_; 
    message_filters::Subscriber<nav_msgs::msg::Odometry> filter_sub_;
    //message_filters::Subscriber<nav_msgs::Odometry>* filter_sub_;
    message_filters::Subscriber<geometry_msgs::msg::PoseWithCovarianceStamped> m_initPoseSub;
    //message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>* m_initPoseSub;

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
    std::string base_pose_ground_truth_topic_; // new parameter in v2.0.0

  public:
    FakeOdomNode(void);
    ~FakeOdomNode(void);
    void stuffFilter(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    //void stuffFilter(const nav_msgs::OdometryConstPtr& odom_msg);
    void update(const nav_msgs::msg::Odometry::SharedPtr message);
    //void update(const nav_msgs::OdometryConstPtr& message);
    void initPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);
    //void initPoseReceived(const geometry_msgs::PoseWithCovarianceStampedConstPtr& msg);
};

FakeOdomNode::FakeOdomNode(void) : Node("fake_localization")
{
      m_posePub = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("amcl_pose", 1);
      //m_posePub = m_nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose",1,true);
      m_particlecloudPub = this->create_publisher<geometry_msgs::msg::PoseArray>("particlecloud", 1);      
      //m_particlecloudPub = m_nh.advertise<geometry_msgs::PoseArray>("particlecloud",1,true);
      m_tfServer = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      //m_tfServer = new tf2_ros::TransformBroadcaster();

      m_tfBuffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
      //m_tfBuffer = new tf2_ros::Buffer();

      // NEW code
      std::chrono::duration<int> buffer_timeout(1);
        // Create the timer interface before call to waitForTransform,
        // to avoid a tf2_ros::CreateTimerInterfaceException exception
      auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        this->get_node_base_interface(), this->get_node_timers_interface() );

      m_tfBuffer->setCreateTimerInterface(timer_interface);
      // end of new code

      m_tfListener = std::make_shared<tf2_ros::TransformListener>(*m_tfBuffer);
      //m_tfListener = new tf2_ros::TransformListener(*m_tfBuffer);

      m_base_pos_received = false;

      // declare parameters
      this->declare_parameter<std::string>("odom_topic", std::string("odom"));
      this->declare_parameter<std::string>("odom_frame_id", std::string("odom"));
      this->declare_parameter<std::string>("base_frame_id", std::string("base_link"));
      this->declare_parameter<std::string>("global_frame_id", std::string("map"));
      this->declare_parameter<double>("delta_x",   0.0);
      this->declare_parameter<double>("delta_y",   0.0);
      this->declare_parameter<double>("delta_yaw", 0.0);
      this->declare_parameter<double>("transform_tolerance", 0.1);


      // load parameters

      this->get_parameter("odom_topic", base_pose_ground_truth_topic_); // this is NEW as of v2.0.0
      RCLCPP_INFO(this->get_logger(),"odom_topic parameter set succesfully to %s",
        base_pose_ground_truth_topic_.c_str());

      this->get_parameter("odom_frame_id", odom_frame_id_);
      RCLCPP_INFO(this->get_logger(),"odom_frame_id parameter set succesfully to %s",
        odom_frame_id_.c_str());
  
      this->get_parameter("base_frame_id", base_frame_id_);
      RCLCPP_INFO(this->get_logger(),"base_frame_id parameter set succesfully to %s",
        base_frame_id_.c_str());

      this->get_parameter("global_frame_id", global_frame_id_);
      RCLCPP_INFO(this->get_logger(),"global_frame_id parameter set succesfully to %s",
        global_frame_id_.c_str());
      
      this->get_parameter("delta_x", delta_x_);
      RCLCPP_INFO(this->get_logger(),"delta_x parameter set succesfully to %f", delta_x_);

      this->get_parameter("delta_y", delta_y_);
      RCLCPP_INFO(this->get_logger(),"delta_y parameter set succesfully to %f", delta_y_);
      
      this->get_parameter("delta_yaw", delta_yaw_);
      RCLCPP_INFO(this->get_logger(),"delta_yaw parameter set succesfully to %f", delta_yaw_);

      this->get_parameter("transform_tolerance", transform_tolerance_);
      RCLCPP_INFO(this->get_logger(),"transform_tolerance parameter set succesfully to %f",
        transform_tolerance_);
      

      // ros::NodeHandle private_nh("~");
      // private_nh.param("odom_frame_id", odom_frame_id_, std::string("odom"));
      // private_nh.param("base_frame_id", base_frame_id_, std::string("base_link")); 
      // private_nh.param("global_frame_id", global_frame_id_, std::string("map"));
      // private_nh.param("delta_x", delta_x_, 0.0);
      // private_nh.param("delta_y", delta_y_, 0.0);
      // private_nh.param("delta_yaw", delta_yaw_, 0.0);      
      // private_nh.param("transform_tolerance", transform_tolerance_, 0.1); 


      m_particleCloud.header.stamp = this->get_clock()->now();
      //m_particleCloud.header.stamp = ros::Time::now();
      m_particleCloud.header.frame_id = global_frame_id_;
      m_particleCloud.poses.resize(1);
      
      //ros::NodeHandle nh;

      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, delta_yaw_);
      m_offsetTf = tf2::Transform(q, tf2::Vector3(delta_x_, delta_y_, 0.0));

      stuff_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        base_pose_ground_truth_topic_, 100, std::bind(&FakeOdomNode::stuffFilter, this, std::placeholders::_1));
      //stuff_sub_ = nh.subscribe("base_pose_ground_truth", 100, &FakeOdomNode::stuffFilter, this);

      filter_sub_.subscribe(this, "");
      //filter_sub_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh, "", 100);
      filter_ = std::make_shared<tf2_ros::MessageFilter<nav_msgs::msg::Odometry>>(
        filter_sub_, *m_tfBuffer, base_frame_id_, 100, this->get_node_logging_interface(),
        this->get_node_clock_interface(), buffer_timeout);
      //filter_ = new tf2_ros::MessageFilter<nav_msgs::Odometry>(*filter_sub_,
      //  *m_tfBuffer, base_frame_id_, 100, nh);      
      filter_->registerCallback(&FakeOdomNode::update, this);
      //filter_->registerCallback([this](auto& msg){ update(msg); });

      // subscription to "2D Pose Estimate" from RViz:
      m_initPoseSub.subscribe(this, "initialpose");
      //m_initPoseSub = new message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped>(nh, "initialpose", 1);
      m_initPoseFilter = std::make_shared<tf2_ros::MessageFilter<geometry_msgs::msg::PoseWithCovarianceStamped>>(
        m_initPoseSub, *m_tfBuffer, global_frame_id_, 1, this->get_node_logging_interface(),
        this->get_node_clock_interface(), buffer_timeout);
      //m_initPoseFilter = new tf2_ros::MessageFilter<geometry_msgs::PoseWithCovarianceStamped>(
      //  *m_initPoseSub, *m_tfBuffer, global_frame_id_, 1, nh);
      m_initPoseFilter->registerCallback(&FakeOdomNode::initPoseReceived, this);
      //m_initPoseFilter->registerCallback([this](auto& msg){ initPoseReceived(msg); });
}

FakeOdomNode::~FakeOdomNode(void)
{
      //if (m_tfServer)
      //  delete m_tfServer; 
      //if (m_tfListener)
      //  delete m_tfListener;
      //if (m_tfBuffer)
      //  delete m_tfBuffer;
}

void FakeOdomNode::stuffFilter(const nav_msgs::msg::Odometry::SharedPtr odom_msg){
      //we have to do this to force the message filter to wait for transforms
      //from odom_frame_id_ to base_frame_id_ to be available at time odom_msg.header.stamp
      //really, the base_pose_ground_truth should come in with no frame_id b/c it doesn't make sense

      filter_->add(odom_msg);
      // boost::shared_ptr<nav_msgs::Odometry> stuff_msg(new nav_msgs::Odometry);
      // *stuff_msg = *odom_msg;
      // stuff_msg->header.frame_id = odom_frame_id_;
      // filter_->add(stuff_msg);
}

void FakeOdomNode::update(const nav_msgs::msg::Odometry::SharedPtr message){
      tf2::Transform txi;
      tf2::convert(message->pose.pose, txi);
      txi = m_offsetTf * txi;

      geometry_msgs::msg::TransformStamped odom_to_map;
      //geometry_msgs::TransformStamped odom_to_map;
      try
      {
        geometry_msgs::msg::TransformStamped txi_inv;
        //geometry_msgs::TransformStamped txi_inv;
        txi_inv.header.frame_id = base_frame_id_;
        txi_inv.header.stamp = message->header.stamp;
        tf2::convert(txi.inverse(), txi_inv.transform);

        m_tfBuffer->transform(txi_inv, odom_to_map, odom_frame_id_);
      }
      catch (tf2::TransformException &e)
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to transform to %s from %s: %s\n",
          odom_frame_id_.c_str(), base_frame_id_.c_str(), e.what());
        //ROS_ERROR("Failed to transform to %s from %s: %s\n", odom_frame_id_.c_str(), base_frame_id_.c_str(), e.what());
        return;
      }

      geometry_msgs::msg::TransformStamped trans;
      //geometry_msgs::TransformStamped trans;
      int32_t toler_secs = (int32_t) transform_tolerance_;
      uint32_t toler_nanosecs = (uint32_t) ( (transform_tolerance_ - (double) toler_secs) * 1e9);
      trans.header.stamp.sec = message->header.stamp.sec + toler_secs;
      trans.header.stamp.nanosec = message->header.stamp.nanosec + toler_nanosecs;
      //trans.header.stamp = message->header.stamp + ros::Duration(transform_tolerance_);
      trans.header.frame_id = global_frame_id_;
      trans.child_frame_id = message->header.frame_id;
      tf2::Transform odom_to_map_tf2;
      tf2::convert(odom_to_map.transform, odom_to_map_tf2);
      tf2::Transform odom_to_map_inv = odom_to_map_tf2.inverse();
      tf2::convert(odom_to_map_inv, trans.transform);
      m_tfServer->sendTransform(trans);

      tf2::Transform current;
      tf2::convert(message->pose.pose, current);

      //also apply the offset to the pose
      current = m_offsetTf * current;

      geometry_msgs::msg::Transform current_msg;
      //geometry_msgs::Transform current_msg;
      tf2::convert(current, current_msg);

      // Publish localized pose
      m_currentPos.header = message->header;
      m_currentPos.header.frame_id = global_frame_id_;
      tf2::convert(current_msg.rotation, m_currentPos.pose.pose.orientation);
      m_currentPos.pose.pose.position.x = current_msg.translation.x;
      m_currentPos.pose.pose.position.y = current_msg.translation.y;
      m_currentPos.pose.pose.position.z = current_msg.translation.z;
      m_posePub->publish(m_currentPos);
      //m_posePub.publish(m_currentPos);

      // The particle cloud is the current position. Quite convenient.
      m_particleCloud.header = m_currentPos.header;
      m_particleCloud.poses[0] = m_currentPos.pose.pose;
      m_particlecloudPub->publish(m_particleCloud);
      //m_particlecloudPub.publish(m_particleCloud);
}

void FakeOdomNode::initPoseReceived(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg){
      tf2::Transform pose;
      tf2::convert(msg->pose.pose, pose);

      if (msg->header.frame_id != global_frame_id_){
        RCLCPP_WARN(this->get_logger(), "Frame ID of \"initialpose\" (%s) is different from the global frame %s",
          msg->header.frame_id.c_str(), global_frame_id_.c_str());
        //ROS_WARN("Frame ID of \"initialpose\" (%s) is different from the global frame %s", msg->header.frame_id.c_str(), global_frame_id_.c_str());
      }

      // set offset so that current pose is set to "initialpose"    
      geometry_msgs::msg::TransformStamped baseInMap;
      //geometry_msgs::TransformStamped baseInMap;
      try{
        // just get the latest
        baseInMap = m_tfBuffer->lookupTransform(base_frame_id_, global_frame_id_, rclcpp::Time(0));
        //baseInMap = m_tfBuffer->lookupTransform(base_frame_id_, global_frame_id_, ros::Time(0));
      } catch (tf2::TransformException &e) {
        RCLCPP_WARN(this->get_logger(), "Failed to lookup transform!");
        //ROS_WARN("Failed to lookup transform!");
        return;
      }

      tf2::Transform baseInMapTf2;
      tf2::convert(baseInMap.transform, baseInMapTf2);
      tf2::Transform delta = pose * baseInMapTf2;
      m_offsetTf = delta * m_offsetTf;
}

int main(int argc, char** argv)
{
  //ros::init(argc, argv, "fake_localization");
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = std::make_shared<FakeOdomNode>();
  //FakeOdomNode odom;

  rclcpp::spin(node->get_node_base_interface());
  //ros::spin();

  // Shutdown the ROS2 communication
  rclcpp::shutdown();

  return 0;
}
