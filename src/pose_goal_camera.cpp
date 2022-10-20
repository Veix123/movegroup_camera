/*
Copyright 2019, University of Tartu

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following
disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/* Author: Veiko Vunder */
/* E-mail: veiko.vunder@ut.ee */

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>
#include <image_transport/image_transport.h>

// Global flags triggering planning and execution
bool trigger_plan = false;
bool trigger_exec = false;

void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
  
  cv_bridge::CvImagePtr cv_ptr;
  cv::Mat3b img_hsv;

  try
  {
    
    // Convert from ROS message to an opencv image
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    // Convert from BGR8 colorspace to HSV colorspace for more convenient analysis
    cv::cvtColor(cv_ptr->image, img_hsv, cv::COLOR_BGR2HSV);

    // Get the Saturation channels from the HSV colorspace for both sides of an image
    unsigned int left_sat = (cv_ptr->image).at<cv::Vec3b>(img_hsv.rows / 2, img_hsv.cols/4)[1];
    unsigned int right_sat = (cv_ptr->image).at<cv::Vec3b>(img_hsv.rows / 2, img_hsv.cols/4*3)[1];
    ROS_INFO("Left sat: %u \tRight sat: %u", left_sat, right_sat);

    // Conditions at which planning and execution will trigger
    if(left_sat > 230)
    {
      trigger_plan = true;
    }

    if(right_sat > 230)
    {
      trigger_exec = true;
    }

    // Display received image in a separate window.
    cv::imshow("debug image", cv_ptr->image);
    cv::waitKey(50);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception %s", e.what());
  }
}

int main(int argc, char** argv)
{
  // Basic ROS setup
  // ^^^^^^^^^^^^^^^
  ros::init(argc, argv, "pose_goal");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Create a subscriber using image_transport
  image_transport::ImageTransport it(node_handle);
  image_transport::Subscriber sub = it.subscribe("/usb_cam/image_raw", 1, image_callback);

  // Setting the planning group you would like to control and plan for.
  moveit::planning_interface::MoveGroupInterface move_group("xarm7");

  // Preparing a variable for storing motion plan.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  while(ros::ok())
  {
    // Check if planning was triggered in the callback
    if(trigger_plan)
    {
      // Getting the current pose of the end-effector.
      geometry_msgs::PoseStamped current_pose;
      current_pose = move_group.getCurrentPose();
  
      // Modifying the current pose into a target pose.
      geometry_msgs::Pose target_pose = current_pose.pose;
      target_pose.position.x += 0.1;
      target_pose.position.y += -0.2;
      target_pose.position.z += -0.0;

      // Setting the target pose for the end-effector
      move_group.setPoseTarget(target_pose);

      // Compute the plan for the specified target pose
      moveit::core::MoveItErrorCode success = move_group.plan(my_plan);
      if (success)
      {
        ROS_INFO("[movegroup_interface_demo/pose_goal] Planning OK. Proceeding.");
      }
      else
      {
        ROS_WARN("[movegroup_interface_demo/pose_goal] Planning failed.");
      }

      // Give some time before replanning
      ros::Duration(2).sleep();
      trigger_plan = false;
    }

    // Check if execution was triggered in the callback
    if(trigger_exec)
    {
      // Executing the computed plan
      move_group.execute(my_plan);
      ros::Duration(2).sleep();
      trigger_exec = false;
    }
  }

  ros::shutdown();
  return 0;
}
