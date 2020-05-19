/*
 * Copyright 2012 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

/*
 * Desc: 3D position interface for ground truth.
 * Author: Sachin Chitta and John Hsu
 * Date: 1 June 2008
 */

/*
 * Gazebo-P3D-plugin modified to simulate basic RTK-GPS-functionality for testing purposes
 */

#ifndef GAZEBO_ROS_RTK_GPS_HH
#define GAZEBO_ROS_RTK_GPS_HH

#include <string>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <gazebo_plugins/PubQueue.h>
#include <queue>

namespace gazebo
{

  class GazeboRosRTK : public ModelPlugin
  {
    /// \brief Constructor
    public: GazeboRosRTK();

    /// \brief Destructor
    public: virtual ~GazeboRosRTK();

    /// \brief Load the controller
    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf);

    /// \brief Update the controller
    protected: virtual void UpdateChild();

    private: physics::WorldPtr world_;
    private: physics::ModelPtr model_;

    /// \brief The parent Model
    private: physics::LinkPtr link_;

    /// \brief The body of the frame to display pose, twist
    private: physics::LinkPtr reference_link_;


    /// \brief pointer to ros node
    private: ros::NodeHandle* rosnode_;
    private: ros::Publisher pub_;
    private: ros::Publisher pub_ground_truth_;
    private: PubQueue<nav_msgs::Odometry>::Ptr pub_Queue;

    /// \brief ros message
    private: std::queue<std::pair<bool, nav_msgs::Odometry>> pose_msgs_;
    private: nav_msgs::Odometry ground_truth_pose_msg_;

    /// \brief store bodyname
    private: std::string link_name_;

    /// \brief topic name
    private: std::string topic_name_;
    private: std::string ground_truth_topic_name_;

    /// \brief frame transform name, should match link name
    /// FIXME: extract link name directly?
    private: std::string frame_name_;
    private: std::string tf_frame_name_;

    /// \brief allow specifying constant xyz and rpy offsets
    private: ignition::math::Pose3d offset_;
    private: ignition::math::Vector3d antenna_offset_;

    /// \brief mutex to lock access to fields used in message callbacks
    private: boost::mutex lock;

    /// \brief save last_time
    private: common::Time last_time_;
    private: ignition::math::Vector3d last_vpos_;
    private: ignition::math::Vector3d last_veul_;
    private: ignition::math::Vector3d apos_;
    private: ignition::math::Vector3d aeul_;
    private: ignition::math::Vector3d last_frame_vpos_;
    private: ignition::math::Vector3d last_frame_veul_;
    private: ignition::math::Vector3d frame_apos_;
    private: ignition::math::Vector3d frame_aeul_;

    // rate control
    private: double update_rate_;

    /// \brief Gaussian noise
    private: double gaussian_noise_pos_x_;
    private: double gaussian_noise_pos_y_;
    private: double gaussian_noise_pos_z_;
    private: double gaussian_noise_orientation_;
    private: double gaussian_noise_twist_x_;
    private: double gaussian_noise_twist_y_;
    private: double gaussian_noise_twist_z_;

    struct rectangle
    {
      double xa = 0.0;
      double ya = 0.0;
      double xb = 0.0;
      double yb = 0.0;
      double xc = 0.0;
      double yc = 0.0;
      double xd = 0.0;
      double yd = 0.0;
      double area_rect = 0.0;
      double add_noise = 0.0;
      bool gps_available = true;
    };

    private: std::vector<struct rectangle> issue_areas;

    /// \brief Gaussian noise generator
    private: double GaussianKernel(double mu, double sigma);

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;

    private: ros::CallbackQueue rtk_queue_;
    private: void RTKQueueThread();
    private: boost::thread callback_queue_thread_;

    // Pointer to the update event connection
    private: event::ConnectionPtr update_connection_;

    private: unsigned int seed;

    // ros publish multi queue, prevents publish() blocking
    private: PubMultiQueue pmq;


    /// \brief topic name
    private: std::string issue_config_;
    private: void getGPSIssueAreas(std::string file);
    void calculateAreaIssue(double pos_x, double pos_y, bool *gps_available, double *add_noise);
    size_t maximum_queue_size_;

  };
}
#endif
