/*
 * Copyright 2013 Open Source Robotics Foundation
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

#include <string>
#include <tf/tf.h>
#include <stdlib.h>

#include "gazebo_plugins/gazebo_ros_rtk_gps.h"

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(GazeboRosRTK);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosRTK::GazeboRosRTK()
{
  this->seed = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosRTK::~GazeboRosRTK()
{
  this->update_connection_.reset();
  // Finalize the controller
  this->rosnode_->shutdown();
  this->rtk_queue_.clear();
  this->rtk_queue_.disable();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosRTK::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
{
  // Get the world name.
  this->world_ = _parent->GetWorld();
  this->model_ = _parent;

  // load parameters
  this->robot_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->robot_namespace_ =
      _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  if (!_sdf->HasElement("bodyName"))
  {
    ROS_FATAL_NAMED("rtk", "gazebo_ros_rtk_gps plugin missing <bodyName>, cannot proceed");
    return;
  }
  else
    this->link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();

  this->link_ = _parent->GetLink(this->link_name_);
  if (!this->link_)
  {
    ROS_FATAL_NAMED("rtk", "gazebo_ros_rtk_gps plugin error: bodyName: %s does not exist\n",
      this->link_name_.c_str());
    return;
  }

  if (!_sdf->HasElement("topicName"))
  {
    ROS_FATAL_NAMED("rtk", "gazebo_ros_rtk_gps plugin missing <topicName>, cannot proceed");
    return;
  }
  else
    this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();

  if (!_sdf->HasElement("frameName"))
  {
    ROS_DEBUG_NAMED("rtk", "gazebo_ros_rtk_gps plugin missing <frameName>, defaults to world");
    this->frame_name_ = "world";
  }
  else
    this->frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();

  if (!_sdf->HasElement("xyzOffset"))
  {
    ROS_DEBUG_NAMED("rtk", "gazebo_ros_rtk_gps plugin missing <xyzOffset>, defaults to 0s");
    this->offset_.Pos() = ignition::math::Vector3d(0, 0, 0);
  }
  else
    this->offset_.Pos() = _sdf->GetElement("xyzOffset")->Get<ignition::math::Vector3d>();

  if (!_sdf->HasElement("rpyOffset"))
  {
    ROS_DEBUG_NAMED("rtk", "gazebo_ros_rtk_gps plugin missing <rpyOffset>, defaults to 0s");
    this->offset_.Rot() = ignition::math::Quaterniond(ignition::math::Vector3d(0, 0, 0));
  }
  else
    this->offset_.Rot() = ignition::math::Quaterniond(_sdf->GetElement("rpyOffset")->Get<ignition::math::Vector3d>());

  if (!_sdf->HasElement("gaussianNoisePosX"))
  {
    ROS_DEBUG_NAMED("rtk", "gazebo_ros_rtk_gps plugin missing <gaussianNoisePosX>, defaults to 0.0");
    this->gaussian_noise_pos_x_ = 0;
  }
  else
    this->gaussian_noise_pos_x_ = _sdf->GetElement("gaussianNoisePosX")->Get<double>();

  if (!_sdf->HasElement("gaussianNoisePosY"))
  {
    ROS_DEBUG_NAMED("rtk", "gazebo_ros_rtk_gps plugin missing <gaussianNoisePosY>, defaults to 0.0");
    this->gaussian_noise_pos_y_ = 0;
  }
  else
    this->gaussian_noise_pos_y_ = _sdf->GetElement("gaussianNoisePosY")->Get<double>();

  if (!_sdf->HasElement("gaussianNoisePosZ"))
  {
    ROS_DEBUG_NAMED("rtk", "gazebo_ros_rtk_gps plugin missing <gaussianNoisePosZ>, defaults to 0.0");
    this->gaussian_noise_pos_z_ = 0;
  }
  else
    this->gaussian_noise_pos_z_ = _sdf->GetElement("gaussianNoisePosZ")->Get<double>();

  if (!_sdf->HasElement("gaussianNoiseOrientation"))
  {
    ROS_DEBUG_NAMED("rtk", "gazebo_ros_rtk_gps plugin missing <gaussianNoiseOrientation>, defaults to 0.0");
    this->gaussian_noise_orientation_ = 0;
  }
  else
    this->gaussian_noise_orientation_ = _sdf->GetElement("gaussianNoiseOrientation")->Get<double>();

  if (!_sdf->HasElement("gaussianNoiseTwistX"))
  {
    ROS_DEBUG_NAMED("rtk", "gazebo_ros_rtk_gps plugin missing <gaussianNoiseTwistX>, defaults to 0.0");
    this->gaussian_noise_twist_x_ = 0;
  }
  else
    this->gaussian_noise_twist_x_ = _sdf->GetElement("gaussianNoiseTwistX")->Get<double>();

  if (!_sdf->HasElement("gaussianNoiseTwistY"))
  {
    ROS_DEBUG_NAMED("rtk", "gazebo_ros_rtk_gps plugin missing <gaussianNoiseTwistY>, defaults to 0.0");
    this->gaussian_noise_twist_y_ = 0;
  }
  else
    this->gaussian_noise_twist_y_ = _sdf->GetElement("gaussianNoiseTwistY")->Get<double>();

  if (!_sdf->HasElement("gaussianNoiseTwistZ"))
  {
    ROS_DEBUG_NAMED("rtk", "gazebo_ros_rtk_gps plugin missing <gaussianNoiseTwistZ>, defaults to 0.0");
    this->gaussian_noise_twist_z_ = 0;
  }
  else
    this->gaussian_noise_twist_z_ = _sdf->GetElement("gaussianNoiseTwistZ")->Get<double>();

  if (!_sdf->HasElement("updateRate"))
  {
    ROS_DEBUG_NAMED("rtk", "gazebo_ros_rtk_gps plugin missing <updateRate>, defaults to 0.0"
             " (as fast as possible)");
    this->update_rate_ = 0;
  }
  else
    this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("gazebo_ros_rtk_gps", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
      << "Load the Gazebo system plugin 'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
    return;
  }

  this->rosnode_ = new ros::NodeHandle(this->robot_namespace_);

  // publish multi queue
  this->pmq.startServiceThread();

  // resolve tf prefix
  std::string prefix;
  this->rosnode_->getParam(std::string("tf_prefix"), prefix);
  this->tf_frame_name_ = tf::resolve(prefix, this->frame_name_);

  if (this->topic_name_ != "")
  {
    this->pub_Queue = this->pmq.addPub<nav_msgs::Odometry>();
    this->pub_ =
      this->rosnode_->advertise<nav_msgs::Odometry>(this->topic_name_, 1);
  }

#if GAZEBO_MAJOR_VERSION >= 8
  this->last_time_ = this->world_->SimTime();
#else
  this->last_time_ = this->world_->GetSimTime();
#endif
  // initialize body
#if GAZEBO_MAJOR_VERSION >= 8
  this->last_vpos_ = this->link_->WorldLinearVel();
  this->last_veul_ = this->link_->WorldAngularVel();
#else
  this->last_vpos_ = this->link_->GetWorldLinearVel().Ign();
  this->last_veul_ = this->link_->GetWorldAngularVel().Ign();
#endif
  this->apos_ = 0;
  this->aeul_ = 0;

  // if frameName specified is "/world", "world", "/map" or "map" report
  // back inertial values in the gazebo world
  if (this->frame_name_ != "/world" &&
      this->frame_name_ != "world" &&
      this->frame_name_ != "/map" &&
      this->frame_name_ != "map")
  {
    this->reference_link_ = this->model_->GetLink(this->frame_name_);
    if (!this->reference_link_)
    {
      ROS_ERROR_NAMED("rtk", "gazebo_ros_rtk_gps plugin: frameName: %s does not exist, will"
                " not publish pose\n", this->frame_name_.c_str());
      return;
    }
  }

  // init reference frame state
  if (this->reference_link_)
  {
    ROS_DEBUG_NAMED("rtk", "got body %s", this->reference_link_->GetName().c_str());
    this->frame_apos_ = 0;
    this->frame_aeul_ = 0;
#if GAZEBO_MAJOR_VERSION >= 8
    this->last_frame_vpos_ = this->reference_link_->WorldLinearVel();
    this->last_frame_veul_ = this->reference_link_->WorldAngularVel();
#else
    this->last_frame_vpos_ = this->reference_link_->GetWorldLinearVel().Ign();
    this->last_frame_veul_ = this->reference_link_->GetWorldAngularVel().Ign();
#endif
  }


  // start custom queue for rtk
  this->callback_queue_thread_ = boost::thread(
    boost::bind(&GazeboRosRTK::RTKQueueThread, this));

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosRTK::UpdateChild, this));
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosRTK::UpdateChild()
{
  if (!this->link_)
    return;

#if GAZEBO_MAJOR_VERSION >= 8
  common::Time cur_time = this->world_->SimTime();
#else
  common::Time cur_time = this->world_->GetSimTime();
#endif

  if (cur_time < this->last_time_)
  {
      ROS_WARN_NAMED("rtk", "Negative update time difference detected.");
      this->last_time_ = cur_time;
  }

  // rate control
  if (this->update_rate_ > 0 &&
      (cur_time-this->last_time_).Double() < (1.0/this->update_rate_))
    return;

  if (this->pub_.getNumSubscribers() > 0)
  {
    // differentiate to get accelerations
    double tmp_dt = cur_time.Double() - this->last_time_.Double();
    if (tmp_dt != 0)
    {
      this->lock.lock();

      if (this->topic_name_ != "")
      {
        // copy data into pose message
        this->pose_msg_.header.frame_id = this->tf_frame_name_;
        this->pose_msg_.header.stamp.sec = cur_time.sec;
        this->pose_msg_.header.stamp.nsec = cur_time.nsec;

        this->pose_msg_.child_frame_id = this->link_name_;

        ignition::math::Pose3d pose, frame_pose;
        ignition::math::Vector3d frame_vpos;
        ignition::math::Vector3d frame_veul;

        // get inertial Rates
        // Get Pose/Orientation
#if GAZEBO_MAJOR_VERSION >= 8
        ignition::math::Vector3d vpos = this->link_->WorldLinearVel();
        ignition::math::Vector3d veul = this->link_->WorldAngularVel();

        pose = this->link_->WorldPose();
#else
        ignition::math::Vector3d vpos = this->link_->GetWorldLinearVel().Ign();
        ignition::math::Vector3d veul = this->link_->GetWorldAngularVel().Ign();

        pose = this->link_->GetWorldPose().Ign();
#endif

        // Apply Reference Frame
        if (this->reference_link_)
        {
          // convert to relative pose, rates
#if GAZEBO_MAJOR_VERSION >= 8
          frame_pose = this->reference_link_->WorldPose();
          frame_vpos = this->reference_link_->WorldLinearVel();
          frame_veul = this->reference_link_->WorldAngularVel();
#else
          frame_pose = this->reference_link_->GetWorldPose().Ign();
          frame_vpos = this->reference_link_->GetWorldLinearVel().Ign();
          frame_veul = this->reference_link_->GetWorldAngularVel().Ign();
#endif
          pose.Pos() = pose.Pos() - frame_pose.Pos();
          pose.Pos() = frame_pose.Rot().RotateVectorReverse(pose.Pos());
          pose.Rot() *= frame_pose.Rot().Inverse();

          vpos = frame_pose.Rot().RotateVector(vpos - frame_vpos);
          veul = frame_pose.Rot().RotateVector(veul - frame_veul);
        }

        // Apply Constant Offsets
        // apply xyz offsets and get position and rotation components
        pose.Pos() = pose.Pos() + this->offset_.Pos();
        // apply rpy offsets
        pose.Rot() = this->offset_.Rot()*pose.Rot();
        pose.Rot().Normalize();

        // compute accelerations (not used)
        this->apos_ = (this->last_vpos_ - vpos) / tmp_dt;
        this->aeul_ = (this->last_veul_ - veul) / tmp_dt;
        this->last_vpos_ = vpos;
        this->last_veul_ = veul;

        this->frame_apos_ = (this->last_frame_vpos_ - frame_vpos) / tmp_dt;
        this->frame_aeul_ = (this->last_frame_veul_ - frame_veul) / tmp_dt;
        this->last_frame_vpos_ = frame_vpos;
        this->last_frame_veul_ = frame_veul;

        // Fill out messages
        this->pose_msg_.pose.pose.position.x    = pose.Pos().X() +
          this->GaussianKernel(0, this->gaussian_noise_pos_x_);
        this->pose_msg_.pose.pose.position.y    = pose.Pos().Y() +
          this->GaussianKernel(0, this->gaussian_noise_pos_y_);
        this->pose_msg_.pose.pose.position.z    = pose.Pos().Z() +
          this->GaussianKernel(0, this->gaussian_noise_pos_z_);

        this->pose_msg_.pose.pose.orientation.x = pose.Rot().X() +
          this->GaussianKernel(0, this->gaussian_noise_orientation_);
        this->pose_msg_.pose.pose.orientation.y = pose.Rot().Y() +
          this->GaussianKernel(0, this->gaussian_noise_orientation_);
        this->pose_msg_.pose.pose.orientation.z = pose.Rot().Z() +
          this->GaussianKernel(0, this->gaussian_noise_orientation_);
        this->pose_msg_.pose.pose.orientation.w = pose.Rot().W() +
          this->GaussianKernel(0, this->gaussian_noise_orientation_);

        this->pose_msg_.twist.twist.linear.x  = vpos.X() +
          this->GaussianKernel(0, this->gaussian_noise_twist_x_);
        this->pose_msg_.twist.twist.linear.y  = vpos.Y() +
          this->GaussianKernel(0, this->gaussian_noise_twist_y_);
        this->pose_msg_.twist.twist.linear.z  = vpos.Z() +
          this->GaussianKernel(0, this->gaussian_noise_twist_z_);
        // pass euler angular rates
        this->pose_msg_.twist.twist.angular.x = veul.X() +
          this->GaussianKernel(0, this->gaussian_noise_twist_x_);
        this->pose_msg_.twist.twist.angular.y = veul.Y() +
          this->GaussianKernel(0, this->gaussian_noise_twist_y_);
        this->pose_msg_.twist.twist.angular.z = veul.Z() +
          this->GaussianKernel(0, this->gaussian_noise_twist_z_);

        // fill in covariance matrix
        /// @todo: let user set separate linear and angular covariance values.
        this->pose_msg_.pose.covariance[0] = this->gaussian_noise_pos_x_ * this->gaussian_noise_pos_x_;
        this->pose_msg_.pose.covariance[7] = this->gaussian_noise_pos_y_ * this->gaussian_noise_pos_y_;
        this->pose_msg_.pose.covariance[14] = this->gaussian_noise_pos_z_ * this->gaussian_noise_pos_z_;
        this->pose_msg_.pose.covariance[21] = this->gaussian_noise_orientation_ * this->gaussian_noise_orientation_;
        this->pose_msg_.pose.covariance[28] = this->gaussian_noise_orientation_ * this->gaussian_noise_orientation_;
        this->pose_msg_.pose.covariance[35] = this->gaussian_noise_orientation_ * this->gaussian_noise_orientation_;

        this->pose_msg_.twist.covariance[0] = this->gaussian_noise_twist_x_ * this->gaussian_noise_twist_x_;
        this->pose_msg_.twist.covariance[7] = this->gaussian_noise_twist_y_ * this->gaussian_noise_twist_y_;
        this->pose_msg_.twist.covariance[14] = this->gaussian_noise_twist_z_ * this->gaussian_noise_twist_z_;
        this->pose_msg_.twist.covariance[21] = this->gaussian_noise_twist_x_ * this->gaussian_noise_twist_x_;
        this->pose_msg_.twist.covariance[28] = this->gaussian_noise_twist_y_ * this->gaussian_noise_twist_y_;
        this->pose_msg_.twist.covariance[35] = this->gaussian_noise_twist_z_ * this->gaussian_noise_twist_z_;

        // publish to ros
        this->pub_Queue->push(this->pose_msg_, this->pub_);
      }

      this->lock.unlock();

      // save last time stamp
      this->last_time_ = cur_time;
    }
  }
}

//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double GazeboRosRTK::GaussianKernel(double mu, double sigma)
{
  // using Box-Muller transform to generate two independent standard
  // normally disbributed normal variables see wikipedia

  // normalized uniform random variable
  double U = static_cast<double>(rand_r(&this->seed)) /
             static_cast<double>(RAND_MAX);

  // normalized uniform random variable
  double V = static_cast<double>(rand_r(&this->seed)) /
             static_cast<double>(RAND_MAX);

  double X = sqrt(-2.0 * ::log(U)) * cos(2.0*M_PI * V);
  // double Y = sqrt(-2.0 * ::log(U)) * sin(2.0*M_PI * V);

  // there are 2 indep. vars, we'll just use X
  // scale to our mu and sigma
  X = sigma * X + mu;
  return X;
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosRTK::RTKQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->rtk_queue_.callAvailable(ros::WallDuration(timeout));
  }
}
}
