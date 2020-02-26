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

#include <yaml-cpp/yaml.h>

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

  if (_sdf->HasElement("topicNameGroundTruth"))
  {
    ground_truth_topic_name_ = _sdf->GetElement("topicNameGroundTruth")->Get<std::string>();
  }

  if (!_sdf->HasElement("frameName"))
  {
    ROS_DEBUG_NAMED("rtk", "gazebo_ros_rtk_gps plugin missing <frameName>, defaults to world");
    this->frame_name_ = "world";
  }
  else
    this->frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();

  if (!_sdf->HasElement("baseStationPose"))
  {
    ROS_DEBUG_NAMED("rtk", "gazebo_ros_rtk_gps plugin missing <baseStationPose>, defaults to origin");
  }
  else
  {
    this->offset_ = _sdf->GetElement("baseStationPose")->Get<ignition::math::Pose3d>();
  }

  if (_sdf->HasElement("xyzOffset"))
  {
    this->antenna_offset_ = _sdf->GetElement("xyzOffset")->Get<ignition::math::Vector3d>();
  }
  else
  {
    this->antenna_offset_.Set(0., 0., 0.);
  }

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

  if (!_sdf->HasElement("issueConfig"))
  {
    ROS_DEBUG_NAMED("rtk", "gazebo_ros_rtk_gps plugin missing <issueConfig>, defaults to ''");
    this->issue_config_ = "";
  }
  else
    this->issue_config_ = _sdf->GetElement("issueConfig")->Get<std::string>();

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

  if (ground_truth_topic_name_ != "")
  {
    this->pub_Queue = this->pmq.addPub<nav_msgs::Odometry>();
    pub_ground_truth_ = rosnode_->advertise<nav_msgs::Odometry>(ground_truth_topic_name_, 1);
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

  // Load GPS issue areas
  this->getGPSIssueAreas("");
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
        this->ground_truth_pose_msg_.header.frame_id = this->tf_frame_name_;
        this->ground_truth_pose_msg_.header.stamp.sec = cur_time.sec;
        this->ground_truth_pose_msg_.header.stamp.nsec = cur_time.nsec;

        this->ground_truth_pose_msg_.child_frame_id = this->link_name_;

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

        // Apply Constant Offsets to simulate base reference station at that position
        pose -= offset_;
        pose.Pos() -= pose.Rot() * this->antenna_offset_;

        vpos = offset_.Rot().RotateVectorReverse(vpos);
        veul = offset_.Rot().RotateVectorReverse(veul);

        // Fill out messages ground truth message
        ground_truth_pose_msg_.pose.pose.position.x    = pose.Pos().X();
        ground_truth_pose_msg_.pose.pose.position.y    = pose.Pos().Y();
        ground_truth_pose_msg_.pose.pose.position.z    = pose.Pos().Z();

        ground_truth_pose_msg_.pose.pose.orientation.x = pose.Rot().X();
        ground_truth_pose_msg_.pose.pose.orientation.y = pose.Rot().Y();
        ground_truth_pose_msg_.pose.pose.orientation.z = pose.Rot().Z();
        ground_truth_pose_msg_.pose.pose.orientation.w = pose.Rot().W();

        ground_truth_pose_msg_.twist.twist.linear.x  = vpos.X();
        ground_truth_pose_msg_.twist.twist.linear.y  = vpos.Y();
        ground_truth_pose_msg_.twist.twist.linear.z  = vpos.Z();
        // pass euler angular rates
        ground_truth_pose_msg_.twist.twist.angular.x = veul.X();
        ground_truth_pose_msg_.twist.twist.angular.y = veul.Y();
        ground_truth_pose_msg_.twist.twist.angular.z = veul.Z();

        // fill in covariance matrix
        ground_truth_pose_msg_.pose.covariance[0] = 0.;
        ground_truth_pose_msg_.pose.covariance[7] = 0.;
        ground_truth_pose_msg_.pose.covariance[14] = 0.;
        ground_truth_pose_msg_.pose.covariance[21] = 0.;
        ground_truth_pose_msg_.pose.covariance[28] = 0.;
        ground_truth_pose_msg_.pose.covariance[35] = 0.;

        ground_truth_pose_msg_.twist.covariance[0] = 0.;
        ground_truth_pose_msg_.twist.covariance[7] = 0.;
        ground_truth_pose_msg_.twist.covariance[14] = 0.;
        ground_truth_pose_msg_.twist.covariance[21] = 0.;
        ground_truth_pose_msg_.twist.covariance[28] = 0.;
        ground_truth_pose_msg_.twist.covariance[35] = 0.;

        // compute accelerations (not used)
        this->apos_ = (this->last_vpos_ - vpos) / tmp_dt;
        this->aeul_ = (this->last_veul_ - veul) / tmp_dt;
        this->last_vpos_ = vpos;
        this->last_veul_ = veul;

        this->frame_apos_ = (this->last_frame_vpos_ - frame_vpos) / tmp_dt;
        this->frame_aeul_ = (this->last_frame_veul_ - frame_veul) / tmp_dt;
        this->last_frame_vpos_ = frame_vpos;
        this->last_frame_veul_ = frame_veul;



        // Check if current position is inside an issue-area and if so, add noise or don't publish message in case of no GPS.
        bool gps_available = true;
        double add_noise = 0.0;
        calculateAreaIssue(pose.Pos().X(), pose.Pos().Y(), &gps_available, &add_noise);
        //ROS_ERROR_STREAM("#######################################################");
        //ROS_ERROR_STREAM("################ GPS-available = " << gps_available);
        //ROS_ERROR_STREAM("################     add_noise = " << add_noise);

        // Fill out messages
        this->pose_msg_.pose.pose.position.x    = pose.Pos().X() +
          this->GaussianKernel(0, this->gaussian_noise_pos_x_ + add_noise);
        this->pose_msg_.pose.pose.position.y    = pose.Pos().Y() +
          this->GaussianKernel(0, this->gaussian_noise_pos_y_ + add_noise);
        this->pose_msg_.pose.pose.position.z    = pose.Pos().Z() +
          this->GaussianKernel(0, this->gaussian_noise_pos_z_ + add_noise);

        this->pose_msg_.pose.pose.orientation.x = pose.Rot().X() +
          this->GaussianKernel(0, this->gaussian_noise_orientation_);
        this->pose_msg_.pose.pose.orientation.y = pose.Rot().Y() +
          this->GaussianKernel(0, this->gaussian_noise_orientation_);
        this->pose_msg_.pose.pose.orientation.z = pose.Rot().Z() +
          this->GaussianKernel(0, this->gaussian_noise_orientation_);
        this->pose_msg_.pose.pose.orientation.w = pose.Rot().W() +
          this->GaussianKernel(0, this->gaussian_noise_orientation_);

        this->pose_msg_.twist.twist.linear.x  = vpos.X() +
          this->GaussianKernel(0, this->gaussian_noise_twist_x_ + add_noise);
        this->pose_msg_.twist.twist.linear.y  = vpos.Y() +
          this->GaussianKernel(0, this->gaussian_noise_twist_y_ + add_noise);
        this->pose_msg_.twist.twist.linear.z  = vpos.Z() +
          this->GaussianKernel(0, this->gaussian_noise_twist_z_ + add_noise);
        // pass euler angular rates
        this->pose_msg_.twist.twist.angular.x = veul.X() +
          this->GaussianKernel(0, this->gaussian_noise_twist_x_);
        this->pose_msg_.twist.twist.angular.y = veul.Y() +
          this->GaussianKernel(0, this->gaussian_noise_twist_y_);
        this->pose_msg_.twist.twist.angular.z = veul.Z() +
          this->GaussianKernel(0, this->gaussian_noise_twist_z_);

        // fill in covariance matrix
        /// @todo: let user set separate linear and angular covariance values.
        this->pose_msg_.pose.covariance[0] = std::pow(this->gaussian_noise_pos_x_ + add_noise, 2.) * 1000.;
        this->pose_msg_.pose.covariance[7] = std::pow(this->gaussian_noise_pos_y_ + add_noise, 2.) * 1000.;
        this->pose_msg_.pose.covariance[14] = std::pow(this->gaussian_noise_pos_z_ + add_noise, 2.) * 1000.;
        this->pose_msg_.pose.covariance[21] = this->gaussian_noise_orientation_ * this->gaussian_noise_orientation_ * 1000.;
        this->pose_msg_.pose.covariance[28] = this->gaussian_noise_orientation_ * this->gaussian_noise_orientation_ * 1000.;
        this->pose_msg_.pose.covariance[35] = this->gaussian_noise_orientation_ * this->gaussian_noise_orientation_ * 1000.;

        this->pose_msg_.twist.covariance[0] = std::pow(this->gaussian_noise_twist_x_ + add_noise, 2.) * 1000.;
        this->pose_msg_.twist.covariance[7] = std::pow(this->gaussian_noise_twist_y_ + add_noise, 2.) * 1000.;
        this->pose_msg_.twist.covariance[14] = std::pow(this->gaussian_noise_twist_z_ + add_noise, 2.) * 1000.;
        this->pose_msg_.twist.covariance[21] = this->gaussian_noise_twist_x_ * this->gaussian_noise_twist_x_ * 1000.;
        this->pose_msg_.twist.covariance[28] = this->gaussian_noise_twist_y_ * this->gaussian_noise_twist_y_ * 1000.;
        this->pose_msg_.twist.covariance[35] = this->gaussian_noise_twist_z_ * this->gaussian_noise_twist_z_ * 1000.;

        // publish to ros in case GPS is available
        if(gps_available == true)
          this->pub_Queue->push(this->pose_msg_, this->pub_);

        if (pub_ground_truth_ && (pub_ground_truth_.getNumSubscribers() > 0))
        {
          pub_Queue->push(ground_truth_pose_msg_, pub_ground_truth_);
        }
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

////////////////////////////////////////////////////////////////////////////////
void GazeboRosRTK::getGPSIssueAreas(std::string file)
{
  if(this->issue_config_ == "")
    return;

  ROS_INFO_STREAM("ROS-RTK-GPS: Parsing file containing GPS-issue-areas: " <<  this->issue_config_);

  try
  {
    YAML::Node yaml = YAML::LoadFile(this->issue_config_);
    for (const YAML::Node& area : yaml["areas"])
    {
      double xa = area["p1"]["x"].as<double>();
      double ya = area["p1"]["y"].as<double>();
      double xb = area["p2"]["x"].as<double>();
      double yb = area["p2"]["y"].as<double>();
      double xc = area["p3"]["x"].as<double>();
      double yc = area["p3"]["y"].as<double>();
      double xd = area["p4"]["x"].as<double>();
      double yd = area["p4"]["y"].as<double>();
      double add_noise = area["additional_gaussian_noise"].as<double>();
      bool gps_available = area["gps_available"].as<bool>();

      struct rectangle rect;
      rect.xa = xa;
      rect.ya = ya;
      rect.xb = xb;
      rect.yb = yb;
      rect.xc = xc;
      rect.yc = yc;
      rect.xd = xd;
      rect.yd = yd;

      // Algorithm see: https://martin-thoma.com/how-to-check-if-a-point-is-inside-a-rectangle/
      // Calculate area from a rectangle-shaped polygon as described in https://www.mathopenref.com/coordpolygonarea.html
      double area_rect = 0.5 * std::abs((xa*yb - ya*xb) + (xb*yc - yb*xc) + (xc*yd - yc*xd) + (xd*ya - yd*xa));
      // Set area, noise and gps-flag
      rect.area_rect = area_rect;
      rect.add_noise = add_noise;
      rect.gps_available = gps_available;
      this->issue_areas.push_back(rect);
    }
  }
  catch (YAML::BadFile&)
  {
    ROS_ERROR_STREAM("File: '" << this->issue_config_ << "' not found");
  }
  catch (YAML::Exception& ex)
  {
    ROS_ERROR_STREAM("RTK-GPS-issue-config file can not be parsed: " << ex.what());
  }
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosRTK::calculateAreaIssue(double pos_x, double pos_y, bool *gps_available, double *add_noise)
{
  // Algorithm see: https://martin-thoma.com/how-to-check-if-a-point-is-inside-a-rectangle/
  *gps_available = true;
  *add_noise = 0.0;
  // Go through all GPS-issue-areas
  for(int i=0; i < this->issue_areas.size(); i++)
  {
    // Get corner-points of issue-rectangle
    double xa = this->issue_areas[i].xa;
    double ya = this->issue_areas[i].ya;
    double xb = this->issue_areas[i].xb;
    double yb = this->issue_areas[i].yb;
    double xc = this->issue_areas[i].xc;
    double yc = this->issue_areas[i].yc;
    double xd = this->issue_areas[i].xd;
    double yd = this->issue_areas[i].yd;

    // Calculate triangles using 2 corners and the current position
    double area_triangle_abp = 0.5*abs((xa*(yb-pos_y) + xb*(pos_y-ya) + pos_x*(ya-yb)));
    double area_triangle_bcp = 0.5*abs((xb*(yc-pos_y) + xc*(pos_y-yb) + pos_x*(yb-yc)));

    double area_triangle_cdp = 0.5*abs((xc*(yd-pos_y) + xd*(pos_y-yc) + pos_x*(yc-yd)));
    double area_triangle_dap = 0.5*abs((xd*(ya-pos_y) + xa*(pos_y-yd) + pos_x*(yd-ya)));

    // Sum up all rectangles
    double area_combined = area_triangle_abp + area_triangle_bcp + area_triangle_cdp + area_triangle_dap;

    /*ROS_INFO_STREAM("  AREA number " << i << " | pos_x = " << pos_x << " - pos_y = " << pos_y);
    ROS_INFO_STREAM("  rect-area     = " << this->issue_areas[i].area_rect);
    ROS_INFO_STREAM("  area_combined = " << area_combined);
    ROS_INFO_STREAM("  area_abp = " << area_triangle_abp);
    ROS_INFO_STREAM("  area_bcp = " << area_triangle_bcp);
    ROS_INFO_STREAM("  area_cdp = " << area_triangle_cdp);
    ROS_INFO_STREAM("  area_dap = " << area_triangle_dap);*/

    // If the combined area is larger than the area of the rectangle, then the point is outside of the rectangle
    bool point_inside = true;
    if(area_combined > this->issue_areas[i].area_rect)
    {
      point_inside = false;
      continue;
    }

    // If the current position is inside an issue-area, check if GPS is available at that area. If not, return no GPS.
    if(this->issue_areas[i].gps_available == false)
    {
      *add_noise = 0.0;
      *gps_available = false;
      return;
    }

    // If GPS is available, set the additional noise of this area. In case of overlapping areas, the one with the biggest noise is chosen.
    if(this->issue_areas[i].add_noise > *add_noise)
    {
      *add_noise = this->issue_areas[i].add_noise;
      *gps_available = true;
    }
  }
}

}
