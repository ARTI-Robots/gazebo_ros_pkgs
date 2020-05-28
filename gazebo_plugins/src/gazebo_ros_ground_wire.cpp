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

#include <string>
#include <tf/tf.h>
#include <stdlib.h>

#include <gazebo_plugins/gazebo_ros_ground_wire.h>

#include <yaml-cpp/yaml.h>

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(GazeboRosGroundWire);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosGroundWire::GazeboRosGroundWire()
{
  this->seed = 0;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
GazeboRosGroundWire::~GazeboRosGroundWire()
{
  this->update_connection_.reset();
  // Finalize the controller
  this->rosnode_->shutdown();
  this->ground_wire_queue_.clear();
  this->ground_wire_queue_.disable();
  this->callback_queue_thread_.join();
  delete this->rosnode_;
}

////////////////////////////////////////////////////////////////////////////////
// Load the controller
void GazeboRosGroundWire::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
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
    ROS_FATAL_NAMED("ground_wire", "gazebo_ros_ground_wire plugin missing <bodyName>, cannot proceed");
    return;
  }
  else
    this->link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();

  this->link_ = _parent->GetLink(this->link_name_);
  if (!this->link_)
  {
    ROS_FATAL_NAMED("ground_wire", "gazebo_ros_ground_wire plugin error: bodyName: %s does not exist\n",
      this->link_name_.c_str());
    return;
  }

  if (!_sdf->HasElement("topicName"))
  {
    ROS_FATAL_NAMED("ground_wire", "gazebo_ros_ground_wire plugin missing <topicName>, cannot proceed");
    return;
  }
  else
    this->topic_name_ = _sdf->GetElement("topicName")->Get<std::string>();

  if (!_sdf->HasElement("visualizeTopicName"))
  {
    ROS_DEBUG_NAMED("ground_wire", "gazebo_ros_ground_wire plugin missing <visualizeTopicName>, defaults to /wire_sensor/area");
    this->visualize_topic_name_ = "/wire_sensor/area";
  }
  else
    this->visualize_topic_name_ = _sdf->GetElement("visualizetopicName")->Get<std::string>();


  if (!_sdf->HasElement("frameName"))
  {
    ROS_DEBUG_NAMED("ground_wire", "gazebo_ros_ground_wire plugin missing <frameName>, defaults to world");
    this->frame_name_ = "world";
  }
  else
    this->frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();

  if (!_sdf->HasElement("updateRate"))
  {
    ROS_DEBUG_NAMED("ground_wire", "gazebo_ros_ground_wire plugin missing <updateRate>, defaults to 0.0"
             " (as fast as possible)");
    this->update_rate_ = 0;
  }
  else
    this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();

  if (!_sdf->HasElement("wireConfig"))
  {
    ROS_DEBUG_NAMED("ground_wire", "gazebo_ros_ground_wire plugin missing <wireConfig>, defaults to ''");
    this->wire_config_ = "";
  }
  else
    this->wire_config_ = _sdf->GetElement("wireConfig")->Get<std::string>();

  if (!_sdf->HasElement("baseStationPose"))
  {
    this->offset_.Set(0., 0., 0., 0., 0., 0.);
  }
  else
  {
    this->offset_ = _sdf->GetElement("baseStationPose")->Get<ignition::math::Pose3d>();
  }

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("gazebo_ros_ground_wire", "A ROS node for Gazebo has not been initialized, unable to load plugin. "
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
    this->pub_inside_wire_ = this->rosnode_->advertise<std_msgs::Bool>(this->topic_name_, 10);
    this->pub_queue_inside_ = this->pmq.addPub<std_msgs::Bool>();
  }

  if (this->visualize_topic_name_ != "")
  {
    this->pub_wire_area_ = this->rosnode_->advertise<visualization_msgs::Marker>(this->visualize_topic_name_, 10, true);
    this->pub_queue_marker_ = this->pmq.addPub<visualization_msgs::Marker>();
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
      ROS_ERROR_NAMED("ground_wire", "gazebo_ros_ground_wire plugin: frameName: %s does not exist, will"
                " not publish pose\n", this->frame_name_.c_str());
      return;
    }
  }

  // init reference frame state
  if (this->reference_link_)
  {
    ROS_DEBUG_NAMED("ground_wire", "got body %s", this->reference_link_->GetName().c_str());
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


  // start custom queue for ground_wire
  this->callback_queue_thread_ = boost::thread(
    boost::bind(&GazeboRosGroundWire::GroundWireQueueThread, this));

  // New Mechanism for Updating every World Cycle
  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->update_connection_ = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&GazeboRosGroundWire::UpdateChild, this));

  // Load GPS issue areas
  this->getGroundWireArea("");
}

////////////////////////////////////////////////////////////////////////////////
// Update the controller
void GazeboRosGroundWire::UpdateChild()
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
      ROS_WARN_NAMED("ground_wire", "Negative update time difference detected.");
      this->last_time_ = cur_time;
  }

  // rate control
  if (this->update_rate_ > 0 &&
      (cur_time-this->last_time_).Double() < (1.0/this->update_rate_))
    return;

  if (this->pub_inside_wire_.getNumSubscribers() > 0 || this->pub_wire_area_.getNumSubscribers() > 0)
  {
    // differentiate to get accelerations
    double tmp_dt = cur_time.Double() - this->last_time_.Double();
    if (tmp_dt != 0)
    {
      this->lock.lock();

      if (this->topic_name_ != "")
      {
        ignition::math::Pose3d pose, frame_pose;

        // Get Pose/Orientation
#if GAZEBO_MAJOR_VERSION >= 8
        pose = this->link_->WorldPose();
#else
        pose = this->link_->GetWorldPose().Ign();
#endif

        // Apply Reference Frame
        if (this->reference_link_)
        {
          // convert to relative pose
#if GAZEBO_MAJOR_VERSION >= 8
          frame_pose = this->reference_link_->WorldPose();
#else
          frame_pose = this->reference_link_->GetWorldPose().Ign();
#endif
          pose.Pos() = pose.Pos() - frame_pose.Pos();
          pose.Pos() = frame_pose.Rot().RotateVectorReverse(pose.Pos());
          pose.Rot() *= frame_pose.Rot().Inverse();
        }

        // Get current position in point
        Point point(pose.Pos().X(), pose.Pos().Y());
        // Check if point is inside polygon
        bool pointInside = boost::geometry::within(point, this->wire_polygon_);
        // Set state in inside-wire-area-message
        inside_wire_msg_.data = pointInside;

        //ROS_INFO_STREAM("#######################################################");
        //ROS_INFO_STREAM("################ inside = " << pointInside);
        //ROS_INFO_STREAM("################ point(x,y) = " << pose.Pos().X() << " | " << pose.Pos().Y());

        // Publish inside-wire-area-message
        if (pub_inside_wire_ && (pub_inside_wire_.getNumSubscribers() > 0))
        {
          pub_queue_inside_->push(inside_wire_msg_, pub_inside_wire_);
        }
      }

      this->lock.unlock();

      // save last time stamp
      this->last_time_ = cur_time;
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Put laser data to the interface
void GazeboRosGroundWire::GroundWireQueueThread()
{
  static const double timeout = 0.01;

  while (this->rosnode_->ok())
  {
    this->ground_wire_queue_.callAvailable(ros::WallDuration(timeout));
  }
}

////////////////////////////////////////////////////////////////////////////////
void GazeboRosGroundWire::getGroundWireArea(std::string file)
{
  if(this->wire_config_ == "")
    return;

  ROS_INFO_STREAM("ROS-Ground_Wire: Parsing file containing ground wire area: " <<  this->wire_config_);

  try
  {
    // Load wire-area-yaml
    YAML::Node yaml = YAML::LoadFile(this->wire_config_);
    // Create vector for points in polygon
    std::vector<struct point2d> wireArea;
    // Create visualization marker for wire area
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time();
    marker.ns = "wire_area";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;
    marker.scale.x = 0.1;
    marker.color.g = 1.0;
    marker.color.a = 1.0;

    double yaw_angle = -offset_.Rot().Yaw();

    for (const YAML::Node& area : yaml["area"])
    {
      // Get coordinate from yaml
      double x = area["point"]["x"].as<double>();
      double y = area["point"]["y"].as<double>();
      double z = area["point"]["z"].as<double>();

      // However, if the RTK-base-station is rotated, it doesn't align with the world-frame anymore. Therefore add a yaw-rotation to the points
      double rotatedX = cos(yaw_angle) * (x) - sin(yaw_angle) * (y);
      double rotatedY = sin(yaw_angle) * (x) + cos(yaw_angle) * (y);
      x = rotatedX;
      y = rotatedY;

      // Add point to polygon
      struct point2d point;
      wireArea.push_back(point);
      boost::geometry::append(this->wire_polygon_, Point(x, y));

      // Add point to visualization marker
      geometry_msgs::Point p;
      p.x = x;
      p.y = y;
      p.z = z;
      marker.points.push_back(p);
    }
    // In case the polygon is not closed, close it regardless
    geometry_msgs::Point front = marker.points.front();
    marker.points.push_back(front);

    // Correct the polygon (for example if the closing point has been forgotten or the order of points is not clockwise):
    boost::geometry::correct(this->wire_polygon_);

    //ROS_ERROR_STREAM("Got wire: " << boost::geometry::wkt(this->wire_polygon_));

    // Publish the wire-area visualization marker (latched topic)
    if (pub_wire_area_)
    {
      pub_queue_marker_->push(marker, pub_wire_area_);
    }
  }
  catch (YAML::BadFile&)
  {
    ROS_ERROR_STREAM("File: '" << this->wire_config_ << "' not found");
  }
  catch (YAML::Exception& ex)
  {
    ROS_ERROR_STREAM("Ground-wire-config file can not be parsed: " << ex.what());
  }
}

}
