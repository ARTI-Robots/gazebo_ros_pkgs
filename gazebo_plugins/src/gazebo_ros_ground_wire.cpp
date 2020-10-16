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

#include <boost/geometry/geometries/segment.hpp>

#include "gazebo/physics/PhysicsTypes.hh"

namespace gazebo
{
GZ_REGISTER_MODEL_PLUGIN(GazeboRosGroundWire);

////////////////////////////////////////////////////////////////////////////////
// Constructor
GazeboRosGroundWire::GazeboRosGroundWire()
  : tf_listener_(tf_buffer_)
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
  {
    this->robot_namespace_ =
      _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";
  }

  if (!_sdf->HasElement("bodyName"))
  {
    ROS_FATAL_NAMED("ground_wire", "gazebo_ros_ground_wire plugin missing <bodyName>, cannot proceed");
    return;
  }
  else
  {
    this->link_name_ = _sdf->GetElement("bodyName")->Get<std::string>();
  }

  if (!_sdf->HasElement("sensorFrame"))
  {
    ROS_FATAL_NAMED("ground_wire", "gazebo_ros_ground_wire plugin missing <sensorFrame>, cannot proceed");
    return;
  }
  else
  {
    this->sensor_frame_name_ = _sdf->GetElement("sensorFrame")->Get<std::string>();
  }

  if (!_sdf->HasElement("sensorFrameLeft"))
  {
    ROS_FATAL_NAMED("ground_wire", "gazebo_ros_ground_wire plugin missing <sensorFrameLeft>, cannot proceed");
    return;
  }
  else
  {
    this->sensor_frame_left_name_ = _sdf->GetElement("sensorFrameLeft")->Get<std::string>();
  }

  if (!_sdf->HasElement("sensorFrameRight"))
  {
    ROS_FATAL_NAMED("ground_wire", "gazebo_ros_ground_wire plugin missing <sensorFrameRight>, cannot proceed");
    return;
  }
  else
  {
    this->sensor_frame_right_name_ = _sdf->GetElement("sensorFrameRight")->Get<std::string>();
  }

  if (!_sdf->HasElement("maxSensorDist"))
  {
    ROS_FATAL_NAMED("ground_wire", "gazebo_ros_ground_wire plugin missing <maxSensorDist>, cannot proceed");
    return;
  }
  else
  {
    this->max_sensor_dist_ = _sdf->GetElement("maxSensorDist")->Get<double>();
  }

  this->link_ = _parent->GetLink(this->link_name_);
  if (!this->link_)
  {
    ROS_FATAL_NAMED("ground_wire", "gazebo_ros_ground_wire plugin error: bodyName: %s does not exist\n",
                    this->link_name_.c_str());
    return;
  }

  if (!_sdf->HasElement("topicNamespace"))
  {
    ROS_FATAL_NAMED("ground_wire", "gazebo_ros_ground_wire plugin missing <topicNamespace>, cannot proceed");
    return;
  }
  else
  {
    this->topic_namespace_ = _sdf->GetElement("topicNamespace")->Get<std::string>();
  }

  if (!_sdf->HasElement("frameName"))
  {
    ROS_DEBUG_NAMED("ground_wire", "gazebo_ros_ground_wire plugin missing <frameName>, defaults to world");
    this->frame_name_ = "world";
  }
  else
  {
    this->frame_name_ = _sdf->GetElement("frameName")->Get<std::string>();
  }

  if (!_sdf->HasElement("updateRate"))
  {
    ROS_DEBUG_NAMED("ground_wire", "gazebo_ros_ground_wire plugin missing <updateRate>, defaults to 0.0"
                                   " (as fast as possible)");
    this->update_rate_ = 0;
  }
  else
  {
    this->update_rate_ = _sdf->GetElement("updateRate")->Get<double>();
  }

  if (!_sdf->HasElement("wireConfig"))
  {
    ROS_DEBUG_NAMED("ground_wire", "gazebo_ros_ground_wire plugin missing <wireConfig>, defaults to ''");
    this->wire_config_ = "";
  }
  else
  {
    this->wire_config_ = _sdf->GetElement("wireConfig")->Get<std::string>();
  }

  if (!_sdf->HasElement("baseStationPose"))
  {
    this->offset_.Set(0., 0., 0., 0., 0., 0.);
  }
  else
  {
    this->offset_ = _sdf->GetElement("baseStationPose")->Get<ignition::math::Pose3d>();
  }

  if (!_sdf->HasElement("closeToBaseDistance"))
  {
    this->close_to_base_distance_ = 1.;
  }
  else
  {
    this->close_to_base_distance_ = _sdf->GetElement("closeToBaseDistance")->Get<double>();
  }

  // Make sure the ROS node for Gazebo has already been initialized
  if (!ros::isInitialized())
  {
    ROS_FATAL_STREAM_NAMED("gazebo_ros_ground_wire",
                           "A ROS node for Gazebo has not been initialized, unable to load plugin. "
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

  if (this->topic_namespace_ != "")
  {
    this->pub_inside_wire_ = this->rosnode_->advertise<std_msgs::Bool>(this->topic_namespace_ + "/state", 10);
    this->pub_queue_inside_ = this->pmq.addPub<std_msgs::Bool>();

    this->pub_wire_dist_ = this->rosnode_->advertise<std_msgs::Float64>(this->topic_namespace_ + "/dist", 10);
    this->pub_queue_dist_ = this->pmq.addPub<std_msgs::Float64>();

    this->pub_wire_angle_ = this->rosnode_->advertise<std_msgs::Float64>(this->topic_namespace_ + "/angle", 10);
    this->pub_queue_angle_ = this->pmq.addPub<std_msgs::Float64>();

    this->pub_wire_marker_area_ = this->rosnode_->advertise<visualization_msgs::Marker>(
      this->topic_namespace_ + "/marker/area", 10, true);
    this->pub_queue_marker_area_ = this->pmq.addPub<visualization_msgs::Marker>();

    this->pub_wire_marker_angle_ = this->rosnode_->advertise<visualization_msgs::Marker>(
      this->topic_namespace_ + "/marker/angle", 10, true);
    this->pub_queue_marker_angle_ = this->pmq.addPub<visualization_msgs::Marker>();

    this->pub_wire_msg_ = this->rosnode_->advertise<gazebo_msgs::WireSensorState>(this->topic_namespace_ + "/raw", 10,
                                                                                  true);
    this->pub_queue_wire_msg_ = this->pmq.addPub<gazebo_msgs::WireSensorState>();
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
    ROS_ERROR("MICHAEL_DEBUG got body %s", this->reference_link_->GetName().c_str());
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

  // Just to be on the safe side, initialize the wire_sensor_poses with zeros
  wire_sensor_pose_.Set(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  wire_sensor_left_pose_.Set(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  wire_sensor_right_pose_.Set(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

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
  {
    return;
  }

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
      (cur_time - this->last_time_).Double() < (1.0 / this->update_rate_))
  {
    return;
  }

  if (this->pub_inside_wire_.getNumSubscribers() > 0 || this->pub_wire_marker_area_.getNumSubscribers() > 0
      || this->pub_wire_msg_.getNumSubscribers() > 0)
  {
    // differentiate to get accelerations
    double tmp_dt = cur_time.Double() - this->last_time_.Double();
    if (tmp_dt != 0)
    {
      this->lock.lock();

      if (this->topic_namespace_ != "")
      {
        ignition::math::Pose3d pose, frame_pose, pose_sensor_left, pose_sensor_right;

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

        // In case the sensor-frame is not the same as the Gazebo-model-frame
        if (this->sensor_frame_name_ != this->link_->GetName().c_str())
        {
          getSensorFrameTransform();
          ignition::math::Pose3d rot = this->wire_sensor_pose_.RotatePositionAboutOrigin(pose.Rot().Inverse());
          pose.Set(pose.Pos() + rot.Pos(), pose.Rot());
        }

        // Transform the left wire-sensor-offset according to the current orientation of the robot
        ignition::math::Pose3d rotLeft = this->wire_sensor_left_pose_.RotatePositionAboutOrigin(pose.Rot().Inverse());
        pose_sensor_left.Set(pose.Pos() + rotLeft.Pos(), pose_sensor_left.Rot());

        // Transform the right wire-sensor-offset according to the current orientation of the robot
        ignition::math::Pose3d rotRight = this->wire_sensor_right_pose_.RotatePositionAboutOrigin(pose.Rot().Inverse());
        pose_sensor_right.Set(pose.Pos() + rotRight.Pos(), pose_sensor_right.Rot());


        // Get current position in point
        Point point(pose.Pos().X(), pose.Pos().Y());

        // Get current position of the wire_frame_sensor links
        getSensorFrameTransform();

        // In case the sensor-frame is not the same as the model-frame
        if (this->sensor_frame_name_ != this->link_->GetName().c_str())
        {
          // Get current position of the model-frame plus the sensor-offset
          //getSensorFrameTransform();
          Point point(pose.Pos().X() + this->wire_sensor_pose_.Pos().X(),
                      pose.Pos().Y() + this->wire_sensor_pose_.Pos().Y());
        }
        else
        {
          // Get current position in point
          Point point(pose.Pos().X(), pose.Pos().Y());
        }

        // Check if point is inside polygon
        bool pointInside = boost::geometry::within(point, this->wire_polygon_);
        // Set state in inside-wire-area-message
        inside_wire_msg_.data = pointInside;

        // Publish inside-wire-area-message
        if (pub_inside_wire_ && (pub_inside_wire_.getNumSubscribers() > 0))
        {
          pub_queue_inside_->push(inside_wire_msg_, pub_inside_wire_);
        }

        double centerPointX = 0.0;
        double centerPointY = 0.0;
        double dist = getGroundWireDistance(pose, &centerPointX, &centerPointY);

        std_msgs::Float64 dist_msg;
        dist_msg.data = dist;
        pub_queue_dist_->push(dist_msg, pub_wire_dist_);

        //--------------------------------------------------------------------------------------------------------------

        // Get intensity for left and right wire sensor
        Point pointLeft(pose_sensor_left.Pos().X(), pose_sensor_left.Pos().Y());
        Point pointRight(pose_sensor_right.Pos().X(), pose_sensor_right.Pos().Y());

        gazebo_msgs::WireSensorState state;
        state.header.stamp = ros::Time::now();

        // Check if left sensor is inside polygon
        bool inside_wire_left = boost::geometry::within(pointLeft, this->wire_polygon_);

        // Calculate intensity for left wire sensor
        double centerPointLeftX = 0.0;
        double centerPointLeftY = 0.0;
        double distLeft = getGroundWireDistance(pose_sensor_left, &centerPointLeftX, &centerPointLeftY);

        if (inside_wire_left)
        {
          if (fabs(distLeft) > 0.02)
          {
            state.inside_wire_left = true;
          }
          else
          {
            state.inside_wire_left = old_wire_state_.inside_wire_left;
          }
        }
        else
        {
          if (fabs(distLeft) > 0.05)
          {
            state.inside_wire_left = false;
          }
          else
          {
            state.inside_wire_left = old_wire_state_.inside_wire_left;
          }
        }

        if (fabs(distLeft) < fabs(this->max_sensor_dist_))
        {
          double distance_in_cm = distLeft * 100.;
          if (state.inside_wire_left)
          {
            double lower_distance_cubic_fit = 5.91744 * std::pow(std::min(distance_in_cm, 6.), 3.)
                                              - 130.323 * std::pow(std::min(distance_in_cm, 6.), 2.)
                                              + 922.925 * std::min(distance_in_cm, 6.) + 585.525;
            double higher_distance_quadratic_fit =
              std::pow(-1.77747 * distance_in_cm, 2.) + 43.1582 * distance_in_cm + 2456.73;
            state.intensity_left = std::min(lower_distance_cubic_fit, higher_distance_quadratic_fit);
          }
          else
          {
            double lower_distance_cubic_fit = 7.21212 * std::pow(std::max(distance_in_cm, -6.), 3.)
                                              + 39.474 * std::pow(std::max(distance_in_cm, -6.), 2.)
                                              - 380.078 * std::max(distance_in_cm, -6.) + 486.55;
            double higher_distance_quadratic_fit =
              -2.79371 * std::pow(distance_in_cm, 2.) - 63.5622 * distance_in_cm + 2240.77;
            state.intensity_left = std::min(lower_distance_cubic_fit, higher_distance_quadratic_fit);
          }

          state.intensity_left += left_coil_offset_ + this->GaussianKernel(0, 50);
        }
        else
        {
          state.intensity_left = 0.0;
        }

        double distanceToBaseLeft = getDistanceToBase(pose_sensor_left);
        if (fabs(distanceToBaseLeft) < fabs(this->close_to_base_distance_))
        {
          state.base_close_left = true;
        }
        else
        {
          state.base_close_left = false;
        }

        // Check if right sensor is inside polygon
        bool inside_wire_right  = boost::geometry::within(pointRight, this->wire_polygon_);

        // Calculate intensity for right wire sensor
        double centerPointRightX = 0.0;
        double centerPointRightY = 0.0;
        double distRight = getGroundWireDistance(pose_sensor_right, &centerPointRightX, &centerPointRightY);

        if (inside_wire_right)
        {
          if (fabs(distRight) > 0.02)
          {
            state.inside_wire_right = true;
          }
          else
          {
            state.inside_wire_right = old_wire_state_.inside_wire_right;
          }
        }
        else
        {
          if (fabs(distRight) > 0.05)
          {
            state.inside_wire_right = false;
          }
          else
          {
            state.inside_wire_right = old_wire_state_.inside_wire_right;
          }
        }

        if (fabs(distRight) < fabs(this->max_sensor_dist_))
        {
          double distance_in_cm = distRight * 100.;
          if (state.inside_wire_right)
          {
            double lower_distance_cubic_fit = 5.91744 * std::pow(std::min(distance_in_cm, 6.), 3.)
                                              - 130.323 * std::pow(std::min(distance_in_cm, 6.), 2.)
                                              + 922.925 * std::min(distance_in_cm, 6.) + 585.525;
            double higher_distance_quadratic_fit =
              std::pow(-1.77747 * distance_in_cm, 2.) + 43.1582 * distance_in_cm + 2456.73;
            state.intensity_right = std::min(lower_distance_cubic_fit, higher_distance_quadratic_fit);
          }
          else
          {
            double lower_distance_cubic_fit = 7.21212 * std::pow(std::max(distance_in_cm, -6.), 3.)
                                              + 39.474 * std::pow(std::max(distance_in_cm, -6.), 2.)
                                              - 380.078 * std::max(distance_in_cm, -6.) + 486.55;
            double higher_distance_quadratic_fit =
              -2.79371 * std::pow(distance_in_cm, 2.) - 63.5622 * distance_in_cm + 2240.77;
            state.intensity_right = std::min(lower_distance_cubic_fit, higher_distance_quadratic_fit);
          }

          state.intensity_right += right_coil_offset_ + this->GaussianKernel(0, 50);
        }
        else
        {
          state.intensity_right = 0.0;
        }

        double distanceToBaseRight = getDistanceToBase(pose_sensor_right);
        if (fabs(distanceToBaseRight) < fabs(this->close_to_base_distance_))
        {
          state.base_close_right = true;
        }
        else
        {
          state.base_close_right = false;
        }

        old_wire_state_ = state;

        this->pub_queue_wire_msg_->push(state, pub_wire_msg_);

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
  if (this->wire_config_ == "")
  {
    return;
  }

  ROS_INFO_STREAM("ROS-Ground_Wire: Parsing file containing ground wire area: " << this->wire_config_);

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

      // Add point to polygon
      struct point2d point;
      wireArea.push_back(point);
      boost::geometry::append(this->wire_polygon_, Point(x, y));

      // Add point to visualization marker
      geometry_msgs::Point p;
      p.x = rotatedX;
      p.y = rotatedY;
      p.z = z;
      marker.points.push_back(p);
    }
    // In case the polygon is not closed, close it regardless
    geometry_msgs::Point front = marker.points.front();
    marker.points.push_back(front);

    // Correct the polygon (for example if the closing point has been forgotten or the order of points is not clockwise):
    boost::geometry::correct(this->wire_polygon_);

    // Publish the wire-area visualization marker (latched topic)
    if (pub_wire_marker_area_)
    {
      pub_queue_marker_area_->push(marker, pub_wire_marker_area_);
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

double GazeboRosGroundWire::getGroundWireDistance(
  ignition::math::Pose3d pose, double* centerPointX, double* centerPointY)
{
  const Point pt(pose.Pos().X(), pose.Pos().Y());
  auto distance = std::numeric_limits<float>::max();

  boost::geometry::for_each_segment(this->wire_polygon_,
                                    [&distance, &pt, centerPointX, centerPointY](const auto& segment) {
                                      double d = distance;
                                      distance = std::min<float>(distance, boost::geometry::distance(segment, pt));

                                      if (distance < d)
                                      {
                                        // Get the segment edge points of the nearest segment
                                        double x1 = segment.first.x();
                                        double y1 = segment.first.y();
                                        double x2 = segment.second.x();
                                        double y2 = segment.second.y();

                                        // Calculate the nearest point of the segment to the robot
                                        double a = pt.x() - x1;
                                        double b = pt.y() - y1;
                                        double c = x2 - x1;
                                        double d = y2 - y1;

                                        double dot = a * c + b * d;
                                        double len_sq = c * c + d * d;
                                        double param = -1;
                                        if (len_sq != 0)  // If the length is zer0
                                        {
                                          param = dot / len_sq;
                                        }

                                        if (param < 0)
                                        {
                                          // Nearest point is not on the segment, therefore use the nearest edge point
                                          *centerPointX = x1;
                                          *centerPointY = y1;
                                        }
                                        else if (param > 1)
                                        {
                                          // Nearest point is not on the segment, therefore use the nearest edge point
                                          *centerPointX = x2;
                                          *centerPointY = y2;
                                        }
                                        else
                                        {
                                          // Nearest point is on the segment
                                          *centerPointX = x1 + param * c;
                                          *centerPointY = y1 + param * d;
                                        }
                                      }
                                    });

  double robot_direction = pose.Rot().Yaw();
  const Point vRobot(cos(robot_direction), sin(robot_direction));
  const Point vWire(*centerPointX - pt.x(), *centerPointY - pt.y());

  double dot = vRobot.x() * vWire.x() + vRobot.y() * vWire.y();
  double det = vRobot.x() * vWire.y() - vRobot.y() * vWire.x();
  double angle = atan2(det, dot);

  if (this->pub_wire_angle_.getNumSubscribers() > 0 || this->pub_wire_marker_angle_.getNumSubscribers() > 0)
  {
    double offset_angle = -offset_.Rot().Yaw();

    std_msgs::Float64 msgAngle;
    msgAngle.data = angle;
    if (pub_wire_angle_)
    {
      pub_queue_angle_->push(msgAngle, pub_wire_angle_);
    }

    if (pub_wire_marker_angle_)
    {
      // Create visualization marker for wire angle
      visualization_msgs::Marker marker;
      marker.header.frame_id = "world";
      marker.header.stamp = ros::Time();
      marker.ns = "wire_angle";
      marker.id = 0;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;
      marker.color.b = 1.0;
      marker.color.g = 0.1;
      marker.color.a = 1.0;
      marker.scale.x = 0.02;
      marker.scale.y = 0.045;
      marker.scale.z = 0.15;

      // However, if the RTK-base-station is rotated, it doesn't align with the world-frame anymore. Therefore add a yaw-rotation to the points
      geometry_msgs::Point p1;
      p1.x = cos(offset_angle) * (pose.Pos().X()) - sin(offset_angle) * (pose.Pos().Y());  // pose.Pos().X()
      p1.y = sin(offset_angle) * (pose.Pos().X()) + cos(offset_angle) * (pose.Pos().Y());  // pose.Pos().Y()
      p1.z = pose.Pos().Z();
      marker.points.push_back(p1);
      geometry_msgs::Point p2;
      p2.x = cos(offset_angle) * (*centerPointX) - sin(offset_angle) * (*centerPointY);  // *centerPointX
      p2.y = sin(offset_angle) * (*centerPointX) + cos(offset_angle) * (*centerPointY);  // *centerPointY
      p2.z = pose.Pos().Z();
      marker.points.push_back(p2);
      pub_queue_marker_angle_->push(marker, pub_wire_marker_angle_);
    }
  }
  return distance;
}

void GazeboRosGroundWire::getSensorFrameTransform()
{
  try
  {
    geometry_msgs::TransformStamped trans = tf_buffer_.lookupTransform(this->link_->GetName().c_str(),
                                                                       this->sensor_frame_name_, ros::Time(0));

    tf::Quaternion quat;
    tf::quaternionMsgToTF(trans.transform.rotation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    wire_sensor_pose_.Set(trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z,
                          roll, pitch, yaw);

    //------------------------------------------------------------------------------------------------------------------

    geometry_msgs::TransformStamped transLeft = tf_buffer_.lookupTransform(this->sensor_frame_name_,
                                                                           this->sensor_frame_left_name_, ros::Time(0));

    tf::Quaternion quatLeft;
    tf::quaternionMsgToTF(transLeft.transform.rotation, quatLeft);
    double rollLeft, pitchLeft, yawLeft;
    tf::Matrix3x3(quatLeft).getRPY(rollLeft, pitchLeft, yawLeft);

    wire_sensor_left_pose_.Set(transLeft.transform.translation.x, transLeft.transform.translation.y,
                               transLeft.transform.translation.z, rollLeft, pitchLeft, yawLeft);

    //------------------------------------------------------------------------------------------------------------------

    geometry_msgs::TransformStamped transRight = tf_buffer_.lookupTransform(this->sensor_frame_name_,
                                                                            this->sensor_frame_right_name_,
                                                                            ros::Time(0));

    tf::Quaternion quatRight;
    tf::quaternionMsgToTF(transRight.transform.rotation, quatRight);
    double rollRight, pitchRight, yawRight;
    tf::Matrix3x3(quatRight).getRPY(rollRight, pitchRight, yawRight);

    wire_sensor_right_pose_.Set(transRight.transform.translation.x, transRight.transform.translation.y,
                                transRight.transform.translation.z, rollRight, pitchRight, yawRight);

  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR("%s", ex.what());
  }
}

double GazeboRosGroundWire::getDistanceToBase(ignition::math::Pose3d pose)
{
  return pose.Pos().Distance(this->offset_.Pos());
}

//////////////////////////////////////////////////////////////////////////////
// Utility for adding noise
double GazeboRosGroundWire::GaussianKernel(double mu, double sigma)
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
}
