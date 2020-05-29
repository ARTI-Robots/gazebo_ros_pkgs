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
 * Desc: Publishes state if inside or outside of ground wire area
 * Author: Michael Stradner
 * Date: 15 May 2020
 */

/*
 * Publishes state if inside or outside of ground wire area
 */

#ifndef GAZEBO_ROS_GROUND_WIRE_HH
#define GAZEBO_ROS_GROUND_WIRE_HH

#include <string>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>

#include <ros/callback_queue.h>
#include <ros/advertise_options.h>

#include <gazebo/physics/physics.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/common/Time.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/common/Events.hh>

#include <gazebo_plugins/PubQueue.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <visualization_msgs/Marker.h>

#include <std_msgs/Float64.h>

namespace gazebo
{
  using Coordinate = double;
  using Point = boost::geometry::model::d2::point_xy<Coordinate>;
  using Polygon = boost::geometry::model::polygon<Point>;

  class GazeboRosGroundWire : public ModelPlugin
  {
    /// \brief Constructor
    public: GazeboRosGroundWire();

    /// \brief Destructor
    public: virtual ~GazeboRosGroundWire();

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
    //private: ros::Publisher pub_;
    private: ros::Publisher pub_inside_wire_;
    private: PubQueue<std_msgs::Bool>::Ptr pub_queue_inside_;
    private: ros::Publisher pub_wire_marker_area_;
    private: PubQueue<visualization_msgs::Marker>::Ptr pub_queue_marker_area_;
    private: ros::Publisher pub_wire_dist_;
    private: PubQueue<std_msgs::Float64>::Ptr pub_queue_dist_;
    private: ros::Publisher pub_wire_angle_;
    private: PubQueue<std_msgs::Float64>::Ptr pub_queue_angle_;
    private: ros::Publisher pub_wire_marker_angle_;
    private: PubQueue<visualization_msgs::Marker>::Ptr pub_queue_marker_angle_;

    /// \brief ros message
    private: nav_msgs::Odometry pose_msg_;
    private: std_msgs::Bool inside_wire_msg_;

    /// \brief store bodyname
    private: std::string link_name_;

    /// \brief topic name
    private: std::string topic_namespace_;

    /// \brief frame transform name, should match link name
    /// FIXME: extract link name directly?
    private: std::string frame_name_;
    private: std::string tf_frame_name_;

    /// \brief RTK base position offset from the gazebo coordinates (ground wire coordinates should be in RTK frame)
    private: ignition::math::Pose3d offset_;

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


    struct point2d
    {
      double x = 0.0;
      double y = 0.0;
    };

    //private: std::vector<struct point2d> wire_area_;
    private: Polygon wire_polygon_;

    private:

    /// \brief for setting ROS name space
    private: std::string robot_namespace_;

    private: ros::CallbackQueue ground_wire_queue_;
    private: void GroundWireQueueThread();
    private: boost::thread callback_queue_thread_;

    // Pointer to the update event connection
    private: event::ConnectionPtr update_connection_;

    private: unsigned int seed;

    // ros publish multi queue, prevents publish() blocking
    private: PubMultiQueue pmq;


    /// \brief topic name
    private: std::string wire_config_;
    private: void getGroundWireArea(std::string file);
    //void calculateAreaIssue(double pos_x, double pos_y, bool *gps_available, double *add_noise);

    double getGroundWireDistance(ignition::math::Pose3d pose, double *centerPointX, double *centerPointY);
  };
}
#endif
