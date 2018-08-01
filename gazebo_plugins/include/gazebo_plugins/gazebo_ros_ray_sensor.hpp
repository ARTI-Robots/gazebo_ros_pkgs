// Copyright 2018 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef GAZEBO_PLUGINS__GAZEBO_ROS_RAY_SENSOR_HPP_
#define GAZEBO_PLUGINS__GAZEBO_ROS_RAY_SENSOR_HPP_

#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/sensors/RaySensor.hh>
#include <gazebo/sensors/GpuRaySensor.hh>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <gazebo_ros/node.hpp>
#include <boost/variant.hpp>


#include <string>
#include <memory>


namespace gazebo_plugins
{

class GazeboRosRaySensorPrivate;

/// Plugin to attach to a gazebo ray or gpu_ray sensor and publish its data to ROS
/**
  \details
  SDF parameters:
  \verbatim
  <outputType>: Optional. The message type of the plugin's output. Can be any of the following:
    * sensor_msgs/PointCloud2: 3D cloud of points, Default
    * sensor_msgs/PointCloud:  3D cloud of points
    * sensor_msgs/LaserScan:   2D scan, uses center vertical ray if there are multiple
    * sensor_msgs/Range:       Single distance value, minimum of all ray ranges of parent sensor
  \endverbatim

  \verbatim
  <frameName>: TF Frame id of output header.
               If not set, frame id will be name of sensor's parent link
  \endverbatim

  \verbatim
  <minIntensity>: Clip intensity values for output to this value.
                  Default: 0.0
  \endverbatim

  \verbatim
  <radiationType>: The radiation type to publish when the output type is sensor_msgs/Range.
                   Can be either "infrared" or "ultrasonic".
                   Default: "infrared".
  \endverbatim

  Example SDF:
  \code{.xml}
    <plugin name="my_ray_senor_plugin" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <!-- Configure namespace and remap to publish to /ray/pointcloud2 -->
        <namespace>/ray</namespace>
        <argument>~/out:=pointcloud2</argument>
      </ros>
      <!-- Output as a PointCloud2, see above for other types -->
      <outputType>sensor_msgs/PointCloud2</outputType>
      <!-- Clip intensity values so all are above 100, optional -->
      <minIntensity>100.0</minIntensity>
      <!-- Frame id for header of output, defaults to sensor's parent link name -->
      <frameName>ray_link</frameName>
    </plugin>
  \endcode
*/
class GazeboRosRaySensor : public gazebo::SensorPlugin
{
public:
  /// \brief Constructor
  GazeboRosRaySensor();

  /// \brief Destructor
  virtual ~GazeboRosRaySensor();

  // Documentation Inherited
  void Load(gazebo::sensors::SensorPtr _parent, sdf::ElementPtr _sdf);

private:
  std::unique_ptr<GazeboRosRaySensorPrivate> impl_;
};

}  // namespace gazebo_plugins

#endif  // GAZEBO_PLUGINS__GAZEBO_ROS_RAY_SENSOR_HPP_