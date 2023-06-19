// Copyright 2017-2020 the Autoware Foundation
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <common/types.hpp>
#include <point_cloud_filter_transform_nodes/point_cloud_filter_transform_node.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <memory>
#include <string>
#include <map>
#include <vector>

namespace autoware
{
namespace perception
{
namespace filters
{
/// \brief Boilerplate Apex.OS nodes around point_cloud_filter_transform_nodes
namespace point_cloud_filter_transform_nodes
{
using autoware::common::lidar_utils::has_intensity_and_throw_if_no_xyz;
using autoware::common::lidar_utils::sanitize_point_cloud;
using autoware::common::types::float64_t;
using autoware::common::types::PointXYZI;
using autoware::common::types::PointXYZIF;
using geometry_msgs::msg::TransformStamped;
using sensor_msgs::msg::PointCloud2;

TransformStamped get_transform(
  const std::string & input_frame_id,
  const std::string & output_frame_id,
  float64_t r_x, float64_t r_y, float64_t r_z, float64_t r_w, float64_t t_x,
  float64_t t_y, float64_t t_z)
{
  TransformStamped ret;
  ret.header.frame_id = input_frame_id;
  ret.child_frame_id = output_frame_id;
  ret.transform.rotation.x = r_x;
  ret.transform.rotation.y = r_y;
  ret.transform.rotation.z = r_z;
  ret.transform.rotation.w = r_w;
  ret.transform.translation.x = t_x;
  ret.transform.translation.y = t_y;
  ret.transform.translation.z = t_z;
  return ret;
}

PointCloud2FilterTransformNode::PointCloud2FilterTransformNode(
  const rclcpp::NodeOptions & node_options)
: Node("point_cloud_filter_transform_node", node_options),
  m_angle_filter{
    static_cast<float32_t>(declare_parameter("start_angle").get<float64_t>()),
    static_cast<float32_t>(declare_parameter("end_angle").get<float64_t>())},
  m_distance_filter{
    static_cast<float32_t>(declare_parameter("min_radius").get<float64_t>()),
    static_cast<float32_t>(declare_parameter("max_radius").get<float64_t>())},
  m_input_frame_id{declare_parameter("input_frame_id").get<std::string>()},
  m_output_frame_id{declare_parameter("output_frame_id").get<std::string>()},
  m_init_timeout{std::chrono::milliseconds{declare_parameter("init_timeout_ms").get<int32_t>()}},
  m_timeout{std::chrono::milliseconds{declare_parameter("timeout_ms").get<int32_t>()}},
  m_sub_ptr{create_subscription<PointCloud2>(
      "points_in", rclcpp::QoS{10},
      std::bind(
        &PointCloud2FilterTransformNode::process_filtered_transformed_message, this, _1))},
  m_pub_ptr{create_publisher<PointCloud2>("points_filtered", rclcpp::QoS{10})},
  m_expected_num_publishers{
    static_cast<size_t>(declare_parameter("expected_num_publishers").get<int32_t>())},
  m_expected_num_subscribers{
    static_cast<size_t>(declare_parameter("expected_num_subscribers").get<int32_t>())},
  m_pcl_size{static_cast<std::uint32_t>(declare_parameter("pcl_size").get<uint32_t>())}
{  /// Declare transform parameters with the namespace
  this->declare_parameter("static_transformer.quaternion.x");
  this->declare_parameter("static_transformer.quaternion.y");
  this->declare_parameter("static_transformer.quaternion.z");
  this->declare_parameter("static_transformer.quaternion.w");
  this->declare_parameter("static_transformer.translation.x");
  this->declare_parameter("static_transformer.translation.y");
  this->declare_parameter("static_transformer.translation.z");

  /// Declare objects to hold transform parameters
  rclcpp::Parameter quat_x_param;
  rclcpp::Parameter quat_y_param;
  rclcpp::Parameter quat_z_param;
  rclcpp::Parameter quat_w_param;
  rclcpp::Parameter trans_x_param;
  rclcpp::Parameter trans_y_param;
  rclcpp::Parameter trans_z_param;


  /// If transform parameters exist in the param file use them
  if (this->get_parameter("static_transformer.quaternion.x", quat_x_param) &&
    this->get_parameter("static_transformer.quaternion.y", quat_y_param) &&
    this->get_parameter("static_transformer.quaternion.z", quat_z_param) &&
    this->get_parameter("static_transformer.quaternion.w", quat_w_param) &&
    this->get_parameter("static_transformer.translation.x", trans_x_param) &&
    this->get_parameter("static_transformer.translation.y", trans_y_param) &&
    this->get_parameter("static_transformer.translation.z", trans_z_param))
  {
    RCLCPP_WARN(get_logger(), "Using transform from file.");
    m_static_transformer = std::make_unique<StaticTransformer>(
      get_transform(
        m_input_frame_id, m_output_frame_id,
        quat_x_param.as_double(),
        quat_y_param.as_double(),
        quat_z_param.as_double(),
        quat_w_param.as_double(),
        trans_x_param.as_double(),
        trans_y_param.as_double(),
        trans_z_param.as_double()).transform);
  } else {  /// Else lookup transform being published on /tf or /static_tf topics
    /// TF buffer
    tf2_ros::Buffer tf2_buffer(this->get_clock());
    /// TF listener
    tf2_ros::TransformListener tf2_listener(tf2_buffer);
    while (rclcpp::ok()) {
      try {
        RCLCPP_INFO(get_logger(), "Looking up the transform.");
        m_static_transformer = std::make_unique<StaticTransformer>(
          tf2_buffer.lookupTransform(
            m_output_frame_id, m_input_frame_id,
            tf2::TimePointZero).transform);
        break;
      } catch (const std::exception & transform_exception) {
        RCLCPP_INFO(get_logger(), "No transform was available. Retrying after 100 ms.");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        continue;
      }
    }
  }
  
  using autoware::common::types::PointXYZI;
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI>{
    m_filtered_transformed_msg, m_output_frame_id}.resize(m_pcl_size);
}

const PointCloud2 & PointCloud2FilterTransformNode::filter_and_transform(const PointCloud2 & msg)
{
  // Verify frame_id
  if (msg.header.frame_id != m_input_frame_id) {
    throw std::runtime_error(
            "Raw topic from unexpected frame. Expected: " +
            m_input_frame_id + ", got: " + msg.header.frame_id);
  }

  sensor_msgs::PointCloud2ConstIterator<float32_t> x_it(msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float32_t> y_it(msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float32_t> z_it(msg, "z");

  auto && intensity_it = autoware::common::lidar_utils::IntensityIteratorWrapper(msg);

  using autoware::common::types::PointXYZI;
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI> modifier{m_filtered_transformed_msg};
  modifier.clear();
  modifier.reserve(m_pcl_size);

  m_filtered_transformed_msg.header.stamp = msg.header.stamp;

  for (size_t it = 0; it < (msg.data.size() / 16); it++) {
    PointXYZI pt;
    pt.x = *x_it;
    pt.y = *y_it;
    pt.z = *z_it;
    intensity_it.get_current_value(pt.intensity);

    if (point_not_filtered(pt)) {
      auto transformed_point = transform_point(pt);
      transformed_point.intensity = pt.intensity;
      modifier.push_back(transformed_point);
    }

    ++x_it;
    ++y_it;
    ++z_it;
    intensity_it.next();

    if (intensity_it.eof()) {
      break;
    }
  }
  return m_filtered_transformed_msg;
}

void
PointCloud2FilterTransformNode::process_filtered_transformed_message(
  const PointCloud2::SharedPtr msg)
{
/* FHA (Operates at wrong time early - late)
    
		1) Give the callback a timestamp that specify when this callback is called.
		2) Get the difference between the callback timestamp and the latest message timestamp.
		3) Compare this difference with a minimum and maximum values.

		pseudocode:
		#define MAX maximum_delay
		#define MIN minimum_delay
		callback_stamp = node->now();
		difference = callback_stamp - latest_stamp
		if(difference > MAX) { operates at wrong time late}
		if(difference < MIN) { operates at wrong time early}
		
end FHA */

/* FHA (Recieve Error Data - Simple-Code)
Note: some of this code is already handled 
Steps:
	1-The first check ensures that the LiDAR message is not empty. If it's empty, it prints a message indicating that the LiDAR data is empty and returns early
	2-The number of points in the LiDAR message is printed by calculating the size of the data array divided by the size of a single point (16 bytes in this case)
	3-The frame ID of the LiDAR message is printed to verify if it matches the expected frame ID.
	4-A sample point is extracted from the LiDAR message by accessing the "x", "y", and "z" fields. The coordinates of the sample point are printed to verify that the data is being parsed correctly
  // Check if the LiDAR message contains data
  if (msg->data.empty())
  {
    std::cout << "LiDAR data is empty!" << std::endl;
    return;
  }

  // Print the number of points in the LiDAR message
  std::cout << "Number of points in the LiDAR message: " << (msg->data.size() / 16) << std::endl;

  // Print the frame ID of the LiDAR message
  std::cout << "LiDAR message frame ID: " << msg->header.frame_id << std::endl;

  // Print a sample point from the LiDAR message
  float32_t x = 0.0f, y = 0.0f, z = 0.0f;
  sensor_msgs::PointCloud2ConstIterator<float32_t> x_it(*msg, "x");
  sensor_msgs::PointCloud2ConstIterator<float32_t> y_it(*msg, "y");
  sensor_msgs::PointCloud2ConstIterator<float32_t> z_it(*msg, "z");
  if (x_it != x_it.end() && y_it != y_it.end() && z_it != z_it.end())
  {
    x = *x_it;
    y = *y_it;
    z = *z_it;
  }
  std::cout << "Sample point coordinates: x=" << x << ", y=" << y << ", z=" << z << std::endl;
*/
  const auto filtered_transformed_msg = filter_and_transform(*msg);
  m_pub_ptr->publish(filtered_transformed_msg);
}

/*
FHA (Sending error data - Pesudocode)
	1-Check for the Data before publish it using (try-catch)
	2-if there are any errors (Stop and publish an error message)
	
	void PointCloud2FilterTransformNode::process_filtered_transformed_message(
	  const PointCloud2::SharedPtr msg)
	{
	  try {
		const auto filtered_transformed_msg = filter_and_transform(*msg);
		m_pub_ptr->publish(filtered_transformed_msg);
	  } catch (const std::exception& e) {
		// Handle the error
		RCLCPP_ERROR(get_logger(), "Error processing point cloud: %s", e.what());
	  
		// Create and publish an error message
		std::string error_message = "Error processing point cloud: " + std::string(e.what());
		std_msgs::msg::String error_msg;
		error_msg.data = error_message;
		m_error_pub_ptr->publish(error_msg);
	  }
	}
	
*/
}  // namespace point_cloud_filter_transform_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::perception::filters::point_cloud_filter_transform_nodes::PointCloud2FilterTransformNode)