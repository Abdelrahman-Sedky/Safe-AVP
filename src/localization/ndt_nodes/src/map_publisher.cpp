

#include <common/types.hpp>
#include <ndt_nodes/map_publisher.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <memory>
#include <string>

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;
using autoware::common::types::float64_t;

namespace autoware
{
namespace localization
{
namespace ndt_nodes
{
/// Clear the given pointcloud message
void reset_pc_msg(sensor_msgs::msg::PointCloud2 & msg)
{
  using autoware::common::types::PointXYZI;
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI>{msg}.clear();
}

NDTMapPublisherNode::NDTMapPublisherNode(
  const rclcpp::NodeOptions & node_options
)
: Node("ndt_map_publisher_node", node_options),
  m_pcl_file_name(declare_parameter("map_pcd_file").get<std::string>()),
  m_yaml_file_name(declare_parameter("map_yaml_file").get<std::string>()),
  m_viz_map(declare_parameter("viz_map", false))
{
	/********************************FHA***********************************************
	/*	Hazard Category: (7)Receives erroneous data:
	/*	Failure Type: Data failure.
	/*	How to Discover: Monitor the system for any error messages, exceptions, or inconsistencies related to the loading or processing of map PCD files and point cloud data.
	/*	Mitigation Technique:
	/*	Implement data validation mechanisms to check the integrity, format, and compatibility of the received map PCD files and point cloud data.
	/*	Use error handling and logging to report and handle any data-related errors or inconsistencies.
	/*	Conduct extensive testing using a variety of valid and invalid data inputs to ensure the system handles erroneous data appropriately.
	/*	Implement fallback or alternative data sources to mitigate the impact of receiving erroneous data.
	***********************************************************************************/

  using PointXYZ = perception::filters::voxel_grid::PointXYZ;
  PointXYZ min_point;
  min_point.x =
    static_cast<float32_t>(declare_parameter("map_config.min_point.x").get<float32_t>());
  min_point.y =
    static_cast<float32_t>(declare_parameter("map_config.min_point.y").get<float32_t>());
  min_point.z =
    static_cast<float32_t>(declare_parameter("map_config.min_point.z").get<float32_t>());
  PointXYZ max_point;
  max_point.x =
    static_cast<float32_t>(declare_parameter("map_config.max_point.x").get<float32_t>());
  max_point.y =
    static_cast<float32_t>(declare_parameter("map_config.max_point.y").get<float32_t>());
  max_point.z =
    static_cast<float32_t>(declare_parameter("map_config.max_point.z").get<float32_t>());
  PointXYZ voxel_size;
  voxel_size.x =
    static_cast<float32_t>(declare_parameter("map_config.voxel_size.x").get<float32_t>());
  voxel_size.y =
    static_cast<float32_t>(declare_parameter("map_config.voxel_size.y").get<float32_t>());
  voxel_size.z =
    static_cast<float32_t>(declare_parameter("map_config.voxel_size.z").get<float32_t>());
  const std::size_t capacity =
    static_cast<std::size_t>(declare_parameter("map_config.capacity").get<std::size_t>());
  const std::string map_frame = declare_parameter("map_frame").get<std::string>();
  const std::string map_topic = "ndt_map";
  const std::string viz_map_topic = "viz_ndt_map";

  m_map_config_ptr = std::make_unique<MapConfig>(min_point, max_point, voxel_size, capacity);
  init(map_frame, map_topic, viz_map_topic);
  run();
}

void NDTMapPublisherNode::init(
  const std::string & map_frame,
  const std::string & map_topic,
  const std::string & viz_map_topic)
{
	/********************************FHA***********************************************
	/*	Hazard Category: (9)Conflicting data or information:
	/*	Failure Type: Data failure.
	/*	How to Discover: Compare the map configuration parameters with the actual map data for any inconsistencies or conflicts.
	/*	Mitigation Technique:
	/*	Implement proper validation and synchronization mechanisms to ensure the coherence of the map configuration parameters and the actual map data.
	/*	Conduct rigorous testing to verify that the map configuration parameters align with the loaded map data.
	/*	Implement appropriate error handling and reporting mechanisms to identify and address any conflicting data or information.
	/*	Use logging and monitoring to detect and investigate any inconsistencies between the map configuration and the loaded map data.
	***********************************************************************************/

  m_ndt_map_ptr = std::make_unique<ndt::DynamicNDTMap>(*m_map_config_ptr);

  using autoware::common::types::PointXYZI;
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI> initializer{
    m_source_pc, map_frame};

  m_pub = create_publisher<sensor_msgs::msg::PointCloud2>(
    map_topic,
    rclcpp::QoS(rclcpp::KeepLast(5U)).transient_local());

  if (m_viz_map) {   // create a publisher for map_visualization
    using PointXYZ = perception::filters::voxel_grid::PointXYZ;

    m_viz_pub = create_publisher<sensor_msgs::msg::PointCloud2>(
      viz_map_topic, rclcpp::QoS(rclcpp::KeepLast(5U)).transient_local());

    // TODO(jwhitleywork) Hard-coded voxel size and capacity.
    // To be removed when #380 is in.
    PointXYZ viz_voxel_size;
    viz_voxel_size.x = 0.4F;
    viz_voxel_size.y = 0.4F;
    viz_voxel_size.z = 0.4F;
    m_viz_map_config_ptr = std::make_unique<MapConfig>(
      m_map_config_ptr->get_min_point(),
      m_map_config_ptr->get_max_point(),
      viz_voxel_size,
      10000000U);

    // Initialize Voxel Grid and output message for downsampling map
    using autoware::common::types::PointXYZI;
    point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI> downsampled_pc_initializer{
      m_downsampled_pc, map_frame};
    m_voxelgrid_ptr = std::make_unique<VoxelGrid>(*m_viz_map_config_ptr);

    // Periodic publishing is a temp. hack until the rviz in ade has transient_local qos support.
    // TODO(yunus.caliskan): Remove the loop and publish only once after #380
    m_visualization_timer = create_wall_timer(
      std::chrono::seconds(1),
      [this]() {
        if (m_downsampled_pc.width > 0U) {
          m_viz_pub->publish(m_downsampled_pc);
        }
      });
  }

  m_pub_earth_map = create_publisher<tf2_msgs::msg::TFMessage>(
    "/tf_static",
    rclcpp::QoS(rclcpp::KeepLast(5U)).transient_local());
}

void NDTMapPublisherNode::run()
{
	
	/********************************FHA***********************************************
	/*	Hazard Category: (1)Fails to operate
	/*	Failure Type: Functional failure.
	/*	How to Discover: Monitor the system for any errors or exceptions during map loading, initialization, insertion, serialization, or publishing.
	/*	Mitigation Technique:
	/*	Implement proper error handling and logging to identify and report failures during map loading, initialization, and other critical operations.
	/*	Perform thorough testing and validation of map loading and initialization processes to ensure they function correctly.
	/*	Implement robust error recovery mechanisms to handle failures gracefully and provide appropriate feedback to the system operator.
	*************************************************************************************/
	
  ndt::geocentric_pose_t pose = ndt::load_map(m_yaml_file_name, m_pcl_file_name, m_source_pc);
  publish_earth_to_map_transform(pose);
  m_ndt_map_ptr->insert(m_source_pc);
  m_ndt_map_ptr->serialize_as<SerializedMap>(m_map_pc);

  if (m_viz_map) {
    reset_pc_msg(m_downsampled_pc);
    downsample_pc();
  }
  publish();
}

void NDTMapPublisherNode::publish_earth_to_map_transform(ndt::geocentric_pose_t pose)
{
  geometry_msgs::msg::TransformStamped tf;

  tf.transform.translation.x = pose.x;
  tf.transform.translation.y = pose.y;
  tf.transform.translation.z = pose.z;

  tf2::Quaternion q;
  q.setRPY(pose.roll, pose.pitch, pose.yaw);
  tf.header.stamp = rclcpp::Clock().now();
  tf.transform.rotation = tf2::toMsg(q);
  tf.header.frame_id = "earth";
  tf.child_frame_id = "map";

  tf2_msgs::msg::TFMessage static_tf_msg;
  static_tf_msg.transforms.push_back(tf);

	/********************************FHA***********************************************
	/*	Hazard Category: (8)Sends erroneous data:
	/*	Failure Type: Data failure.
	/*	How to Discover: Monitor the published map, down-sampled point cloud, or visualization for any inconsistencies, incorrect formats, or unexpected behavior.
	/*	Mitigation Technique:
	/*	Implement proper data validation and sanity checks before publishing the map, down-sampled point cloud, or visualization.
	/*	Conduct thorough testing to verify the correctness and integrity of the published data.
	/*	Use logging and monitoring to detect any inconsistencies or errors in the published data.
	/*	Implement error recovery mechanisms to handle and rectify any erroneous data publication.
	**************************************************************************************/
	
	/********************************FHA***********************************************
	/*	Hazard Category: (5),(6)Operates at the wrong time (late or early):
	/*	Failure Type: Timing failure.
	/*	How to Discover: Monitor the system behavior and timestamps of map publication to identify any occurrences of delayed operations.
	/*	Mitigation Technique:
	/*	Analyze the system's timing requirements and adjust the timing mechanisms accordingly to prevent delays.
	/*	Conduct extensive testing to validate the timing accuracy of the system under various conditions.
	/*	Implement appropriate timeout mechanisms or triggers to prevent excessive delays in map publication.
	/*	Implement sanity checks and validation to prevent premature map publication.
	**************************************************************************************/
	
  m_transform_pub_timer = create_wall_timer(
    std::chrono::seconds(1),
    [this, static_tf_msg]() {
      m_pub_earth_map->publish(static_tf_msg);
    });

  m_pub_earth_map->publish(static_tf_msg);
}

void NDTMapPublisherNode::downsample_pc()
{
  try {
    m_voxelgrid_ptr->insert(m_source_pc);
    m_downsampled_pc = m_voxelgrid_ptr->get();
  } catch (const std::exception & e) {
    std::string err_msg{e.what()};
    RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
  } catch (...) {
    std::string err_msg{"Unknown error occurred in "};
    err_msg += get_name();
    RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
    throw;
  }
}

void NDTMapPublisherNode::publish()
{
  if (m_map_pc.width > 0U) {
    m_pub->publish(m_map_pc);
  }
}

void NDTMapPublisherNode::reset()
{
  reset_pc_msg(m_map_pc);
  reset_pc_msg(m_source_pc);

  if (m_viz_map) {
    reset_pc_msg(m_downsampled_pc);
  }

  m_ndt_map_ptr->clear();
}

}  // namespace ndt_nodes
}  // namespace localization
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(autoware::localization::ndt_nodes::NDTMapPublisherNode)


// Copyright 2019-2020 the Autoware Foundation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.