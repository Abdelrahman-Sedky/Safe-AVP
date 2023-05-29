// Copyright 2019-2021 the Autoware Foundation
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

#include <common/types.hpp>
#include <point_cloud_fusion_nodes/point_cloud_fusion_node.hpp>
#include <point_cloud_msg_wrapper/point_cloud_msg_wrapper.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

using autoware::common::types::bool8_t;
using autoware::common::types::float32_t;

namespace autoware
{
namespace perception
{
namespace filters
{
namespace point_cloud_fusion_nodes
{

PointCloudFusionNode::PointCloudFusionNode(
  const rclcpp::NodeOptions & node_options)
: Node("point_cloud_fusion_nodes", node_options),
  m_cloud_publisher(create_publisher<PointCloudMsgT>("output_topic", rclcpp::QoS(10))),
  m_input_topics(static_cast<std::size_t>(declare_parameter("number_of_sources").get<int>())),
  m_output_frame_id(declare_parameter("output_frame_id").get<std::string>()),
  m_cloud_capacity(static_cast<uint32_t>(declare_parameter("cloud_size").get<int>()))
{
	// Hazard 3: Operates inadvertently
	// Issue Description: The node may operate inadvertently due to unexpected triggering of the callback function.
	// How to Discover it: Monitor the node's behavior and output for unexpected or unintended operations.
	// Mitigation Technique: Implement appropriate checks and conditions to ensure the node operates only when intended.

  for (size_t i = 0; i < m_input_topics.size(); ++i) {
    m_input_topics[i] = "input_topic" + std::to_string(i + 1);
  }
  init();
}

void PointCloudFusionNode::init()
{
  m_core = std::make_unique<point_cloud_fusion::PointCloudFusion>(
    m_cloud_capacity,
    m_input_topics.size());

  using autoware::common::types::PointXYZI;
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI>{
    m_cloud_concatenated, m_output_frame_id}.reserve(m_cloud_capacity);

  if (m_input_topics.size() > 8 || m_input_topics.size() < 2) {
	  
    // Hazard 9: Conflicting data or information
    // Issue Description: The number of input topics for point cloud fusion is outside the valid range, leading to conflicting or inconsistent data.
    // How to Discover it: Analyze the input configuration and check if the number of input topics is within the valid range.
    // Mitigation Technique: Implement proper validation checks during node initialization to ensure the number of input topics is within the valid range. Provide informative error messages if the configuration is invalid.

    throw std::domain_error(
            "Number of sources for point cloud fusion must be between 2 and 8."
            " Found: " + std::to_string(m_input_topics.size()));
  }

  for (size_t i = 0; i < 8; ++i) {
    if (i < m_input_topics.size()) {
		
		// Hazard 4: Operates at the wrong time (early)
		// Issue Description: The node may start processing the point clouds before it receives all the necessary input messages.
		// How to Discover it: Monitor the node's behavior and output for early fusion operations.
		// Mitigation Technique: Implement synchronization mechanisms to ensure all input messages are received before processing.
		
      m_cloud_subscribers[i] = std::make_unique<message_filters::Subscriber<PointCloudMsgT>>(
        this, m_input_topics[i]);
    } else {
		
		// Hazard 5: Operates at the wrong time (late)
		// Issue Description: The node may receive outdated input messages, causing delayed processing and fusion.
		// How to Discover it: Monitor the node's behavior and output for delayed fusion operations.
		// Mitigation Technique: Implement mechanisms to handle late-arriving input messages and prevent delayed processing.

      m_cloud_subscribers[i] = std::make_unique<message_filters::Subscriber<PointCloudMsgT>>(
        this, m_input_topics[0]);
    }
  }
  m_cloud_synchronizer = std::make_unique<message_filters::Synchronizer<SyncPolicyT>>(
    SyncPolicyT(10), *m_cloud_subscribers[0], *m_cloud_subscribers[1], *m_cloud_subscribers[2],
    *m_cloud_subscribers[3], *m_cloud_subscribers[4], *m_cloud_subscribers[5],
    *m_cloud_subscribers[6], *m_cloud_subscribers[7]);

  m_cloud_synchronizer->registerCallback(
    std::bind(
      &PointCloudFusionNode::pointcloud_callback, this, std::placeholders::_1,
      std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5,
      std::placeholders::_6, std::placeholders::_7, std::placeholders::_8));
	  
	// Hazard 1: Fails to operate
	// Issue Description: The node fails to initialize properly due to missing or incorrect configuration parameters.
	// How to Discover it: Insufficient or incorrect configuration input during node initialization.
	// Mitigation Technique: Thoroughly validate and verify the configuration parameters before initializing the node. Implement proper error handling and informative error messages to assist in identifying configuration issues.
}

std::chrono::nanoseconds PointCloudFusionNode::convert_msg_time(builtin_interfaces::msg::Time stamp)
{
  return std::chrono::seconds(stamp.sec) + std::chrono::nanoseconds(stamp.nanosec);
}

void
PointCloudFusionNode::pointcloud_callback(
  const PointCloudMsgT::ConstSharedPtr & msg1, const PointCloudMsgT::ConstSharedPtr & msg2,
  const PointCloudMsgT::ConstSharedPtr & msg3, const PointCloudMsgT::ConstSharedPtr & msg4,
  const PointCloudMsgT::ConstSharedPtr & msg5, const PointCloudMsgT::ConstSharedPtr & msg6,
  const PointCloudMsgT::ConstSharedPtr & msg7, const PointCloudMsgT::ConstSharedPtr & msg8)
{
	
	// Hazard 7: Receives erroneous data
	// Issue Description: The node receives erroneous or invalid point cloud data from one or more input topics.
	// How to Discover it: Analyze the received point cloud data and check for inconsistencies or unexpected values.
	// Mitigation Technique: Implement data validation mechanisms to check the integrity and validity of received point cloud data. Discard or handle erroneous data appropriately to prevent propagating errors throughout the fusion process.

  std::array<PointCloudMsgT::ConstSharedPtr, 8> msgs{msg1, msg2, msg3, msg4, msg5, msg6, msg7,
    msg8};
	
	// Hazard 6: Unable to stop operation
	// Issue Description: The node is unable to stop the fusion operation in case of unexpected situations or errors.
	// How to Discover it: Trigger a stop command or interrupt signal while the node is processing and check if it responds appropriately.
	// Mitigation Technique: Implement mechanisms to detect stop commands or interrupt signals and handle them properly. Gracefully stop the fusion process and perform necessary cleanup tasks to ensure the node can be safely stopped.

  // reset pointcloud before using
  using autoware::common::types::PointXYZI;
  point_cloud_msg_wrapper::PointCloud2Modifier<PointXYZI> modifier{m_cloud_concatenated};
  modifier.clear();
  modifier.reserve(m_cloud_capacity);

  auto latest_stamp = msgs[0]->header.stamp;
  auto total_size = 0U;

	// Hazard 2: Operates incorrectly/erroneously
	// Issue Description: The node incorrectly combines or fuses point cloud data, resulting in inaccurate or unreliable fused point clouds.
	// How to Discover it: Analyze the output fused point clouds and compare them with the expected results or ground truth data.
	// Mitigation Technique: Implement robust fusion algorithms and validation mechanisms to ensure accurate fusion of point cloud data. Perform extensive testing and validation against known scenarios and datasets to verify the correctness of the fusion process.
  
  // Get the latest time stamp of the point clouds and find the total size after concatenation
  for (uint32_t msg_idx = 0; msg_idx < m_input_topics.size(); ++msg_idx) {
    const auto & stamp = msgs[msg_idx]->header.stamp;
    if (convert_msg_time(stamp) > convert_msg_time(latest_stamp)) {
      latest_stamp = stamp;
    }
    total_size += msgs[msg_idx]->width;
  }

  if (total_size > m_cloud_capacity) {
    RCLCPP_WARN(
      get_logger(), "pointclouds that are trying to be fused exceed the cloud capacity. "
      "The exceeded clouds will be ignored.");
  }

  // Go through all the messages and fuse them.
  uint32_t fused_cloud_size = 0;
  try {
    fused_cloud_size = m_core->fuse_pc_msgs(msgs, m_cloud_concatenated);
  } catch (point_cloud_fusion::PointCloudFusion::Error fuse_error) {
    if (fuse_error == point_cloud_fusion::PointCloudFusion::Error::TOO_LARGE) {
      RCLCPP_WARN(get_logger(), "Pointcloud is too large to be fused and will be ignored.");
    } else if (fuse_error == point_cloud_fusion::PointCloudFusion::Error::INSERT_FAILED) {
      RCLCPP_ERROR(get_logger(), "Points could not be added correctly to the fused cloud.");
    } else {
      RCLCPP_ERROR(get_logger(), "Unknown error.");
    }
  }

  if (fused_cloud_size > 0) {
    // Resize and publish.
    modifier.resize(fused_cloud_size);

    m_cloud_concatenated.header.stamp = latest_stamp;
	
	// Hazard 8: Sends erroneous data
	// Issue Description: The node sends fused point cloud data that contains errors or inconsistencies.
	// How to Discover it: Analyze the published fused point cloud data and check for inconsistencies or unexpected values.
	// Mitigation Technique: Implement data validation mechanisms to ensure the integrity and validity of the fused point cloud data before publishing. Perform thorough testing and validation to ensure the correctness of the fused point cloud output.

    m_cloud_publisher->publish(m_cloud_concatenated);
  }
}
}  // namespace point_cloud_fusion_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::perception::filters::point_cloud_fusion_nodes::PointCloudFusionNode)
