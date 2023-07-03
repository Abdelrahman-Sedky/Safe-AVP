// Copyright 2019 the Autoware Foundation
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


//lint -e537 NOLINT cpplint wants this due to std::make_unique
#include <rclcpp/node_options.hpp>
#include <voxel_grid_nodes/voxel_cloud_node.hpp>
#include <voxel_grid_nodes/algorithm/voxel_cloud_approximate.hpp>
#include <voxel_grid_nodes/algorithm/voxel_cloud_centroid.hpp>
#include <common/types.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <memory>
#include <string>
#include <algorithm>

using autoware::common::types::bool8_t;
using autoware::common::types::uchar8_t;
using autoware::common::types::float32_t;

namespace autoware
{
namespace perception
{
namespace filters
{
namespace voxel_grid_nodes
{
////////////////////////////////////////////////////////////////////////////////
VoxelCloudNode::VoxelCloudNode(
  const rclcpp::NodeOptions & node_options)
: Node("voxel_grid_cloud_node", node_options),
  m_sub_ptr{create_subscription<Message>(
      "points_in",
      rclcpp::QoS(
        static_cast<size_t>(static_cast<int>(declare_parameter("subscription.qos.history_depth",
        10)))
      ).durability(
        parse_durability_parameter(
          declare_parameter("subscription.qos.durability", "volatile")
        )
      )
      ,
      std::bind(&VoxelCloudNode::callback, this, std::placeholders::_1)
    )},
  m_pub_ptr{create_publisher<Message>(
      "points_downsampled",
      rclcpp::QoS(
        static_cast<size_t>(static_cast<int>(declare_parameter("publisher.qos.history_depth", 10)))
      ).durability(
        parse_durability_parameter(
          declare_parameter("publisher.qos.durability", "volatile")
        )
      )
    )},
  m_has_failed{false}
{
  // Build config manually (messages only have default constructors)
  voxel_grid::PointXYZ min_point;
  min_point.x = static_cast<float32_t>(declare_parameter("config.min_point.x").get<float32_t>());
  min_point.y = static_cast<float32_t>(declare_parameter("config.min_point.y").get<float32_t>());
  min_point.z = static_cast<float32_t>(declare_parameter("config.min_point.z").get<float32_t>());
  voxel_grid::PointXYZ max_point;
  max_point.x = static_cast<float32_t>(declare_parameter("config.max_point.x").get<float32_t>());
  max_point.y = static_cast<float32_t>(declare_parameter("config.max_point.y").get<float32_t>());
  max_point.z = static_cast<float32_t>(declare_parameter("config.max_point.z").get<float32_t>());
  voxel_grid::PointXYZ voxel_size;
  voxel_size.x = static_cast<float32_t>(declare_parameter("config.voxel_size.x").get<float32_t>());
  voxel_size.y = static_cast<float32_t>(declare_parameter("config.voxel_size.y").get<float32_t>());
  voxel_size.z = static_cast<float32_t>(declare_parameter("config.voxel_size.z").get<float32_t>());
  const std::size_t capacity =
    static_cast<std::size_t>(declare_parameter("config.capacity").get<std::size_t>());
  const voxel_grid::Config cfg{min_point, max_point, voxel_size, capacity};
  // Init
  init(cfg, declare_parameter("is_approximate").get<bool8_t>());
}

////////////////////////////////////////////////////////////////////////////////
void VoxelCloudNode::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
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
          1)Validate the input data before processing it.
		     if (!isValidData(msg))
              {
                 // Handle invalid data
                      return;
              }
          2)Implement data filtering or outlier removal techniques to mitigate the impact of erroneous data.
		       preprocessData(msg);
			   try {
        // Processing logic here

        // Publish the processed data
        m_pub_ptr->publish(m_voxelgrid_ptr->get());
                    }
    catch (const std::exception& e) {
        // Handle exceptions
        // Log or handle the error accordingly
                     }
    catch (...) {
        // Handle unknown exceptions
        // Log or handle the error accordingly
        throw;  // Rethrow the exception if needed
                }
 }
        bool isValidData(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
       {
    // Implement data validation logic here
    // Check if the input data meets the defined criteria for validity
    // Return true if the data is valid, false otherwise
	   if (msg->fields.size() < 3) {
        return false; // Invalid data if there are fewer than 3 fields (e.g., x, y, z)
        }

    // Add more validity checks based on your requirements

    return true; // Data is considered valid
       }
	   
     void preprocessData(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
       {
    // Apply data filtering or outlier removal techniques to preprocess the data
    // For example, you can use statistical outlier removal algorithms, voxel filters, or other methods.

    // Apply your chosen filtering technique to remove outliers or filter the data as required
    // Store the preprocessed data for further processing
        }
*/ 

/*
FHA (Sending error data - Pesudocode)
       1- Validate the output before sending
	    try {
        // Step 1: Perform the processing logic to generate the output
        sensor_msgs::msg::PointCloud2 outputData = processInputData();

        // Step 2: Validate the output data
        if (isValidData(outputData))
        {
            // Step 3: Publish/send the output data
            m_pub_ptr->publish(outputData);
        }
        else
        {
            // Handle invalid output data
        }
    }
    catch (const std::exception& e) {
        // Handle exceptions
        // Log or handle the error accordingly
    }
    catch (...) {
        // Handle unknown exceptions
        // Log or handle the error accordingly
        throw;  // Rethrow the exception if needed
    }
     
	 sensor_msgs::msg::PointCloud2 processInputData()
     {
    // Implement the processing logic to generate the output data
    // Perform the necessary operations on the input data and generate the output
    // Return the generated output data
     }
*/
  try {
    m_voxelgrid_ptr->insert(*msg);
    m_pub_ptr->publish(m_voxelgrid_ptr->get());
  } catch (const std::exception & e) {
    std::string err_msg{get_name()};
    err_msg += ": " + std::string(e.what());
    RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
    m_has_failed = true;
  } catch (...) {
    std::string err_msg{"Unknown error occurred in "};
    err_msg += get_name();
    RCLCPP_ERROR(this->get_logger(), err_msg.c_str());
    throw;
  }
}
////////////////////////////////////////////////////////////////////////////////
void VoxelCloudNode::init(const voxel_grid::Config & cfg, const bool8_t is_approximate)
{
  // construct voxel grid
  if (is_approximate) {
    m_voxelgrid_ptr = std::make_unique<algorithm::VoxelCloudApproximate>(cfg);
  } else {
    m_voxelgrid_ptr = std::make_unique<algorithm::VoxelCloudCentroid>(cfg);
  }
}

/////////////////////////////////////////////////////////////////////////////
rmw_qos_durability_policy_t parse_durability_parameter(
  const std::string & durability)
{
  if (durability == "transient_local") {
    return RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL;
  } else if (durability == "volatile") {
    return RMW_QOS_POLICY_DURABILITY_VOLATILE;
  }

  throw std::runtime_error(
          "Durability setting '" + durability + "' is not supported."
          "Please try 'volatile' or 'transient_local'.");
}
}  // namespace voxel_grid_nodes
}  // namespace filters
}  // namespace perception
}  // namespace autoware

RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::perception::filters::voxel_grid_nodes::VoxelCloudNode)
