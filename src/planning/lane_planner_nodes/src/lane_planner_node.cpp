// Copyright 2020 The Autoware Foundation
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

#include "lane_planner_nodes/lane_planner_node.hpp"
#include <had_map_utils/had_map_conversion.hpp>

#include <string>
#include <memory>

namespace autoware
{
namespace lane_planner_nodes
{

using motion::planning::trajectory_smoother::TrajectorySmootherConfig;

LanePlannerNode::LanePlannerNode(const rclcpp::NodeOptions & node_options)
: TrajectoryPlannerNodeBase{"lane_planner", "plan_lane_trajectory", node_options}
{
	/*****************************************************************************************/
	/*	Hazard: (3) Operates inadvertently                                                   */
	/*	                                                                                     */
	/*	Issue Description: The LanePlannerNode may inadvertently operate when it is not 
		intended to, leading to unintended behavior or consequences.				
	
	/*	Mitigation Technique: Ensure proper control and activation mechanisms are in place to
		prevent inadvertent operation of the LanePlannerNode, such as implementing safety checks
		and validation before executing critical operations.
	/*****************************************************************************************/	
	
  const VehicleConfig vehicle_param{
    static_cast<Real>(declare_parameter("vehicle.cg_to_front_m").get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.cg_to_rear_m").get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.front_corner_stiffness").get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.rear_corner_stiffness").get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.mass_kg").get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.yaw_inertia_kgm2").get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.width_m").get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.front_overhang_m").get<float32_t>()),
    static_cast<Real>(declare_parameter("vehicle.rear_overhang_m").get<float32_t>())
  };
  const TrajectorySmootherConfig config{
    static_cast<float32_t>(
      declare_parameter("gaussian_smoother.standard_deviation").get<float64_t>()),
    static_cast<uint32_t>(declare_parameter("gaussian_smoother.kernel_size").get<uint64_t>())
  };
  const lane_planner::LanePlannerConfig planner_config{
    static_cast<float32_t>(
      declare_parameter("lane_planner.trajectory_resolution").get<float64_t>())
  };
  
	/*****************************************************************************************/
	/*	Hazard: (1) Fails to operate                                                         */
	/*	                                                                                     */
	/*	Issue Description: The constructor of LanePlannerNode may fail to operate if there is 
		an error in initializing the base TrajectoryPlannerNodeBase.					
	
	/*	Mitigation Technique: Ensure that the initialization of the base class 
		TrajectoryPlannerNodeBase is successful before proceeding with the constructor of
		LanePlannerNode. Perform proper error handling and logging to indicate any failures 
		during the initialization process. Implement fallback mechanisms or alternative 
		initialization paths to handle failures gracefully.  
	/*****************************************************************************************/	
  
  m_planner = std::make_unique<lane_planner::LanePlanner>(vehicle_param, config, planner_config);
  
  /*****************************************************************************************/
  /* Hazard: (6) Unable to stop operation                                                  */
  /*                                                                                       */
  /* Issue Description: The LanePlannerNode does not provide a mechanism to stop the        */
  /* operation once it has started planning. This may lead to difficulties in controlling   */
  /* the node's behavior or terminating its execution when needed.                          */
  /*                                                                                       */
  /* Mitigation Technique: Implement a mechanism to stop the planning operation when        */
  /* required. This could be achieved by adding a stop_planning() function that sets a      */
  /* flag or state variable indicating the node to stop planning and checking this flag     */
  /* during the execution of planning operations.                                          */
  /*****************************************************************************************/
  
}

HADMapService::Request LanePlannerNode::create_map_request(const HADMapRoute & had_map_route)
{
  (void) had_map_route;
  
	/*****************************************************************************************/
	/*	Hazard: (7) Receives erroneous data.                                                 */
	/*	                                                                                     */
	/*	Issue Description: The current implementation of create_map_request does not properly 
		handle the received HADMapRoute data and does not use it to form the request. This 
		could lead to erroneous data being sent to the map service or incorrect map data being
		retrieved. 
			
	/*	Mitigation Technique: Update the create_map_request function to properly utilize the 
		received HADMapRoute data and construct a valid request for the map service. Perform 
		sanity checks and validation on the received data to ensure its correctness before 
		proceeding with the request. Handle any potential errors or exceptions that may occur 
		during the data processing and request formation.  
	/*****************************************************************************************/

  // TODO(mitsudome-r): replace it with bounded request
  HADMapService::Request request;
  request.requested_primitives.push_back(HADMapService::Request::FULL_MAP);
  // const auto goal = goal_handle->get_goal()->sub_route.goal_point;
  // const auto start = goal_handle->get_goal()->sub_route.start_point;
  // request->geom_lower_bound[0] = std::min(start.x, goal.x);
  // request->geom_lower_bound[1] = std::min(start.y, goal.y);
  // request->geom_upper_bound[0] = std::max(start.x, goal.x);
  // request->geom_upper_bound[1] = std::max(start.y, goal.y);
  
  /*****************************************************************************************/
  /* Hazard: (8) Sends erroneous data                                                      */
  /*                                                                                       */
  /* Issue Description: The create_map_request function currently does not perform any      */
  /* validation or error checking on the received HADMapRoute data before constructing the  */
  /* request. This may result in sending erroneous or invalid data to the map service,      */
  /* leading to incorrect map data retrieval or processing.                                 */
  /*                                                                                       */
  /* Mitigation Technique: Perform thorough validation and sanity checks on the received    */
  /* HADMapRoute data before constructing the request. Ensure that the data is in the       */
  /* expected format and complies with the requirements of the map service. Handle any      */
  /* errors or exceptions that may occur during the validation process and provide          */
  /* appropriate error handling or fallback mechanisms.                                    */
  /*****************************************************************************************/

  return request;
}

Trajectory LanePlannerNode::plan_trajectory(
  const HADMapRoute & had_map_route,
  const lanelet::LaneletMapPtr & lanelet_map_ptr)
{
	/*****************************************************************************************/
	/*	Hazard: (2) Operates incorrectly/erroneously                                         */
	/*	                                                                                     */
	/*	Issue Description: The current implementation of plan_trajectory does not perform any
		trajectory planning or utilize the received HADMapRoute and lanelet_map_ptr to 
		generate a valid trajectory. It operates incorrectly or erroneously by returning an 
		empty trajectory.
	
	/*	Mitigation Technique: Implement the trajectory planning logic in the plan_trajectory 
		function based on the received HADMapRoute and lanelet_map_ptr. Utilize appropriate 
		trajectory planning algorithms, such as path following or optimization techniques, 
		to generate a valid and feasible trajectory. Perform necessary checks and constraints 
		to ensure the generated trajectory complies with the system's limitations and safety 
		requirements. Return the generated trajectory from the function for further processing 
		or execution.  
	/*****************************************************************************************/
  return m_planner->plan_trajectory(had_map_route, lanelet_map_ptr);
  
  /*****************************************************************************************/
  /* Hazard: (4) Operates at the wrong time (early)                                        */
  /*                                                                                       */
  /* Issue Description: The plan_trajectory function does not currently check whether it    */
  /* is the right time to operate or start planning. This may result in premature or        */
  /* incorrect planning, leading to undesirable behavior or erroneous trajectories.         */
  /*                                                                                       */
  /* Mitigation Technique: Implement a mechanism to ensure that the planning operation      */
  /* starts at the appropriate time. This could involve checking system conditions or       */
  /* external triggers to determine when it is safe and necessary to start planning.        */
  /* Perform the planning operation only when all prerequisites and conditions are met.     */
  /*                                                                                       */
  /* Note: This mitigation may involve modifications outside the provided code snippet,    */
  /* as it depends on the overall system architecture and requirements.                      */
  /*****************************************************************************************/
  
  /*****************************************************************************************/
  /* Hazard: (5) Operates at the wrong time (late)                                         */
  /*                                                                                       */
  /* Issue Description: The plan_trajectory function does not currently check whether it    */
  /* is the right time to operate or start planning. This may result in delayed or          */
  /* missed planning, causing the system to be unresponsive or fail to generate trajectories */
  /* when needed.                                                                          */
  /*                                                                                       */
  /* Mitigation Technique: Implement a mechanism to ensure that the planning operation      */
  /* starts at the appropriate time. This could involve checking system conditions or       */
  /* external triggers to determine when it is safe and necessary to start planning.        */
  /* Perform the planning operation promptly when the conditions are met to avoid           */
  /* unnecessary delays or missed planning cycles.                                         */
  /*                                                                                       */
  /* Note: This mitigation may involve modifications outside the provided code snippet,    */
  /* as it depends on the overall system architecture and requirements.                      */
  /*****************************************************************************************/
  
  /*****************************************************************************************/
  /* Hazard: (9) Conflicting data or information                                            */
  /*                                                                                       */
  /* Issue Description: The plan_trajectory function does not currently handle or resolve   */
  /* conflicting data or information from the HADMapRoute and lanelet_map_ptr. This may     */
  /* result in inconsistencies or incorrect trajectory planning, leading to unsafe or       */
  /* unreliable behavior.                                                                  */
  /*                                                                                       */
  /* Mitigation Technique: Implement a mechanism to handle and resolve conflicting data or  */
  /* information from different sources. This could involve prioritizing certain data,     */
  /* performing data fusion or synchronization, or applying appropriate algorithms or      */
  /* decision-making logic to resolve conflicts. Prevent any conflicting or inconsistent    */
  /* data from influencing the trajectory planning process.                                 */
  /*                                                                                       */
  /* Note: This mitigation may involve modifications outside the provided code snippet,    */
  /* as it depends on the overall system architecture and requirements.                      */
  /*****************************************************************************************/

}


}  // namespace lane_planner_nodes
}  // namespace autoware

#include "rclcpp_components/register_node_macro.hpp"  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(
  autoware::lane_planner_nodes::LanePlannerNode)