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

#include "trajectory_planner_node_base/trajectory_planner_node_base.hpp"
#include <had_map_utils/had_map_conversion.hpp>

//lint -e537 NOLINT  // cpplint vs pclint
#include <string>
#include <chrono>
#include <memory>

namespace autoware
{
namespace trajectory_planner_node_base
{

TrajectoryPlannerNodeBase::TrajectoryPlannerNodeBase(
  const std::string & node_name,
  const std::string & action_server_name,
  const rclcpp::NodeOptions & node_options)
: Node{node_name, node_options},
  m_planner_state{PlannerState::IDLE}
{
	/*****************************************************************************************/
	/*	Failure Mode: (1)Fails to operate                                                    */
	/*	                                                                                     */
	/*	Hazard Type: Operational Failure                                                     */
	/*	                                                                                     */
	/*	Issue Description: The node may fail to operate if the map service is not available 
		or if there are initialization errors.					
	
	/*	Mitigation Technique: Properly handle errors during initialization and ensure that the 
		map service is available before proceeding.   
	/*****************************************************************************************/	
	
  using namespace std::literals::chrono_literals;

  // Setup Map Service
  m_map_client = this->create_client<HADMapService>("HAD_Map_Service");

  while (!m_map_client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(get_logger(), "Interrupted while waiting for map server.");
      rclcpp::shutdown();
      return;
    }
    RCLCPP_INFO(get_logger(), "Waiting for map service...");
  }

  m_planner_server = rclcpp_action::create_server<PlanTrajectoryAction>(
    this->get_node_base_interface(),
    this->get_node_clock_interface(),
    this->get_node_logging_interface(),
    this->get_node_waitables_interface(),
    action_server_name,
    [this](auto uuid, auto goal) {return this->handle_goal(uuid, goal);},
    [this](auto goal_handle) {return this->handle_cancel(goal_handle);},
    [this](auto goal_handle) {return this->handle_accepted(goal_handle);});
}

bool8_t TrajectoryPlannerNodeBase::is_trajectory_valid(const Trajectory & trajectory)
{
	/*****************************************************************************************/
	/*	Failure Mode: (2)Operates incorrectly/erroneously                                    */
	/*	                                                                                     */
	/*	Hazard Type: Functional Failure                                                      */
	/*	                                                                                     */
	/*	Issue Description: The function only checks if the trajectory points are empty, 
		which is a minimal validation.More comprehensive validation, such as checking 
		resolution, kinematic feasibility, etc., is missing.				
	
	/*	Mitigation Technique: Add additional validation checks to ensure the trajectory's 
		correctness. 
	/*****************************************************************************************/	
  if (trajectory.points.empty()) {
    return false;
  }

  // currently we only check minimal validation
  // TODO(mitudome-r): add more validation in the future.
  // e.g. resolution of trajectory, kinematic feasibility, ...

  return true;
}
bool8_t TrajectoryPlannerNodeBase::is_planning()
{
  return m_planner_state == PlannerState::PLANNING;
}
	
void TrajectoryPlannerNodeBase::start_planning()
{
	/*	Failure Mode: (6)Unable to stop operation			 	   		                     */
	/*	                                                                                     */
	/*	Hazard Type: Timing Failure                     	                                 */
	/*	                                                                                     */
	/*	Issue Description:THIS FUNCTION does not perform any checks to ensure that planning is 
		not already in progress.
		It can potentially lead to the planner operating incorrectly if called when planning
		is already ongoing.	
	/*	                                                                                     */
	/*	Mitigation Technique: Add a check to ensure planning is not already in progress 
		before starting planning.
	/*****************************************************************************************/
  m_planner_state = PlannerState::PLANNING;
}
void TrajectoryPlannerNodeBase::stop_planning()
{
	/*	Failure Mode: (6)Unable to stop operation			 	   		                     */
	/*	                                                                                     */
	/*	Hazard Type: Timing Failure                                                          */
	/*	                                                                                     */
	/*	Issue Description: this function only updates the planner state but does not perform
		any actions to stop an ongoing planning process.
		If there are any active operations related to planning, they might continue even 
		after the planner state is set to IDLE.													 
	/*	                                                                                     */
	/*	Mitigation Technique: Implement the necessary logic to stop an ongoing planning 
		process when this function is called.														
	/*****************************************************************************************/
	
  m_planner_state = PlannerState::IDLE;
}

rclcpp_action::GoalResponse TrajectoryPlannerNodeBase::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  const std::shared_ptr<const PlanTrajectoryAction::Goal> goal)
{
  (void)goal;
  (void)uuid;
	
  if (is_planning()) {
	/*****************************************************************************************/
	/*	Failure Mode: (5)Operates at the wrong time (late)        		                     */
	/*	                                                                                     */
	/*	Hazard Type: Timing Failure                                                          */
	/*	                                                                                     */
	/*	Issue Description: The handle_goal() function does not handle the case when a goal 
		is received after planning has already finished.It should reject late goals to ensure 
		the planner operates within the expected timeframe.
	/*	                                                                                     */
	/*	Mitigation Technique: Implement a mechanism to reject goals received after the 
		planning process has completed.
	/*****************************************************************************************/
	
    // Can't start replaying if we already are
    RCLCPP_ERROR(this->get_logger(), "Planner is already running. Rejecting new goal.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  RCLCPP_INFO(this->get_logger(), "received new goal");
  start_planning();
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TrajectoryPlannerNodeBase::handle_cancel(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  (void)goal_handle;
  if (is_planning()) {
    RCLCPP_INFO(this->get_logger(), "Cancel replaying");
    auto result = std::make_shared<PlanTrajectoryAction::Result>();
    result->result = PlanTrajectoryAction::Result::FAIL;
    m_goal_handle->canceled(result);
    stop_planning();
  }
  
	/*****************************************************************************************/
	/*	Failure Mode: (4)Operates at the wrong time (early)      		                     */
	/*	                                                                                     */
	/*	Hazard Type: Timing Failure                                                          */
	/*	                                                                                     */
	/*	Issue Description: The handle_cancel() function cancels the planning process 
		immediately if it is already in progress.However, this could result in early 
		cancellation before the planning has fully completed. 														    	
	/*	                                                                                     */
	/*	Mitigation Technique: Add checks or conditions to ensure that cancellation occurs at 
		appropriate stages of the planning process.    														
	/*****************************************************************************************/

  return rclcpp_action::CancelResponse::ACCEPT;
}

void TrajectoryPlannerNodeBase::handle_accepted(
  const std::shared_ptr<GoalHandle> goal_handle)
{
  // Store the goal handle in order to send result in map_response callback.
  m_goal_handle = goal_handle;
  
	/*	Failure Mode: (9)Conflicting data or information			 	   		             */
	/*	                                                                                     */
	/*	Hazard Type: Data Integrity Failure                                                  */
	/*	                                                                                     */
	/*	Issue Description: The handle_accepted() function assumes that the provided 
		goal_handle is the most recent one.There is a potential risk of conflicting data if
		multiple goal handles are accepted simultaneously.	 */ 										
	/*	                                                                                     */
	/*	Mitigation Technique: Implement proper handling to ensure data consistency and 
		prevent conflicts in case of multiple accepted goal handles.						 */
	/*****************************************************************************************/

  auto map_request = std::make_shared<HADMapService::Request>();
  *map_request = create_map_request(m_goal_handle->get_goal()->sub_route);
  
	/*	Failure Mode: (7)Receives erroneous data			 	   		                     */
	/*	                                                                                     */
	/*	Hazard Type: Data Integrity Failure                                                  */
	/*	                                                                                     */
	/*	Issue Description: 
		1.The node receives a goal with erroneous or invalid data.	
		2.There is no validation or error handling when creating the map request from the goal data.
		  It assumes that the provided sub_route is valid, which can lead to incorrect map requests.
	/*	                                                                                     */
	/*	Mitigation Technique: 
		1.Implement input data validation to ensure the received goal contains valid and meaningful information.
		2.Implement proper validation and error handling to ensure the correctness of the map request.
	/*****************************************************************************************/

  // TODO(mitsudome-r): If synchronized service request is available,
  // replace it with synchronized implementation
  auto result =
    m_map_client->async_send_request(
    map_request,
    std::bind(&TrajectoryPlannerNodeBase::map_response, this, std::placeholders::_1));
}

void TrajectoryPlannerNodeBase::map_response(rclcpp::Client<HADMapService>::SharedFuture future)
{
  auto lanelet_map_ptr = std::make_shared<lanelet::LaneletMap>();
  autoware::common::had_map_utils::fromBinaryMsg(future.get()->map, lanelet_map_ptr);

  RCLCPP_INFO(get_logger(), "Start planning");
	/*****************************************************************************************/
	/*	Failure Mode: (3)Operates inadvertently  			                                 */
	/*	                                                                                     */
	/*	Hazard Type: Functional Failure                                                      */
	/*	                                                                                     */
	/*	Issue Description: The plan_trajectory() function is called directly without checking 
		the planner state.It may lead to unintended planning operations being triggered.  	 */
	/*	                                                                                     */
	/*	Mitigation Technique: Add proper checks to ensure that planning is performed only
		when the planner is in the correct state.
	/*****************************************************************************************/	
  const auto & trajectory = plan_trajectory(m_goal_handle->get_goal()->sub_route, lanelet_map_ptr);
  
	/*****************************************************************************************/
	/*	Failure Mode: (2)Operates incorrectly/erroneously                                    */
	/*	                                                                                     */
	/*	Hazard Type: Functional Failure                                                      */
	/*	                                                                                     */
	/*	Issue Description: The node may generate an incorrect or erroneous trajectory.  	 */
	/*	                                                                                     */
	/*	Mitigation Technique: Perform thorough testing and verification of the planning 
		algorithm to ensure correctness and accuracy.
	/*****************************************************************************************/	
	
  RCLCPP_INFO(get_logger(), "Finished planning");

  if (is_trajectory_valid(trajectory)) {
    auto result = std::make_shared<PlanTrajectoryAction::Result>();
    result->result = PlanTrajectoryAction::Result::SUCCESS;
    result->trajectory = trajectory;
	
	/*	Failure Mode: (8)Sends erroneous data			 		   		                     */
	/*	                                                                                     */
	/*	Hazard Type: Data Integrity Failure                                                  */
	/*	                                                                                     */
	/*	Issue Description: The result trajectory is assigned to the action result without
		proper validation or error checking.There is a possibility that erroneous or invalid
		data could be sent as the result.										 										
	/*	                                                                                     */
	/*	Mitigation Technique:  Implement thorough validation checks to ensure that only valid 
		and reliable data is sent as the result.										 
	/*****************************************************************************************/
	
    m_goal_handle->succeed(result);
    RCLCPP_INFO(get_logger(), "Sent planned trajectory with %d points", trajectory.points.size());
  } else {
	/*****************************************************************************************/
	/*	Failure Mode: (1)Fails to operate                                                    */
	/*	                                                                                     */
	/*	Hazard Type: Operational Failure                                                     */
	/*	                                                                                     */
	/*	Issue Description: The node fails to operate due to an invalid trajectory.						
	
	/*	Mitigation Technique: Provide appropriate feedback indicating that planning has failed 
		due to an invalid trajectory.		
	/*****************************************************************************************/	
    RCLCPP_INFO(get_logger(), "Aborting planning due to invalid trajectory");
    auto result = std::make_shared<PlanTrajectoryAction::Result>();
    result->result = PlanTrajectoryAction::Result::FAIL;
    m_goal_handle->abort(result);
  }
	/*****************************************************************************************/
	/*	Failure Mode: (3)Operates inadvertently          		                             */
	/*	                                                                                     */
	/*	Hazard Type: Operational Failure                                                     */
	/*	                                                                                     */
	/*	Issue Description:The stop_planning() function is called unconditionally after the 
		planning process, regardless of success or failure.
		This can lead to unintended behavior where the planner transitions to the IDLE state 
		even if planning was not started.
	/*	                                                                                     */
	/*	Mitigation Technique: Place the stop_planning() function only when planning is started 
		successfully to avoid unintended operation.	
	/*****************************************************************************************/
	
  stop_planning();
}

}  // namespace trajectory_planner_node_base
}  // namespace autoware