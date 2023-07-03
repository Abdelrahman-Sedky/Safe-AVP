// Copyright 2019 Christopher Ho
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

/*****************************************************************************************/
/*	Failure Mode: (1)Fails to operate                                                    */
/*	                                                                                     */
/*	Hazard Type: Functional failure                                                      */
/*	                                                                                     */
/*	Issue Description: The MPC controller fails to initialize or execute its control     */
/*	algorithm correctly, resulting in a complete failure to operate as intended.         */
/*	                                                                                     */
/*	How to Discover it: Monitor the controller's initialization process and execution    */
/*	for any errors, exceptions, or unexpected termination.                               */
/*	                                                                                     */
/*	Mitigation Technique: Implement robust error handling and logging mechanisms         */
/*	to identify and handle initialization or execution failures. Use appropriate         */
/*	exception handling techniques and ensure proper error messages or notifications      */
/*	are generated to alert system operators.                                             */
/*****************************************************************************************/	

#include "mpc_controller_nodes/mpc_controller_node.hpp"

#include <memory>
#include <string>
#include <utility>

namespace motion
{
namespace control
{
namespace mpc_controller_nodes
{
////////////////////////////////////////////////////////////////////////////////
MpcControllerNode::MpcControllerNode(const std::string & name, const std::string & ns)
: ControllerBaseNode{name, ns}
{
	
/*****************************************************************************************/
/*	Failure Mode: (2)Operates incorrectly/erroneously                                    */
/*	                                                                                     */
/*	Hazard Type: Functional failure                                                      */
/*	                                                                                     */
/*	Issue Description:  The MPC controller performs control actions     				 */
/*	that deviate from the desired behavior, leading to incorrect or erroneous operation. */
/*	                                                                                     */
/*	How to Discover it: Analyze the controller's output and compare it					 */
/*  with the expected control actions or behavior.Monitor system behavior     			 */
/*  and performance for any unexpected or undesired responses.                           */
/*	                                                                                     */
/*	Mitigation Technique: Conduct rigorous testing and validation of the MPC			 */
/*  controller's algorithms using simulated scenarios and real-world data. Implement	 */
/*  model verification techniques, such as model checking or formal methods, 			 */
/*  to ensure the correctness of the controller's operation.                             */           
/*****************************************************************************************/	


/*****************************************************************************************/
/*	Failure Mode: (3)Operates inadvertently    											 */
/*												                             		     */
/*  Hazard Type: Functional failure                                                      */
/*                                                                                       */
/*  Issue Description: The MPC controller operates unintentionally or unexpectedly,      */
/*  leading to unintended control actions or system behavior.                            */
/*                                                                                       */
/*  How to Discover it: Monitor the controller's activation and execution triggers       */
/*  for any unexpected or uncontrolled occurrences. Analyze the system's response        */
/*  and behavior for any unanticipated effects.                                          */
/*                                                                                       */
/*  Mitigation Technique: Implement explicit control mechanisms to activate              */
/*  and deactivate the MPC controller based on well-defined conditions or triggers.      */
/*  Use appropriate system-level checks and safeguards to prevent unintended             */
/*  controller activation or operation.                                                  */
/*****************************************************************************************/


/******************************************************************************************/
/*	Failure Mode: (4),(5)Operates at the wrong time (early or early)   				      */		 
/*                                                                                        */
/*  Hazard Type: Timing-related failure                                                   */
/*                                                                                        */
/*  Issue Description: The MPC controller starts its operation prematurely, initiating    */
/*  control actions before the intended timing or event.								  */
/*  The MPC controller starts its operation later than the intended						  */
/*  timing or event, causing delays in control actions and system response.               */
/*                                                                                        */
/*  How to Discover it: Monitor the controller's activation triggers and timing           */
/*  synchronization with other system components. Analyze the system's behavior           */
/*  to identify instances of early control action.                                        */
/*                                                                                        */
/*  Mitigation Technique: Ensure proper synchronization and coordination between          */
/*  the MPC controller and other system components. Implement appropriate timing          */
/*  mechanisms, such as event-based triggers or synchronization signals, to initiate      */
/*  the controller's operation at the correct time.                                       */
/******************************************************************************************/

/******************************************************************************************/
/*	Failure Mode: (6)Unable to stop operation  					    	        		  */
/*                                                                                        */
/*  Hazard Type: Control failure                                                          */
/*                                                                                        */
/*  Issue Description: The MPC controller lacks mechanisms to stop or terminate           */
/*  its operation under certain conditions or external triggers, leading to               */
/*  uncontrolled or prolonged operation.                                                  */
/*                                                                                        */
/*  How to Discover it: Identify the conditions or events that should trigger             */
/*  the termination of the controller's operation. Monitor the controller's               */
/*  behavior and system response to determine if it can be stopped when necessary.        */
/*                                                                                        */
/*  Mitigation Technique: Implement proper stop and termination mechanisms in the MPC     */
/*  controller to allow for controlled and timely cessation of operation. Define clear    */
/*  criteria and triggers for stopping the controller's execution, and ensure their       */
/*  correct implementation.                                                               */
/******************************************************************************************/

  using mpc_controller::Real;
  using mpc_controller::LimitsConfig;
  const LimitsConfig limits{
    {
      static_cast<Real>(
        declare_parameter("controller.limits.min_longitudinal_velocity_mps").get<double>()),
      static_cast<Real>(
        declare_parameter("controller.limits.max_longitudinal_velocity_mps").get<double>())
    },
    {
      static_cast<Real>(
        declare_parameter("controller.limits.min_lateral_velocity_mps").get<double>()),
      static_cast<Real>(
        declare_parameter("controller.limits.max_lateral_velocity_mps").get<double>())
    },
    {
      static_cast<Real>(declare_parameter("controller.limits.min_acceleration_mps2").get<double>()),
      static_cast<Real>(declare_parameter("controller.limits.max_acceleration_mps2").get<double>())
    },
    {
      static_cast<Real>(declare_parameter("controller.limits.min_yaw_rate_rps").get<double>()),
      static_cast<Real>(declare_parameter("controller.limits.max_yaw_rate_rps").get<double>())
    },
    {
      static_cast<Real>(declare_parameter("controller.limits.min_jerk_mps3").get<double>()),
      static_cast<Real>(declare_parameter("controller.limits.max_jerk_mps3").get<double>())
    },
    {
      static_cast<Real>(declare_parameter("controller.limits.min_steer_angle_rad").get<double>()),
      static_cast<Real>(declare_parameter("controller.limits.max_steer_angle_rad").get<double>())
    },
    {
      static_cast<Real>(
        declare_parameter("controller.limits.min_steer_angle_rate_rps").get<double>()),
      static_cast<Real>(
        declare_parameter("controller.limits.max_steer_angle_rate_rps").get<double>())
    },
  };
  using mpc_controller::VehicleConfig;
  const VehicleConfig vehicle_param{
    static_cast<Real>(declare_parameter("vehicle.cg_to_front_m").get<double>()),
    static_cast<Real>(declare_parameter("vehicle.cg_to_rear_m").get<double>()),
    static_cast<Real>(declare_parameter("vehicle.front_corner_stiffness").get<double>()),
    static_cast<Real>(declare_parameter("vehicle.rear_corner_stiffness").get<double>()),
    static_cast<Real>(declare_parameter("vehicle.mass_kg").get<double>()),
    static_cast<Real>(declare_parameter("vehicle.yaw_inertia_kgm2").get<double>()),
    static_cast<Real>(declare_parameter("vehicle.width_m").get<double>()),
    static_cast<Real>(declare_parameter("vehicle.front_overhang_m").get<double>()),
    static_cast<Real>(declare_parameter("vehicle.rear_overhang_m").get<double>())
  };
  using mpc_controller::BehaviorConfig;
  using controller_common::ControlReference;
  auto ref_type = ControlReference::SPATIAL;
  if (declare_parameter("controller.behavior.is_temporal_reference").get<bool>()) {
    ref_type = ControlReference::TEMPORAL;
  }
  const BehaviorConfig behavior{
    static_cast<Real>(declare_parameter("controller.behavior.stop_rate_mps2").get<double>()),
    std::chrono::milliseconds(declare_parameter("controller.behavior.time_step_ms").get<int64_t>()),
    ref_type
  };

  using mpc_controller::OptimizationConfig;
  using mpc_controller::StateWeight;
  const OptimizationConfig weights{
    StateWeight{
      static_cast<Real>(declare_parameter("controller.weights.nominal.pose").get<double>()),
      static_cast<Real>(declare_parameter("controller.weights.nominal.heading").get<double>()),
      static_cast<Real>(
        declare_parameter("controller.weights.nominal.longitudinal_velocity").get<double>()),
      static_cast<Real>(
        declare_parameter("controller.weights.nominal.lateral_velocity").get<double>()),
      static_cast<Real>(declare_parameter("controller.weights.nominal.yaw_rate").get<double>()),
      static_cast<Real>(declare_parameter("controller.weights.nominal.acceleration").get<double>()),
      static_cast<Real>(declare_parameter("controller.weights.nominal.jerk").get<double>()),
      static_cast<Real>(declare_parameter("controller.weights.nominal.steer_angle").get<double>()),
      static_cast<Real>(
        declare_parameter("controller.weights.nominal.steer_angle_rate").get<double>()),
    },
    StateWeight{
      static_cast<Real>(declare_parameter("controller.weights.terminal.pose").get<double>()),
      static_cast<Real>(declare_parameter("controller.weights.terminal.heading").get<double>()),
      static_cast<Real>(
        declare_parameter("controller.weights.terminal.longitudinal_velocity").get<double>()),
      static_cast<Real>(
        declare_parameter("controller.weights.terminal.lateral_velocity").get<double>()),
      static_cast<Real>(declare_parameter("controller.weights.terminal.yaw_rate").get<double>()),
      static_cast<Real>(
        declare_parameter("controller.weights.terminal.acceleration").get<double>()),
      static_cast<Real>(declare_parameter("controller.weights.terminal.jerk").get<double>()),
      static_cast<Real>(declare_parameter("controller.weights.terminal.steer_angle").get<double>()),
      static_cast<Real>(
        declare_parameter("controller.weights.terminal.steer_angle_rate").get<double>()),
    }
  };

  using mpc_controller::Interpolation;
  auto interpolation = Interpolation::NO;
  if (declare_parameter("controller.interpolation").get<bool>()) {
    interpolation = Interpolation::YES;
  }
  const auto sample_tolerance_ms =
    std::chrono::milliseconds(declare_parameter("controller.sample_tolerance_ms").get<int64_t>());

  const auto control_lookahead_ms =
    std::chrono::milliseconds(declare_parameter("controller.control_lookahead_ms").get<int64_t>());


/*********************************************************************************************/
/*	Failure Mode: (7)Receives erroneous data    						        		     */
/*                                                                                           */
/*  Hazard Type: Data-related failure                                                        */
/*                                                                                           */
/*  Issue Description: The MPC controller receives incorrect or invalid input data,          */
/*  such as measurements or reference signals, which can lead to erroneous control actions.  */
/*                                                                                           */
/*  How to Discover it: Monitor the input data flow to the MPC controller and analyze        */
/*  the controller's response based on the received data. Compare the input data             */
/*  with expected ranges, validity criteria, or known error patterns.                        */
/*                                                                                           */
/*  Mitigation Technique: Implement data validation and sanity checks for the input data     */
/*  received by the MPC controller. Use appropriate data filtering, error detection,         */
/*  and correction techniques to handle erroneous or inconsistent data.                      */
/*********************************************************************************************/


/*********************************************************************************************/
/*	Failure Mode: (8)Sends erroneous data  					        	        		     */
/*                                                                                           */
/*  Issue Description: The MPC controller transmits or communicates erroneous or invalid data*/
/*  to external systems, potentially affecting their operation or decision-making processes. */
/*                                                                                           */
/*  How to Discover it: Monitor the data sent or communicated by the MPC controller and      */
/*  analyze its consistency and conformity with the expected format or requirements          */
/*  of the receiving systems.                                                                */
/*                                                                                           */
/*  Mitigation Technique: Implement data validation and verification mechanisms for the data */
/*  transmitted by the MPC controller. Ensure data integrity and correctness before sending  */
/*  it to external systems. Use appropriate error-checking and fault-tolerant communication  */
/*  protocols to detect and handle transmission errors.                                      */
/*********************************************************************************************/


/*********************************************************************************************/
/*	Failure Mode: (9)Conflicting data or information					        	         */
/*                                                                                           */
/*  Hazard Type: Data-related failure                                                        */
/*                                                                                           */
/*  Issue Description: Conflicting or contradictory data from different sources can lead to  */
/*  inconsistencies or ambiguity in the MPC controller's operation, affecting control actions*/
/*  and system behavior.                                                                     */
/*                                                                                           */
/*  How to Discover it: Analyze the input data from different sources and identify instances */
/*  where conflicting or inconsistent information is provided to the MPC controller.         */
/*  Monitor the controller's behavior and system response for any indications of             */
/*  conflicting data effects.                                                                */
/*                                                                                           */
/*  Mitigation Technique: Implement data fusion and integration techniques to reconcile      */
/*  conflicting or inconsistent data from multiple sources. Use appropriate algorithms,      */
/*  such as sensor fusion or consensus algorithms, to derive a consistent and reliable       */
/*  representation of the system state. Apply data validation and verification techniques    */
/*  to identify and resolve conflicts or inconsistencies in the input data.                  */
/*********************************************************************************************/


  auto controller = std::make_unique<mpc_controller::MpcController>(
    mpc_controller::Config{
          limits,
          vehicle_param,
          behavior,
          weights,
          sample_tolerance_ms,
          control_lookahead_ms,
          interpolation});
  // I argue this is ok for the following reasons:
  // The parent class, ControllerBaseNode, has unique ownership of the controller, and the timer
  // only has a non-owning pointer. This is fine because the timer can never go out of scope before
  // the base class (and thus the owning pointer)
  const auto ctrl_ptr = controller.get();
  set_controller(std::move(controller));

  const auto debug_cycle_duration_param = declare_parameter("debug_trajectory_publish_period_ms");
  if (rclcpp::PARAMETER_INTEGER == debug_cycle_duration_param.get_type()) {
    const auto cycle_duration =
      std::chrono::milliseconds{debug_cycle_duration_param.get<std::int64_t>()};
    if (decltype(cycle_duration)::zero() != cycle_duration) {
      m_debug_traj_pub = create_publisher<autoware_auto_msgs::msg::Trajectory>(
        "mpc_debug_computed_trajectory",
        rclcpp::QoS{10LL});
      const auto debug_publish = [this, ctrl_ptr]() -> void {
          auto traj = ctrl_ptr->get_computed_trajectory();
          traj.header.frame_id = "map";
          m_debug_traj_pub->publish(traj);
        };
      m_debug_timer = create_wall_timer(cycle_duration, debug_publish);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
MpcControllerNode::MpcControllerNode(
  const std::string & name,
  const std::string & ns,
  const std::string & command_topic,
  const std::string & state_topic,
  const std::string & tf_topic,
  const std::string & trajectory_topic,
  const std::string & diagnostic_topic,
  const std::string & static_tf_topic,
  const mpc_controller::Config & config)
: ControllerBaseNode{
    name,
    ns,
    command_topic,
    state_topic,
    tf_topic,
    trajectory_topic,
    diagnostic_topic,
    static_tf_topic}
{
  set_controller(std::make_unique<mpc_controller::MpcController>(config));
}
}  // namespace mpc_controller_nodes
}  // namespace control
}  // namespace motion
