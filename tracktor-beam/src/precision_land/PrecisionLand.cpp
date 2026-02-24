#include "PrecisionLand.hpp"

#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/utils/geometry.hpp>
#include <px4_ros2/utils/message_version.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

static const std::string kModeName = "PrecisionLandCustom";
static const bool kEnableDebugOutput = true;

using namespace px4_ros2::literals;

PrecisionLand::PrecisionLand(rclcpp::Node& node)
	: ModeBase(node, kModeName)
	, _node(node)
{

	_trajectory_setpoint = std::make_shared<px4_ros2::TrajectorySetpointType>(*this);
	
	_vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);

	_vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);

	_target_pose_sub = _node.create_subscription<geometry_msgs::msg::PoseStamped>("/target_pose",
			   rclcpp::QoS(1).best_effort(), std::bind(&PrecisionLand::targetPoseCallback, this, std::placeholders::_1));

	_small_marker_detected_sub = _node.create_subscription<std_msgs::msg::Bool>("/small_marker_detected",
			   rclcpp::QoS(1).best_effort(), std::bind(&PrecisionLand::smallMarkerDetectedCallback, this, std::placeholders::_1));
	
	RCLCPP_INFO(_node.get_logger(), "Subscribed to /small_marker_detected topic");

	_vehicle_land_detected_sub = _node.create_subscription<px4_msgs::msg::VehicleLandDetected>("/fmu/out/vehicle_land_detected",
				     rclcpp::QoS(1).best_effort(), std::bind(&PrecisionLand::vehicleLandDetectedCallback, this, std::placeholders::_1));


    _gimbal_attitude_sub = _node.create_subscription<px4_msgs::msg::GimbalDeviceAttitudeStatus>("/fmu/out/gimbal_device_attitude_status",
                     rclcpp::QoS(1).best_effort(), std::bind(&PrecisionLand::gimbalAttitudeCallback, this, std::placeholders::_1));

	_target_pose_world_pub = _node.create_publisher<geometry_msgs::msg::PoseStamped>("/target_pose_world", rclcpp::QoS(1).best_effort());

	_vehicle_command_pub = _node.create_publisher<px4_msgs::msg::VehicleCommand>(
		topicNamespacePrefix() + "fmu/in/vehicle_command" +
		px4_ros2::getMessageNameVersion<px4_msgs::msg::VehicleCommand>(),
		1);

	_wamv_odom_sub = _node.create_subscription<nav_msgs::msg::Odometry>(
		"/model/wamv/odometry", 
		rclcpp::QoS(1).best_effort(),
		std::bind(&PrecisionLand::wamvOdomCallback, this, std::placeholders::_1));
	loadParameters();

	modeRequirements().manual_control = false;
}

void PrecisionLand::loadParameters()
{
	_node.declare_parameter<float>("descent_vel", 0.5);
	_node.declare_parameter<float>("vel_p_gain", 1.5);
	_node.declare_parameter<float>("vel_i_gain", 0.0);
	_node.declare_parameter<float>("max_velocity", 4.0);
	_node.declare_parameter<float>("target_timeout", 5.0);
	_node.declare_parameter<float>("delta_position", 3.0);
	_node.declare_parameter<float>("delta_velocity", 3.0);
	_node.declare_parameter<float>("velocity_filter_alpha", 1.0);

	_node.get_parameter("descent_vel", _param_descent_vel);
	_node.get_parameter("vel_p_gain", _param_vel_p_gain);
	_node.get_parameter("vel_i_gain", _param_vel_i_gain);
	_node.get_parameter("max_velocity", _param_max_velocity);
	_node.get_parameter("target_timeout", _param_target_timeout);
	_node.get_parameter("delta_position", _param_delta_position);
	_node.get_parameter("delta_velocity", _param_delta_velocity);
	_node.get_parameter("velocity_filter_alpha", _param_velocity_filter_alpha);

	RCLCPP_INFO(_node.get_logger(), "descent_vel: %f", _param_descent_vel);
	RCLCPP_INFO(_node.get_logger(), "vel_i_gain: %f", _param_vel_i_gain);
}

void PrecisionLand::gimbalAttitudeCallback(const px4_msgs::msg::GimbalDeviceAttitudeStatus::SharedPtr msg)
{
    _gimbal_orientation = Eigen::Quaterniond(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
}

void PrecisionLand::vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg)
{
	_land_detected = msg->landed;
	_ground_contact = msg->ground_contact;

	// Disarm immediately when ground contact is detected (first contact with boat)
	if (_ground_contact && !_disarmed_on_contact) {
		RCLCPP_INFO(_node.get_logger(), "Ground contact detected! Disarming vehicle to stop propellers.");
		
		px4_msgs::msg::VehicleCommand cmd{};
		cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM;
		cmd.param1 = 0.0f; // 0 = disarm
		cmd.param2 = 21196.0f; // Force disarm (bypass checks)
		cmd.timestamp = 0; // Let PX4 set the timestamp
		_vehicle_command_pub->publish(cmd);
		
		_disarmed_on_contact = true;
	}
}
void PrecisionLand::wamvGpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
    // 1. Direct Access
    _wamv_lat = msg->latitude;
    _wamv_lon = msg->longitude;

}
void PrecisionLand::wamvOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // 1. Direct Access
    _boat_x = static_cast<float>(msg->pose.pose.position.x);
    _boat_y = static_cast<float>(msg->pose.pose.position.y);
    _boat_found = true;

}
void PrecisionLand::smallMarkerDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
	//RCLCPP_DEBUG(_node.get_logger(), "smallMarkerDetectedCallback received: data=%d", msg->data);
	
	_small_marker_detected = msg->data;
	
	// Always log when we receive a message indicating small marker is detected
	if (_small_marker_detected) {
		RCLCPP_INFO(_node.get_logger(), "Small marker detection callback: detected=%d, previous=%d, state=%s", 
			_small_marker_detected, _small_marker_detected_prev, stateName(_state).c_str());
	}
	
	// Check if small marker was just detected (transition from false to true)
	if (_small_marker_detected && !_small_marker_detected_prev) {
		RCLCPP_INFO(_node.get_logger(), "Small marker transition detected: false -> true");
		
		// Small marker just detected - switch to Approach state at current altitude
		if (_state == State::Descend) {
			_approach_altitude = _vehicle_local_position->positionNed().z();
			switchToState(State::Approach);
			RCLCPP_INFO(_node.get_logger(), "Small marker detected - switching from Descend to Approach at current altitude: %.2f", _approach_altitude);
		} else if (_state == State::Search) {
			_approach_altitude = _vehicle_local_position->positionNed().z();
			switchToState(State::Approach);
			RCLCPP_INFO(_node.get_logger(), "Small marker detected - switching from Search to Approach at current altitude: %.2f", _approach_altitude);
		} else if (_state == State::Approach) {
			// Already in Approach, but update altitude to current position
			_approach_altitude = _vehicle_local_position->positionNed().z();
			RCLCPP_INFO(_node.get_logger(), "Small marker detected - updating Approach altitude to current: %.2f", _approach_altitude);
		}

		RCLCPP_INFO(_node.get_logger(), "Small marker detected - descent velocity set to 0.1");
	}
	
	_small_marker_detected_prev = _small_marker_detected;
}

void PrecisionLand::targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
	if (_search_started) {
		auto tag = ArucoTag {
			.position = Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z),
			.orientation = Eigen::Quaterniond(msg->pose.orientation.w, msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z),
			.timestamp = _node.now(),
		};
		
		// Save tag position/orientation in NED world frame
		_tag = getTagWorld(tag);
		if (_node.now().seconds() - _tag.timestamp.seconds() > 0.2) {
			_target_velocity_valid=false;
		} else {
			_target_velocity_valid=true;
		}
		// Estimate target velocity from position changes (for moving target support)
		if (_prev_tag_timestamp.nanoseconds() > 0) {
			double dt = (_tag.timestamp - _prev_tag_timestamp).seconds();
			
			if (dt > 0.001 && dt < 1.0) {  // Valid time delta
				Eigen::Vector2d current_pos(_tag.position.x(), _tag.position.y());
				Eigen::Vector2d pos_delta = current_pos - _prev_tag_position;
				Eigen::Vector2d raw_velocity = pos_delta / dt;
				
				// Apply low-pass filter to smooth velocity estimate
				double alpha = static_cast<double>(_param_velocity_filter_alpha);
				_target_velocity = alpha * raw_velocity + (1.0 - alpha) * _target_velocity;
			}
		}
		

		// Store current position and timestamp for next iteration
		_prev_tag_position = Eigen::Vector2d(_tag.position.x(), _tag.position.y());
		_prev_tag_timestamp = _tag.timestamp;

		// Publish world-frame tag pose for visualization
		geometry_msgs::msg::PoseStamped world_pose_msg;
		world_pose_msg.header.stamp = _node.now();
		world_pose_msg.header.frame_id = "map";
		world_pose_msg.pose.position.x = _tag.position.x();
		world_pose_msg.pose.position.y = _tag.position.y();
		world_pose_msg.pose.position.z = _tag.position.z();
		world_pose_msg.pose.orientation.w = _tag.orientation.w();
		world_pose_msg.pose.orientation.x = _tag.orientation.x();
		world_pose_msg.pose.orientation.y = _tag.orientation.y();
		world_pose_msg.pose.orientation.z = _tag.orientation.z();
		_target_pose_world_pub->publish(world_pose_msg);
	}

}
/*void PrecisionLand::commandReposition(double lat, double lon, float alt)
{
    px4_msgs::msg::VehicleCommand cmd{};
    
    // Command ID for Repositioning (Loiter at coordinate)
    cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_REPOSITION;
    
    // Standard PX4 system/component IDs
    cmd.target_system = 1;
    cmd.target_component = 1;
    cmd.source_system = 1;
    cmd.source_component = 1;
    cmd.from_external = true;
    cmd.timestamp = _node.get_clock()->now().nanoseconds() / 1000;

    // --- Parameters for VEHICLE_CMD_DO_REPOSITION ---
    // Param 1: Ground Speed in m/s (-1 = use default/current speed)
    cmd.param1 = -1.0f;
    
    // Param 2: Bitmask (Bit 0 = 1 to use param1 as ground speed)
    cmd.param2 = 1.0f; 
    
    // Param 3: Loiter radius (0 = default)
    cmd.param3 = 0.0f;
    
    // Param 4: Yaw Heading (NaN = keep current heading)
    cmd.param4 = std::numeric_limits<float>::quiet_NaN();
    
    // Param 5: Latitude (Note: truncated to float precision)
    cmd.param5 = static_cast<float>(lat);
    
    // Param 6: Longitude (Note: truncated to float precision)
    cmd.param6 = static_cast<float>(lon);
    
    // Param 7: Altitude (AMSL in meters)
    cmd.param7 = alt;

    // Publish the command
    _vehicle_command_pub->publish(cmd);

    RCLCPP_INFO(_node.get_logger(), "Sent Reposition Command to Lat: %f, Lon: %f, Alt: %f", 
        cmd.param5, cmd.param6, cmd.param7);
}*/
PrecisionLand::ArucoTag PrecisionLand::getTagWorld(const ArucoTag& tag)
{
	auto vehicle_position = Eigen::Vector3d(_vehicle_local_position->positionNed().cast<double>());
	auto vehicle_orientation = Eigen::Quaterniond(_vehicle_attitude->attitude().cast<double>());

	Eigen::Vector3d gimbal_mount_offset_body(0.0, 0.0, -0.16);

	Eigen::Quaterniond gimbal_relative = vehicle_orientation.inverse() * _gimbal_orientation;

	// Optical frame to FRD transformation
	Eigen::Matrix3d R_optical_to_frd;
	R_optical_to_frd << 0, 0, 1,
	                    1, 0, 0,
	                    0, 1, 0;
	Eigen::Quaterniond optical_to_frd(R_optical_to_frd);

	Eigen::Affine3d T_world_vehicle = Eigen::Translation3d(vehicle_position) * vehicle_orientation;
	Eigen::Affine3d T_gimbal_offset = Eigen::Translation3d(gimbal_mount_offset_body) * Eigen::Quaterniond::Identity();
	Eigen::Affine3d T_gimbal_relative(gimbal_relative);
	Eigen::Affine3d T_optical(optical_to_frd);
	Eigen::Affine3d T_tag = Eigen::Translation3d(tag.position) * tag.orientation;

	Eigen::Affine3d tag_world_transform = T_world_vehicle
	                                    * T_gimbal_offset
	                                    * T_gimbal_relative
	                                    * T_optical
	                                    * T_tag;

	ArucoTag world_tag = {
		.position = tag_world_transform.translation(),
		.orientation = Eigen::Quaterniond(tag_world_transform.rotation()),
		.timestamp = tag.timestamp,
	};

	return world_tag;
}


void PrecisionLand::onActivate()
{
	// --- Replace the old command block with this one ---
	RCLCPP_INFO(_node.get_logger(), "Requesting gimbal control (V2 Protocol)...");
	uint64_t now_us = _node.get_clock()->now().nanoseconds() / 1000;

	// --- STEP 1: Send CONFIGURE command to request control ---
	// This tells the gimbal manager "Give primary control to ME (this component)"
	px4_msgs::msg::VehicleCommand configure_cmd{};
	configure_cmd.timestamp = now_us;
	configure_cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_GIMBAL_MANAGER_CONFIGURE;
	
	configure_cmd.param1 = 0.0f; // 0.0f = this system
	configure_cmd.param2 = 0.0f; // 0.0f = this component
	configure_cmd.param3 = -1.0f; // no change to secondary
	configure_cmd.param4 = -1.0f; // no change to secondary
	configure_cmd.param5 = 0.0f;  // default gimbal
	
	_vehicle_command_pub->publish(configure_cmd);

    // --- We will send the PITCHYAW command in the updateSetpoint loop ---

	generateSearchWaypoints();
	_search_started = true;
	_disarmed_on_contact = false; // Reset disarm flag when mode is activated
	_target_velocity_valid = false; // Reset target velocity tracking
	_target_velocity = Eigen::Vector2d::Zero();
	_prev_tag_timestamp = rclcpp::Time(0); // Reset timestamp
	_small_marker_detected = false; // Reset small marker detection flag
	_small_marker_detected_prev = false; // Reset previous small marker detection state
	
}

void PrecisionLand::onDeactivate()
{
	// No-op
}

void PrecisionLand::updateSetpoint(float dt_s)
{
	// Initialize map projection once we have a valid reference
	/*if (!_map_ref.init && _vehicle_local_position->refLat() != 0.0) {
		map_projection_init(_map_ref, _vehicle_local_position->refLat(), _vehicle_local_position->refLon());
		RCLCPP_INFO(_node.get_logger(), "Map Projection Initialized!");
	}*/
	// --- Add this block to continuously command the gimbal ---
	px4_msgs::msg::VehicleCommand pitchyaw_cmd{};
	pitchyaw_cmd.timestamp = _node.get_clock()->now().nanoseconds() / 1000;
	pitchyaw_cmd.command = px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_GIMBAL_MANAGER_PITCHYAW;

	// param1: Pitch: -90 degrees (straight down in NED world frame)
	pitchyaw_cmd.param1 = -90.0f; 
	// param2: Yaw: Not controlled (set to NAN)
	pitchyaw_cmd.param2 = 0.0f;    
	// param3: Pitch rate (0)
	pitchyaw_cmd.param3 = 300.0f;   
	// param4: Yaw rate (0)
	pitchyaw_cmd.param4 = 0.0f;   
	
	// param5: GIMBAL_MANAGER_FLAGS
	// Flag 256 = GIMBAL_MANAGER_FLAGS_PITCH_IN_EARTH_FRAME
	// This explicitly forces world-frame stabilization for the pitch axis.
	pitchyaw_cmd.param5 = 28.0f; 

	// param6: Gimbal device ID (0 = default)
	pitchyaw_cmd.param6 = 0.0f;   

	_vehicle_command_pub->publish(pitchyaw_cmd);
	// --- End of added block ---
	float target_lost = checkTargetTimeout();
	//RCLCPP_INFO(_node.get_logger(), "Value of target_lost: %f", target_lost);

	if (target_lost > _param_target_timeout && !_target_lost_prev) {
		RCLCPP_INFO(_node.get_logger(), "Target lost: State %s", stateName(_state).c_str());

	} else if (target_lost < 0.01 && _target_lost_prev) {
		RCLCPP_INFO(_node.get_logger(), "Target acquired");
		switchToState(State::Approach);
	}

	_target_lost_prev = (target_lost > _param_target_timeout);

	// State machine
	switch (_state) {
	case State::Idle: {
		// No-op -- just spin
		break;
	}

	case State::Search: {

		if (!std::isnan(_tag.position.x())) {
			_approach_altitude = _vehicle_local_position->positionNed().z();
			switchToState(State::Approach);
			break;
		}

		/*auto waypoint_position = _search_waypoints[_search_waypoint_index];

		_trajectory_setpoint->updatePosition(waypoint_position);

		if (positionReached(waypoint_position)) {
			_search_waypoint_index++;

			// If we have searched all waypoints, start over
			if (_search_waypoint_index >= static_cast<int>(_search_waypoints.size())) {
				_search_waypoint_index = 0;
			}
		}*/
		if (_boat_found) {
			// Create a target position: Boat X, Boat Y, and 10m Altitude (-10m Z)
			Eigen::Vector3f target_pos(_boat_x, _boat_y, -10.0f);
			
			// Send command
			_trajectory_setpoint->updatePosition(target_pos);
			
		} else {
			// Hover if we don't know where the boat is
			_trajectory_setpoint->updatePosition(_vehicle_local_position->positionNed());
		}
		
		break;
	}

	case State::Approach: {

		/*if (target_lost > _param_target_timeout) {
			RCLCPP_INFO(_node.get_logger(), "Value of target_lost: %f", target_lost);
			RCLCPP_INFO(_node.get_logger(), "Failed! Target lost during %s", stateName(_state).c_str());
			ModeBase::completed(px4_ros2::Result::ModeFailureOther);
			switchToState(State::Idle);
			return;
		}*/

		// Approach using position setpoints
		auto target_position = Eigen::Vector3f(_tag.position.x(), _tag.position.y(), _approach_altitude);

		_trajectory_setpoint->updatePosition(target_position);

		if (positionReached(target_position)) {
			switchToState(State::Descend);
		}

		break;
	}

	case State::Descend: {

		if (target_lost > _param_target_timeout) {
			RCLCPP_INFO(_node.get_logger(), "Failed! Target lost during %s", stateName(_state).c_str());
			ModeBase::completed(px4_ros2::Result::ModeFailureOther);
			switchToState(State::Approach); // changed from idle to make it ascend again to find marker
			return;
		}

		// Descend using velocity setpoints and P velocity controller for XY
		Eigen::Vector2f vel = calculateVelocitySetpointXY();
		_trajectory_setpoint->update(Eigen::Vector3f(vel.x()*1.1, vel.y()*1.1, (_vehicle_local_position->positionNed().z() - _tag.position.z() < 0.5 ? _param_descent_vel*10 : _param_descent_vel)), std::nullopt,
					     std::nullopt);

		//float delta_pos_z = _vehicle_local_position->positionNed().z() - _tag.position.z();
		//if (delta_pos_z < 0.5)
		if (_land_detected) {
			switchToState(State::Finished);
		}

		break;
	}

	case State::Finished: {
		ModeBase::completed(px4_ros2::Result::Success);
		break;
	}
	} // end switch/case
}

Eigen::Vector2f PrecisionLand::calculateVelocitySetpointXY()
{
	// If tag position is not valid, return the previous known velocity (if valid)
	if (_node.now().seconds() - _tag.timestamp.seconds() > 0.01) 
	{
		if (_last_velocity_valid) {
			//RCLCPP_DEBUG(_node.get_logger(), "Using last known velocity: [%.3f, %.3f]", _last_velocity.x(), _last_velocity.y());
			return _last_velocity;
		} else {
			//RCLCPP_DEBUG(_node.get_logger(), "Tag invalid and no last velocity available, returning zero");
			return Eigen::Vector2f::Zero();
		}
	}

	float p_gain = _param_vel_p_gain;
	float i_gain = _param_vel_i_gain;

	// P component
	float delta_pos_x = _vehicle_local_position->positionNed().x() - _tag.position.x();
	float delta_pos_y = _vehicle_local_position->positionNed().y() - _tag.position.y();

	// I component
	_vel_x_integral += delta_pos_x;
	_vel_y_integral += delta_pos_y;
	float max_integral = _param_max_velocity;
	_vel_x_integral = std::clamp(_vel_x_integral, -1.f * max_integral, max_integral);
	_vel_y_integral = std::clamp(_vel_y_integral, -1.f * max_integral, max_integral);

	float Xp = delta_pos_x * p_gain;
	float Xi = _vel_x_integral * i_gain;
	float Yp = delta_pos_y * p_gain;
	float Yi = _vel_y_integral * i_gain;

	// Sum P and I gains
	float vx = -1.f * (Xp + Xi);
	float vy = -1.f * (Yp + Yi);

	// 0.1m/s min vel and 3m/s max vel
	vx = std::clamp(vx, -1.f * _param_max_velocity, _param_max_velocity);
	vy = std::clamp(vy, -1.f * _param_max_velocity, _param_max_velocity);

	Eigen::Vector2f velocity(vx, vy);

	// Update last velocity whenever we calculate a new one (when target is visible)
	_last_velocity = velocity;
	_last_velocity_valid = true;

	return velocity;
}

float PrecisionLand::checkTargetTimeout()
{
	/*if (!_tag.valid()) {
		return 9999.0f;
	}*/

	return _node.now().seconds() - _tag.timestamp.seconds();
}

void PrecisionLand::generateSearchWaypoints()
{
	// Generate spiral search waypoints
	// The search waypoints are generated in the NED frame
	// Parameters for the search pattern
	double start_x = 0.0;
	double start_y = 0.0;
	double current_z = _vehicle_local_position->positionNed().z();
	auto min_z = -1.0;

	double max_radius = 2.0;
	double layer_spacing = 0.5;
	int points_per_layer = 16;
	std::vector<Eigen::Vector3f> waypoints;

	// Generate waypoints
	// Calculate the number of layers needed
	int num_layers = (static_cast<int>((min_z - current_z) / layer_spacing) / 2) < 1 ? 1 : (static_cast<int>((
				 min_z - current_z) / layer_spacing) / 2);

	// Generate waypoints
	for (int layer = 0; layer < num_layers; ++layer) {
		std::vector<Eigen::Vector3f> layer_waypoints;

		// Spiral out to max radius
		double radius = 0.0;

		for (int point = 0; point < points_per_layer + 1; ++point) {
			double angle = 2.0 * M_PI * point / points_per_layer;
			double x = start_x + radius * cos(angle);
			double y = start_y + radius * sin(angle);
			double z = current_z;

			layer_waypoints.push_back(Eigen::Vector3f(x, y, z));
			radius += max_radius / points_per_layer;
		}

		// Push the spiral out waypoints to the main waypoints vector
		waypoints.insert(waypoints.end(), layer_waypoints.begin(), layer_waypoints.end());

		// Decrease the altitude for the inward spiral
		current_z += layer_spacing;

		// Reverse the layer waypoints for spiral in
		std::reverse(layer_waypoints.begin(), layer_waypoints.end());

		// Adjust the z-coordinate for the inward spiral
		for (auto& waypoint : layer_waypoints) {
			waypoint.z() = current_z;
		}

		// Push the reversed waypoints to the main waypoints vector
		waypoints.insert(waypoints.end(), layer_waypoints.begin(), layer_waypoints.end());

		// Decrease the altitude for the next outward spiral
		current_z += layer_spacing;
	}

	_search_waypoints = waypoints;
}

bool PrecisionLand::positionReached(const Eigen::Vector3f& target) const
{
	auto position = _vehicle_local_position->positionNed();
	auto vehicle_velocity = _vehicle_local_position->velocityNed();

	const auto delta_pos = target - position;
	Eigen::Vector2f delta_pos_xy(delta_pos.x(), delta_pos.y());
	//RCLCPP_INFO(_node.get_logger(), "Relative pos x: %f", delta_pos.x());
	//RCLCPP_INFO(_node.get_logger(), "Relative pos y: %f", delta_pos.y());
	// For moving targets: check relative velocity instead of absolute velocity
	// Relative velocity = vehicle velocity - target velocity
	Eigen::Vector2f relative_velocity;
	
	if (_target_velocity_valid) {
		// Calculate relative velocity (how fast drone is moving relative to target)
		Eigen::Vector2f target_vel_2d(_target_velocity.x(), _target_velocity.y());
		Eigen::Vector2f vehicle_vel_2d(vehicle_velocity.x(), vehicle_velocity.y());
		relative_velocity = vehicle_vel_2d - target_vel_2d;

		// Corrected (Using printf-style %f specifier)
		//RCLCPP_INFO(_node.get_logger(), "Relative velocity X: %f", relative_velocity.x());
		//RCLCPP_INFO(_node.get_logger(), "Relative velocity Y: %f", relative_velocity.y());
		//RCLCPP_INFO_STREAM(_node.get_logger(), "Relative velocity: " << relative_velocity.transpose());

	} else {
		// Fallback to absolute velocity if target velocity not available (static target)
		//RCLCPP_INFO(_node.get_logger(), "breh");
		return false;
	}



	return (delta_pos_xy.norm() < _param_delta_position) && (relative_velocity.norm() < _param_delta_velocity);
}

std::string PrecisionLand::stateName(State state)
{
	switch (state) {
	case State::Idle:
		return "Idle";

	case State::Search:
		return "Search";

	case State::Approach:
		return "Approach";

	case State::Descend:
		return "Descend";

	case State::Finished:
		return "Finished";

	default:
		return "Unknown";
	}
}

void PrecisionLand::switchToState(State state)
{
	RCLCPP_INFO(_node.get_logger(), "Switching to %s", stateName(state).c_str());
	_state = state;
}

int main(int argc, char* argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<px4_ros2::NodeWithMode<PrecisionLand>>(kModeName, kEnableDebugOutput));
	rclcpp::shutdown();
	return 0;
}
