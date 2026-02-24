#pragma once

#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_land_detected.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/gimbal_device_attitude_status.hpp>
#include <px4_ros2/control/setpoint_types/experimental/trajectory.hpp>
#include <px4_ros2/utils/message_version.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <vector>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <nav_msgs/msg/odometry.hpp> // Include the message type
#include <std_msgs/msg/bool.hpp>

class PrecisionLand : public px4_ros2::ModeBase
{
public:
	explicit PrecisionLand(rclcpp::Node& node);

	void targetPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
	void smallMarkerDetectedCallback(const std_msgs::msg::Bool::SharedPtr msg);
	void commandReposition(double lat, double lon, float alt);
	void vehicleLandDetectedCallback(const px4_msgs::msg::VehicleLandDetected::SharedPtr msg);
	void gimbalAttitudeCallback(const px4_msgs::msg::GimbalDeviceAttitudeStatus::SharedPtr msg);
	void wamvOdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
	// See ModeBasep
	void onActivate() override;
	void onDeactivate() override;
	void updateSetpoint(float dt_s) override;

private:
	// Helper struct for map projection
	/*struct MapProjectionReference {
		double lat_rad;
		double lon_rad;
		double sin_lat;
		double cos_lat;
		bool init = false;
	};

	MapProjectionReference _map_ref;

	// Add method declaration
	void map_projection_init(MapProjectionReference& ref, double lat_0, double lon_0);
	void map_projection_project(const MapProjectionReference& ref, double lat, double lon, float& x, float& y);*/
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _wamv_odom_sub;
    
    // Store the boat's position
    float _boat_x{0.0f};
    float _boat_y{0.0f};
    bool _boat_found{false};
	struct ArucoTag {
		// Initialize position with NaN values directly in the struct
		Eigen::Vector3d position = Eigen::Vector3d::Constant(std::numeric_limits<double>::quiet_NaN());
		Eigen::Quaterniond orientation;
		rclcpp::Time timestamp;

		bool valid() { return timestamp.nanoseconds() > 0; };
	};

	void loadParameters();

	ArucoTag getTagWorld(const ArucoTag& tag);

	Eigen::Vector2f calculateVelocitySetpointXY();
	float checkTargetTimeout();
	bool positionReached(const Eigen::Vector3f& target) const;

	enum class State {
		Idle,
		Search, 	// Searches for target using a search pattern
		Approach, 	// Positioning over landing target while maintaining altitude
		Descend, 	// Stay over landing target while descending
		Finished
	};

	void switchToState(State state);
	std::string stateName(State state);

	// ros2
	rclcpp::Node& _node;
	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr _wamv_gps_sub;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_sub;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _small_marker_detected_sub;
	rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr _vehicle_land_detected_sub;
	rclcpp::Subscription<px4_msgs::msg::GimbalDeviceAttitudeStatus>::SharedPtr _gimbal_attitude_sub;

	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _target_pose_world_pub;
	rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_command_pub;

	// px4_ros2_cpp
	std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
	std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
	std::shared_ptr<px4_ros2::TrajectorySetpointType> _trajectory_setpoint;

	// Data
	State _state = State::Search;
	bool _search_started = false;

	ArucoTag _tag;
	float _approach_altitude = {};

	Eigen::Quaterniond _gimbal_orientation = Eigen::Quaterniond::Identity();

	// Land detection
	bool _land_detected = false;
	bool _ground_contact = false;
	bool _disarmed_on_contact = false;
	bool _target_lost_prev = true;

	// Small marker detection tracking
	bool _small_marker_detected = false;
	bool _small_marker_detected_prev = false;

	// Target velocity tracking for moving target support
	Eigen::Vector2d _target_velocity = Eigen::Vector2d::Zero();
	Eigen::Vector2d _prev_tag_position = Eigen::Vector2d::Zero();
	rclcpp::Time _prev_tag_timestamp;
	bool _target_velocity_valid = false;

	// Waypoints for Search pattern
	std::vector<Eigen::Vector3f> _search_waypoints;
	// Search pattern generation
	void generateSearchWaypoints();
	// Search pattern index
	int _search_waypoint_index = 0;

	// Parameters
	float _param_descent_vel = {};
	float _param_vel_p_gain = {};
	float _param_vel_i_gain = {};
	float _param_max_velocity = {};
	float _param_target_timeout = {};
	float _param_delta_position = {};
	float _param_delta_velocity = {};
	float _param_velocity_filter_alpha = {};

	float _vel_x_integral {};
	float _vel_y_integral {};

	// Last velocity before marker goes out of view
	Eigen::Vector2f _last_velocity = Eigen::Vector2f::Zero();
	bool _last_velocity_valid = false;


    // Data storage
	double _wamv_lat {};
	double _wamv_lon {};
    
    // Callback declarations
    void wamvGpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
};
