#ifndef __DRONE_BASE_MAVLINK_HPP__
#define __DRONE_BASE_MAVLINK_HPP__

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geographic_msgs/msg/geo_pose_stamped.hpp>

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/altitude.hpp>

#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/param_set_v2.hpp>
#include <mavros_msgs/srv/param_pull.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <rclcpp/rclcpp.hpp>
#include <stdint.h>

#include <math.h>
#include <chrono>
#include <iostream>
#include <cstdlib>
#include <memory>

using namespace std::chrono;
using namespace std::chrono_literals;

class DroneMavlink : public rclcpp::Node
{
public:
	DroneMavlink();

	enum class FlightMode
	{ mOffboard, mTakeOff, mLand, mReturnToLaunch, mHold, mMission };

	struct
	{
		float lat;
		float lon;
		float alt;
	}lastGlobalSetpoint;

	mavros_msgs::msg::State currentState;
	sensor_msgs::msg::NavSatFix gpsPos;
	geometry_msgs::msg::PoseStamped locPos;
	float altitude = 0;


	bool preFlightCheckOK = false;
	bool paramPullOk = false;

	std::string vehicleName;

    void arm();
	void disarm();
    void setFlightMode(FlightMode mode);
    void preFlightCheck(float takeOffAlt, float maxHorSpeed);
    void pullParam();

	void publish_traj_setp_position(float x, float y, float z, float yaw);
	void publish_traj_setp_speed(float vx, float vy, float vz, float yawspeed);
	void publish_traj_setp_geo(float lat, float lon, float alt, bool heading);

	bool isGlSetpReached();
	float azimutToSetp();

private:
	void changeParam(std::string name, int type, int intVal, float floatVal);

	std::atomic<uint64_t> _timestamp;

	// MAVROS SUB AND PUB AND CLI
	rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr _state_sub;
	rclcpp::Subscription<mavros_msgs::msg::Altitude>::SharedPtr _alt_sub;
	rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr _gps_sub;
	rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _loc_pose_sub;

	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr _pos_setp_pub;
	rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr _vel_setp_pub;
	rclcpp::Publisher<geographic_msgs::msg::GeoPoseStamped>::SharedPtr _geo_setp_pub;

	rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr _cmd_cli;
	rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr _mode_cli;
	rclcpp::Client<mavros_msgs::srv::ParamSetV2>::SharedPtr _param_cli;
	rclcpp::Client<mavros_msgs::srv::ParamPull>::SharedPtr _param_req_cli;

	std::string _flightMode;
	int _noChangedParams = 0;
};


#endif /*__DRONE_BASE_MAVLINK_HPP__*/