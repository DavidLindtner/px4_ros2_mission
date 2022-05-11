#ifndef __DRONE_SUPERVISED_HPP__
#define __DRONE_SUPERVISED_HPP__

#include "../base/droneBaseRTPS.hpp"
#include "../base/droneBaseMavlink.hpp"

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geographic_msgs/msg/geo_point.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

class DroneSupervised : public DroneMavlink
{
public:
	DroneSupervised();

	int state = 0;
	int stateOld = 0;
	uint64_t stateCounter = 0;
	uint64_t programCounter = 0;

	float takeOffAlt = 10.0;

private:

	float _x;
	float _y;
	float _z;
	float _yaw;

	bool _relPosPub_Active = false;
	bool _velPub_Active = false;
	bool _absPosPub_Active = false;

	bool _resPosPub_stopped = false;
	bool _velPub_stopped = false;
	bool _absPosPub_stopped = false;

	float holdLat;
	float holdLon;
	float holdAlt;
	
	rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr _pos_sub;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _vel_sub;
	rclcpp::Subscription<geographic_msgs::msg::GeoPoint>::SharedPtr _geo_pos_sub;

	void timerCallback();

	void timerActiveCallback1();
	void timerActiveCallback2();
	void timerActiveCallback3();
	
	rclcpp::TimerBase::SharedPtr _timer;
	rclcpp::TimerBase::SharedPtr _timerActive1;
	rclcpp::TimerBase::SharedPtr _timerActive2;
	rclcpp::TimerBase::SharedPtr _timerActive3;
};


#endif /*__DRONE_SUPERVISED_HPP__*/