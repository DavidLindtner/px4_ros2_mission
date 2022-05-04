#ifndef __OBJECT_TRACKING_DRONE_HPP__
#define __OBJECT_TRACKING_DRONE_HPP__

#include "../base/droneBaseRTPS.hpp"
#include "../base/droneBaseMavlink.hpp"

#include <geometry_msgs/msg/point.hpp>

class ObjectTrackingDrone : public DroneMavlink
{
public:
	ObjectTrackingDrone();

/* STATE:
00 -> Wait 1 sec
10 -> Parameters pull
11 -> PreFlightCheck
20 -> Wait for TakeOff start
30 -> TakeOff mode set
40 -> Arm command send
50 -> Wait for TakeOff end
60 -> Offboard mode send
70 -> Wait for data - offboard hold
80 -> Fly to waypoints
90 -> Offboard hold mode
100 -> Land mode send
*/
	int state = 0;
	int stateOld = 0;
	uint64_t stateCounter = 0;
	uint64_t programCounter = 0;

	float takeOffAlt = 10.0;

	float holdLat;
	float holdLon;
	float holdAlt;

	std::atomic<float> setpLat;
	std::atomic<float> setpLon;

private:
	void mainLoopCallback();
	void timerActiveCallback();

	rclcpp::TimerBase::SharedPtr _timer;
	rclcpp::TimerBase::SharedPtr _timerActive;

	rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr _point_sub;

	bool _point_sub_stopped = false;
	bool _point_sub_started = false;
};


#endif /*__OBJECT_TRACKING_DRONE_HPP__*/