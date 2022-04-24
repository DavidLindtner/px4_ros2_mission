#ifndef __DRONE_SIMPLE_HPP__
#define __DRONE_SIMPLE_HPP__

#include "../base/droneBaseRTPS.hpp"
#include "../base/droneBaseMavlink.hpp"

class DroneSimple : public DroneMavlink
{
public:
	DroneSimple();

/* STATE:
00 -> Wait 1 sec
10 -> PreFlightCheck
20 -> Wait for TakeOff start
30 -> TakeOff mode set
40 -> Arm command send
50 -> Wait for TakeOff end
60 -> Offboard mode send
70 -> Wait for data
80 -> Fly to waypoints
90 -> Land mode send
*/
	int state = 0;
	int stateOld = 0;
	uint64_t stateCounter = 0;
	uint64_t programCounter = 0;

	float holdLat;
	float holdLon;
	float holdAlt;

private:
	void flight_mode_timer_callback();
	rclcpp::TimerBase::SharedPtr _timer;
};


#endif /*__DRONE_SIMPLE_HPP__*/