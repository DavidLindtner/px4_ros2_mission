#ifndef __DRONE_SIMPLE_HPP__
#define __DRONE_SIMPLE_HPP__

#include "../base/droneBaseRTPS.hpp"
#include "../base/droneBaseMavlink.hpp"

class DroneSimple : public DroneMavlink
{
public:
	DroneSimple();

/* STATE:
00 -> PreFlightCheck
10 -> Wait for TakeOff start
20 -> TakeOff mode set
30 -> Arm command send
40 -> Wait for TakeOff end
50 -> Offboard mode send
60 -> Wait for data
70 -> Fly to waypoints
90 -> Land mode send
*/
	int state = 0;
	int stateOld = 0;
	uint64_t stateCounter = 0;
	uint64_t programCounter;

private:
	void flight_mode_timer_callback();
	rclcpp::TimerBase::SharedPtr _timer;
};


#endif /*__DRONE_SIMPLE_HPP__*/