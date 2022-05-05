#include "objectTrackingDrone.hpp"

ObjectTrackingDrone::ObjectTrackingDrone() : DroneMavlink()
{
	_geo_point_sub = this->create_subscription<geographic_msgs::msg::GeoPoint>(
										"/estimated_source_location", 
										1, 
										[this](geographic_msgs::msg::GeoPoint::ConstSharedPtr msg) {
											setpLat.store(msg->latitude);
											setpLon.store(msg->longitude);

											_point_sub_started = true;
											_timerActive->cancel();
											_timerActive = this->create_wall_timer(10000ms, std::bind(&ObjectTrackingDrone::timerActiveCallback, this));
										});

	_timerActive = this->create_wall_timer(60000ms, std::bind(&ObjectTrackingDrone::timerActiveCallback, this));
	_timer = this->create_wall_timer(100ms, std::bind(&ObjectTrackingDrone::mainLoopCallback, this));
}


void ObjectTrackingDrone::timerActiveCallback() { _point_sub_stopped = true; }


void ObjectTrackingDrone::mainLoopCallback()
{
	// SWITCHING BETWEEN STATES
	switch(state)
	{
		case 0:
			// Wait 1 sec
			if(stateCounter >= 10)
				state = 10;
			break;

		case 10:
			// Parameters pull
			if(this->paramPullOk)
				state = 11;
			break;

		case 11:
			// PreFlightCheck
			if(this->preFlightCheckOK)
				state = 20;
			break;

		case 20:
			// Wait for TakeOff start
			if(stateCounter == 10)
				state = 30;
			break;

		case 30:
			// TakeOff flight mode command send
			if(currentState.mode == "AUTO.TAKEOFF")
				state = 40;
			break;

		case 40:
			// Arm command send
			if(currentState.armed)
				state = 50;
			break;

		case 50:
			// Wait for TakeOff end
			if(currentState.mode == "AUTO.LOITER")
				state = 60;
			break;

		case 60:
			// Offboard flight mode command send
			if(currentState.mode == "OFFBOARD")
				state = 70;
			break;

		case 70:
			// Wait for data
			if(_point_sub_started || _point_sub_stopped)
				state = 80;
			break;

		case 80:
			// Fly to waypoints
			if(_point_sub_stopped)
				state = 90;
			break;

		case 90:
			// Offboard hold mode
			break;

		case 100:
			this->setFlightMode(FlightMode::mLand);
			break;
	}


	// BEHAVIOUR OF STATES
	switch(state)
	{
		case 0:
			// Wait 1 sec
			break;

		case 10:
			// Parameters pull
			if(stateCounter == 1)
				this->pullParam();
			break;

		case 11:
			// PreFlightCheck
			if(stateCounter == 1)
				this->preFlightCheck();
			break;

		case 20:
			// Wait for TakeOff start
			break;

		case 30:
			// TakeOff flight mode command send
			if(stateCounter == 1)
				this->setFlightMode(FlightMode::mTakeOff);
			break;

		case 40:
			// Arm command send
			if(stateCounter == 1)
				this->arm();
			break;

		case 50:
			// Wait for TakeOff end
			break;

		case 60:
			// Offboard flight mode command send
			if(stateCounter == 1)
				this->setFlightMode(FlightMode::mOffboard);
			break;

		case 70:
			// Wait for data
			if(stateCounter == 1)
				RCLCPP_INFO(this->get_logger(), "Waiting for data ...");
			break;

		case 80:
			// Fly to waypoints
			if(stateCounter == 1)
				RCLCPP_INFO(this->get_logger(), "Flying to waypoints");
			break;

		case 90:
			// Offboard hold mode
			if(stateCounter == 1)
				RCLCPP_INFO(this->get_logger(), "End of the mission - waiting ...");
			break;

		case 100:
			if(stateCounter == 1)
				this->setFlightMode(FlightMode::mLand);
			break;
	}

	if(state == 50 || state == 60)
	{
		holdLat = gpsPos.latitude;
		holdLon = gpsPos.longitude;
		holdAlt = altitude;
	}
	else if (state == 80)
	{
		holdLat = gpsPos.latitude;
		holdLon = gpsPos.longitude;
	}
	

	// OFFBOARD SETPOINTS SEND
	if(state <= 70)
		this->publish_traj_setp_geo(holdLat, holdLon, holdAlt, false);
	else if (state == 80)
		this->publish_traj_setp_geo(setpLat.load(), setpLon.load(), holdAlt, true);
	else if (state >= 90)
		this->publish_traj_setp_geo(holdLat, holdLon, holdAlt, false);

	//this->isGlSetpReached();

	// RESET STATE COUNTER
	if(state != stateOld)
		stateCounter = 0;

	stateOld = state;

	stateCounter++;
	programCounter++;

}
