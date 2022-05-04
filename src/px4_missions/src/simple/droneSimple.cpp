#include "droneSimple.hpp"

DroneSimple::DroneSimple() : DroneMavlink()
{
	_timer = this->create_wall_timer(100ms, std::bind(&DroneSimple::flight_mode_timer_callback, this));
}


void DroneSimple::flight_mode_timer_callback()
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
			if(stateCounter == 100)
				state = 80;
			break;

		case 80:
			// Fly to waypoints
			break;

		case 90:
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
				this->preFlightCheck(10.0, 10.0);
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
			break;

		case 80:
			// Fly to waypoints
			break;

		case 90:
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

	// OFFBOARD SETPOINTS SEND
	if(state <= 70)
		this->publish_traj_setp_geo(holdLat, holdLon, holdAlt, false);
	else if(state >= 80)
		this->publish_traj_setp_geo(49.228754, 16.572077, 300, true);

	//RCLCPP_INFO(this->get_logger(), "Sme Tam %d", this->isGlSetpReached());

	// RESET STATE COUNTER
	if(state != stateOld)
		stateCounter = 0;

	stateOld = state;

	stateCounter++;
	programCounter++;

}
