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
			// Wait for TakeOff start for 1 sec
			if(programCounter == 10)
				state = 10;
			break;

		case 10:
			// PreFlightCheck
			if(this->preFlightCheckOK)
				state = 20;
			break;

		case 20:
			// TakeOff flight mode command send
			if(currentState.mode == "AUTO.TAKEOFF")
				state = 30;
			break;

		case 30:
			// Arm command send
			if(currentState.armed)
				state = 40;
			break;

		case 40:
			// Wait for TakeOff end
			if(currentState.mode == "AUTO.LOITER")
				state = 50;
			break;

		case 50:
			// Offboard flight mode command send
			if(currentState.mode == "OFFBOARD")
				state = 60;
			break;

		case 60:
			// Wait for data
			break;

		case 70:
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
			// Wait for TakeOff start
			break;

		case 10:
			// PreFlightCheck
			if(stateCounter == 1)
				this->preFlightCheck(10.0);
			break;

		case 20:
			// TakeOff flight mode command send
			if(stateCounter == 1)
				this->setFlightMode(FlightMode::mTakeOff);
			break;

		case 30:
			// Arm command send
			if(stateCounter == 1)
				this->arm();
			break;

		case 40:
			// Wait for TakeOff end
			break;

		case 50:
			// Offboard flight mode command send
			if(stateCounter == 1)
				this->setFlightMode(FlightMode::mOffboard);
			break;

		case 60:
			// Wait for data
			break;

		case 70:
			// Fly to waypoints
			break;

		case 90:
			if(stateCounter == 1)
				this->setFlightMode(FlightMode::mLand);
			break;
	}

	// OFFBOARD SETPOINTS SEND
	if(state <= 60)
		this->publish_traj_setp_speed(0.0, 0.0, 0.0, 0.0);
	else if(state >= 70)
		this->publish_traj_setp_geo(49.228754, 16.573077, 298);


	// RESET STATE COUNTER
	if(state != stateOld)
		stateCounter = 0;

	stateOld = state;

	stateCounter++;
	programCounter++;

}
