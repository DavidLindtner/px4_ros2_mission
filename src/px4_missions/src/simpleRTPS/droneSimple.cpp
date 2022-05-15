#include "droneSimple.hpp"

DroneSimple::DroneSimple() : DroneRTPS()
{
	_timer = this->create_wall_timer(100ms, std::bind(&DroneSimple::flight_mode_timer_callback, this));
}


void DroneSimple::flight_mode_timer_callback()
{
	// SWITCHING BETWEEN STATES
	switch(state)
	{
		case 0:
			if(stateCounter >= 50)
				state = 10;
			break;

		case 10:
			if(stateCounter >= 10)
				state = 20;
			break;

		case 20:
			if(stateCounter >= 10)
				state = 30;
			break;

		case 30:
			if(stateCounter >= 100)
				state = 40;
			break;

		case 40:
			if(stateCounter >= 10)
				state = 50;
			break;

		case 50:
			if(stateCounter >= 200)
				state = 60;
			break;
	}


	// BEHAVIOUR OF STATES
	switch(state)
	{
		case 0:
			break;

		case 10:
			// TakeOff flight mode command send
			if(stateCounter == 1)
				this->setFlightMode(FlightMode::mTakeOff);
			break;

		case 20:
			// Arm command send
			if(stateCounter == 1)
				this->arm();
			break;

		case 30:
			break;

		case 40:
			// Offboard flight mode command send
			if(stateCounter == 1)
				this->setFlightMode(FlightMode::mOffboard);
			break;

		case 50:
			// Wait for TakeOff end
			break;

		case 60:
			if(stateCounter == 1)
				this->setFlightMode(FlightMode::mLand);
			break;
	}


	// OFFBOARD SETPOINTS SEND
	publish_offboard_control_mode(OffboardControl::oRelPos);
	publish_traj_setp_position(10.0, 0.0, -40.0, 1.0);


	// RESET STATE COUNTER
	if(state != stateOld)
		stateCounter = 0;

	stateOld = state;

	stateCounter++;
	programCounter++;

}
