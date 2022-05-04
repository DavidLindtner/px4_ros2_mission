#include "droneSupervised.hpp"

DroneSupervised::DroneSupervised() : DroneMavlink()
{
	this->declare_parameter("takeOffHeight", 10.0);
	takeOffAlt = this->get_parameter("takeOffHeight").as_double();

	_pos_sub = this->create_subscription<geometry_msgs::msg::Pose>(
									vehicleName + "/PositionSetp",
									1,
									[this](const geometry_msgs::msg::Pose::ConstSharedPtr msg){
										_x = msg->position.x;
										_y = msg->position.y;
										_z = msg->position.z;
										tf2::Quaternion q;
										tf2::convert(msg->orientation, q);
										tf2::Matrix3x3 m(q);
										double roll, pitch, yaw;
										m.getRPY(roll, pitch, yaw);
										_yaw = yaw;
										_relPosPub_Active = true;
										_timerActive1->cancel();
										_timerActive1 = this->create_wall_timer(10000ms, std::bind(&DroneSupervised::timerActiveCallback1, this));
									});

	_vel_sub = this->create_subscription<geometry_msgs::msg::Twist>(
									vehicleName + "/VelocitySetp",
									1,
									[this](const geometry_msgs::msg::Twist::ConstSharedPtr msg){
										_x = msg->linear.x;
										_y = msg->linear.y;
										_z = -1*msg->linear.z;
										_yaw = msg->angular.z;

										_velPub_Active = true;
										_timerActive2->cancel();
										_timerActive2 = this->create_wall_timer(10000ms, std::bind(&DroneSupervised::timerActiveCallback2, this));
									});

	_timerActive1 = this->create_wall_timer(10000ms, std::bind(&DroneSupervised::timerActiveCallback1, this));
	_timerActive2 = this->create_wall_timer(10000ms, std::bind(&DroneSupervised::timerActiveCallback2, this));
	_timer = this->create_wall_timer(100ms, std::bind(&DroneSupervised::timerCallback, this));
}

void DroneSupervised::timerActiveCallback1() { _resPosPub_stopped = true; }
void DroneSupervised::timerActiveCallback2() { _velPub_stopped = true; }

void DroneSupervised::timerCallback()
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
			if(_velPub_Active || _relPosPub_Active)
				state = 80;
			break;

		case 80:
			// Fly to waypoints
			if((_velPub_Active  && _velPub_stopped) || (_relPosPub_Active && _resPosPub_stopped))
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
				this->preFlightCheck(takeOffAlt, 6.0);
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
	{
		if(_relPosPub_Active)
			this->publish_traj_setp_position(_x, _y, _z, _yaw);
		if(_velPub_Active)
			this->publish_traj_setp_speed(_x, _y, _z, _yaw);
	}		
	else if (state >= 90)
		this->publish_traj_setp_geo(holdLat, holdLon, holdAlt, false);


	// RESET STATE COUNTER
	if(state != stateOld)
		stateCounter = 0;

	stateOld = state;

	stateCounter++;
	programCounter++;
}
