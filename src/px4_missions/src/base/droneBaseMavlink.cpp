#include "droneBaseMavlink.hpp"

DroneMavlink::DroneMavlink() : Node("DroneMavlink")
{
	_pos_setp_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", 1);
	_vel_setp_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 1);
	_geo_setp_pub = this->create_publisher<geographic_msgs::msg::GeoPoseStamped>("/mavros/setpoint_position/global", 1);

	_state_sub = this->create_subscription<mavros_msgs::msg::State>(
										"/mavros/state", 
										1, 
										[this](mavros_msgs::msg::State::ConstSharedPtr msg) {
											currentState = *msg;
										});

	_alt_sub = this->create_subscription<mavros_msgs::msg::Altitude>(
										"/mavros/altitude", 
										rclcpp::SensorDataQoS(), 
										[this](mavros_msgs::msg::Altitude::ConstSharedPtr msg) {
											altitude = msg->amsl;
										});


	_gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
										"/mavros/global_position/global", 
										rclcpp::SensorDataQoS(), 
										[this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
											gpsPos = *msg;
										});


	_loc_pose_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>(
										"/mavros/local_position/pose", 
										rclcpp::SensorDataQoS(),
										[this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
											locPos = *msg;
										});



	_cmd_cli = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
	_mode_cli = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
	_param_cli = this->create_client<mavros_msgs::srv::ParamSetV2>("/mavros/param/set");

}


void DroneMavlink::preFlightCheck(float takeOffAlt)
{
	changeParam("MIS_TAKEOFF_ALT", 3, 0, takeOffAlt);
	RCLCPP_INFO(this->get_logger(), "TakeOff height set to %f", takeOffAlt);

	changeParam("COM_RCL_EXCEPT", 2, 4, 0);
	RCLCPP_INFO(this->get_logger(), "RC loss exceptions is set");

	changeParam("COM_OBL_ACT", 2, 1, 0);
	RCLCPP_INFO(this->get_logger(), "Failsafe action after Offboard mode lost");

	preFlightCheckOK = true;
}


void DroneMavlink::changeParam(std::string name, int type, int intVal, float floatVal)
{
	// check if services exist
	while (!_param_cli->wait_for_service(1s))
	{
		if (!rclcpp::ok())
		{
			RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
			exit(1);
		}
		RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
	}

  	auto request = std::make_shared<mavros_msgs::srv::ParamSetV2::Request>();

	request->param_id = name;
	request->value.type = type;
	request->value.integer_value = intVal;
	request->value.double_value = floatVal;

	// wait for result
    using ServiceResponseFuture = rclcpp::Client<mavros_msgs::srv::ParamSetV2>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future)
	{
		auto result = future.get();
		if(!result->success)
			RCLCPP_ERROR(this->get_logger(), "ERROR SETTING PARAMETER");
    };
    auto future_result = _param_cli->async_send_request(request, response_received_callback);
}


void DroneMavlink::setFlightMode(FlightMode mode)
{
	// check if services exist
	while (!_mode_cli->wait_for_service(1s))
	{
		if (!rclcpp::ok())
		{
			RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
			exit(1);
		}
		RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
	}
			
	auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();

	switch (mode)
	{
		case FlightMode::mOffboard:
			request->base_mode = 0;
			request->custom_mode = "OFFBOARD";
			_flightMode = "OFFBOARD";
			break;

		case FlightMode::mTakeOff:
			request->base_mode = 0;
			request->custom_mode = "AUTO.TAKEOFF";
			_flightMode = "TAKEOFF";
			break;
			
		case FlightMode::mLand:
			request->base_mode = 0;
			request->custom_mode = "AUTO.LAND";
			_flightMode = "LAND";
			break;
			
		case FlightMode::mReturnToLaunch:
			request->base_mode = 0;
			request->custom_mode = "AUTO.RTL";
			_flightMode = "RTL";
			break;

		case FlightMode::mHold:
			request->base_mode = 0;
			request->custom_mode = "AUTO.LOITER";
			_flightMode = "HOLD";
			break;

		// NOT YET WORKING
		case FlightMode::mMission:
			request->base_mode = 0;
			request->custom_mode = "AUTO.MISSION";
			_flightMode = "MISSION";
			break;
			
		default:
			RCLCPP_INFO(this->get_logger(), "No flight mode set");
	}

	// wait for result
    using ServiceResponseFuture = rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future)
	{
		auto result = future.get();
		if(result->mode_sent)
			RCLCPP_INFO(this->get_logger(), "%s flight mode set", _flightMode.c_str());
		else
			RCLCPP_ERROR(this->get_logger(), "FLIGHT MODE CHANGE ERROR");
    };
    auto future_result = _mode_cli->async_send_request(request, response_received_callback);

}


void DroneMavlink::arm()
{
	// check if services exist
	while (!_cmd_cli->wait_for_service(1s))
	{
		if (!rclcpp::ok())
		{
			RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
			exit(1);
		}
		RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
	}

  	auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
	request->value = true;

	// wait for result
    using ServiceResponseFuture = rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future)
	{
		auto result = future.get();
		if(result->success)
			RCLCPP_INFO(this->get_logger(), "Arm command send");
		else
			RCLCPP_ERROR(this->get_logger(), "ARMING ERROR");
    };
    auto future_result = _cmd_cli->async_send_request(request, response_received_callback);
}


void DroneMavlink::disarm()
{
	// check if services exist
	while (!_cmd_cli->wait_for_service(1s))
	{
		if (!rclcpp::ok())
		{
			RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
			exit(1);
		}
		RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
	}

  	auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
	request->value = false;

	// wait for result
    using ServiceResponseFuture = rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future)
	{
		auto result = future.get();
		if(result->success)
			RCLCPP_INFO(this->get_logger(), "Disarm command send");
		else
			RCLCPP_ERROR(this->get_logger(), "DISARMING ERROR");
    };
    auto future_result = _cmd_cli->async_send_request(request, response_received_callback);
}


void DroneMavlink::publish_traj_setp_position(float x, float y, float z, float yaw)
{
	geometry_msgs::msg::PoseStamped msg;
    msg.pose.position.x = x;
    msg.pose.position.y = y;
    msg.pose.position.z = z;
	msg.pose.orientation.z = yaw;
	_pos_setp_pub->publish(msg);
}


void DroneMavlink::publish_traj_setp_speed(float vx, float vy, float vz, float yawspeed)
{
	geometry_msgs::msg::TwistStamped msg;
    msg.twist.linear.x = vx;
    msg.twist.linear.y = vy;
    msg.twist.linear.z = vz;
	msg.twist.angular.z = yawspeed;
	_vel_setp_pub->publish(msg);
}

void DroneMavlink::publish_traj_setp_geo(float lat, float lon, float alt)
{
	geographic_msgs::msg::GeoPoseStamped msg;
    msg.pose.position.latitude = lat;
    msg.pose.position.longitude = lon;
    msg.pose.position.altitude = alt;
	_geo_setp_pub->publish(msg);

	lastGlobalSetpoint.lat = lat;
	lastGlobalSetpoint.lon = lon;
	lastGlobalSetpoint.alt = alt;
}


bool DroneMavlink::isGlSetpReached()
{
	float latSetpRad = 3.1415 * lastGlobalSetpoint.lat / 180;
	float latActRad = 3.1415 * gpsPos.latitude / 180;

	float latDiff = 3.1415 * (lastGlobalSetpoint.lat - gpsPos.latitude) / 180;
	float lonDiff = 3.1415 * (lastGlobalSetpoint.lon - gpsPos.longitude) / 180;

    double  a = sin(latDiff/2) * sin(latDiff/2) + cos(latSetpRad) * cos(latActRad) * sin(lonDiff/2) * sin(lonDiff/2);
    double  c = 2 * atan2(sqrt(a), sqrt(1-a));
    double  distance = 6372797.56085 * c;

	distance += abs(lastGlobalSetpoint.alt - altitude);
   
	if(distance <= 0.5)
		return true;
	else
		return false;
}