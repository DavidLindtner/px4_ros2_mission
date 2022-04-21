#include "droneBaseMavlink.hpp"

DroneMavlink::DroneMavlink() : Node("DroneMavlink")
{
	_pos_setp_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/setpoint_position/local", 10);
	_vel_setp_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
	_geo_setp_pub = this->create_publisher<geographic_msgs::msg::GeoPoseStamped>("/mavros/setpoint_position/global", 10);

	_state_sub = this->create_subscription<mavros_msgs::msg::State>(
										"/mavros/state", 
										10, 
										[this](mavros_msgs::msg::State::ConstSharedPtr msg) {
											currentState = *msg;
										});

	// NOT YET WORKING
	_gps_sub = this->create_subscription<sensor_msgs::msg::NavSatFix>(
										"/mavros/global_position/global", 
										1000, 
										[this](sensor_msgs::msg::NavSatFix::ConstSharedPtr msg) {
											std::cout << "sme tu" << std::endl;
											gpsPos = *msg;
											std::cout << msg->latitude << std::endl;
										});

	_cmd_cli = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
	_mode_cli = this->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    
	//this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_PARAMETER, 175, 4);
}


void DroneMavlink::setFlightMode(FlightMode mode)
{
	while (!_cmd_cli->wait_for_service(1s))
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
			RCLCPP_INFO(this->get_logger(), "Offboard flight mode set");
			break;

		case FlightMode::mTakeOff:
			request->base_mode = 0;
			request->custom_mode = "AUTO.TAKEOFF";
			RCLCPP_INFO(this->get_logger(), "TakeOff flight mode set");
			break;
			
		case FlightMode::mLand:
			request->base_mode = 0;
			request->custom_mode = "AUTO.LAND";
			RCLCPP_INFO(this->get_logger(), "Land flight mode set");
			break;
			
		case FlightMode::mReturnToLaunch:
			request->base_mode = 0;
			request->custom_mode = "AUTO.RTL";
			RCLCPP_INFO(this->get_logger(), "Return to Launch flight mode set");
			break;

		case FlightMode::mHold:
			request->base_mode = 0;
			request->custom_mode = "AUTO.LOITER";
			RCLCPP_INFO(this->get_logger(), "Hold flight mode set");
			break;

		// NOT YET WORKING
		case FlightMode::mMission:
			request->base_mode = 0;
			request->custom_mode = "AUTO.MISSION";
			RCLCPP_INFO(this->get_logger(), "Mission flight mode set");
			break;
			
		default:
			RCLCPP_INFO(this->get_logger(), "No flight mode set");
	}

	auto result = _mode_cli->async_send_request(request);
}


void DroneMavlink::arm()
{
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
	auto result = _cmd_cli->async_send_request(request);

	RCLCPP_INFO(this->get_logger(), "Arm command send");
}


void DroneMavlink::disarm()
{
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
	auto result = _cmd_cli->async_send_request(request);

	RCLCPP_INFO(this->get_logger(), "Disarm command send");
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
}

