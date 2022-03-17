
#include <missions_interfaces/msg/rel_position.hpp>
#include <missions_interfaces/msg/velocity.hpp>

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <string>
#include <functional>
#include <memory>


using namespace std::chrono_literals;

class MissionSupervisor: public rclcpp::Node
{
public:
	MissionSupervisor() : Node("mission_supervisor")
	{
		this->declare_parameter("posX_m");
		this->declare_parameter("posY_m");
		this->declare_parameter("posZ_m");
		this->declare_parameter("yaw_rad");

		this->declare_parameter("velX_ms");
		this->declare_parameter("velY_ms");
		this->declare_parameter("velZ_ms");
		this->declare_parameter("yaw_rs");

		this->declare_parameter("offboardMode");
		this->declare_parameter("time_s");

		_offboardMode = this->get_parameter("offboardMode").as_int();
		_time_s = this->get_parameter("time_s").as_double_array();

		switch(_offboardMode)
		{
			case 1:
				_pos_X = this->get_parameter("posX_m").as_double_array();
				_pos_Y = this->get_parameter("posY_m").as_double_array();
				_pos_Z = this->get_parameter("posZ_m").as_double_array();
				_pos_yaw = this->get_parameter("yaw_rad").as_double_array();
				if(_pos_X.size() == _pos_Y.size() && _pos_X.size() == _pos_Z.size() && _pos_X.size() == _pos_yaw.size() && _pos_X.size() == _time_s.size())
					_dataLength = _pos_X.size();
				else
					RCLCPP_INFO(this->get_logger(), "WRONG DATA FORMAT - TERMINATING");
				_rel_pos_pub = this->create_publisher<missions_interfaces::msg::RelPosition>("relPosition", 10); 
				break;
			case 2:
				_vel_X = this->get_parameter("velX_ms").as_double_array();
				_vel_Y = this->get_parameter("velY_ms").as_double_array();
				_vel_Z = this->get_parameter("velZ_ms").as_double_array();
				_vel_yaw = this->get_parameter("yaw_rs").as_double_array();
				if(_vel_X.size() == _vel_Y.size() && _vel_X.size() == _vel_Z.size() && _vel_X.size() == _vel_yaw.size() && _vel_X.size() == _time_s.size())
					_dataLength = _vel_X.size();
				else
					RCLCPP_INFO(this->get_logger(), "WRONG DATA FORMAT - TERMINATING");
				_vel_pub = this->create_publisher<missions_interfaces::msg::Velocity>("Velocity", 10); 
				break;
			default:
				RCLCPP_INFO(this->get_logger(), "WRONG DATA FORMAT - TERMINATING");
				rclcpp::shutdown();
		}

		_absolute_time = _time_s[0];
		_timer = this->create_wall_timer(500ms, std::bind(&MissionSupervisor::cyclicTimer, this));
	}

private:

	void cyclicTimer()
	{
		if(_offboardMode == 1)
		{
			auto message = missions_interfaces::msg::RelPosition();
			message.x = _pos_X[_sample_counter];
			message.y = _pos_Y[_sample_counter];
			message.z = _pos_Z[_sample_counter];
			message.yaw = _pos_yaw[_sample_counter];
			_rel_pos_pub->publish(message);
		}
		else if(_offboardMode == 2)
		{
			auto message = missions_interfaces::msg::Velocity();
			message.vx = _vel_X[_sample_counter];
			message.vy = _vel_Y[_sample_counter];
			message.vz = _vel_Z[_sample_counter];
			message.yawspeed = _vel_yaw[_sample_counter];
			_vel_pub->publish(message);
		}
		else
		{
			RCLCPP_INFO(this->get_logger(), "WRONG DATA FORMAT - TERMINATING");
			rclcpp::shutdown();
		}

		if(_cycle_time >= _absolute_time)
		{
			_sample_counter++;
			_absolute_time += _time_s[_sample_counter];
		}

		if(_sample_counter >= _dataLength)
			rclcpp::shutdown();

		_cycle_time += 0.5;
	}

	std::vector<double> _pos_X;
	std::vector<double> _pos_Y;
	std::vector<double> _pos_Z;
	std::vector<double> _pos_yaw;

	std::vector<double> _vel_X;
	std::vector<double> _vel_Y;
	std::vector<double> _vel_Z;
	std::vector<double> _vel_yaw;

	std::vector<double> _time_s;

	int _offboardMode = 0;

	double _cycle_time = 0;
	double _absolute_time = 0;
	int _sample_counter = 0;
	int _dataLength = 0;

	rclcpp::Publisher<missions_interfaces::msg::RelPosition>::SharedPtr _rel_pos_pub;
	rclcpp::Publisher<missions_interfaces::msg::Velocity>::SharedPtr _vel_pub;
	rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MissionSupervisor>());
	rclcpp::shutdown();
	return 0;
}

