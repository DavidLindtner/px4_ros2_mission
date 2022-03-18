
#include <geometry_msgs/msg/quaternion.hpp>

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
		this->declare_parameter("x");
		this->declare_parameter("y");
		this->declare_parameter("z");
		this->declare_parameter("yaw");
		this->declare_parameter("offboardMode");
		this->declare_parameter("time_s");


		_offboardMode = this->get_parameter("offboardMode").as_int();

		_x = this->get_parameter("x").as_double_array();
		_y = this->get_parameter("y").as_double_array();
		_z = this->get_parameter("z").as_double_array();
		_yaw = this->get_parameter("yaw").as_double_array();

		_time_s = this->get_parameter("time_s").as_double_array();

		// check data format
		if(_x.size() == _y.size() && _x.size() == _z.size() && _x.size() == _yaw.size() && _x.size() == _time_s.size())
			_dataLength = _x.size();
		else
			RCLCPP_INFO(this->get_logger(), "WRONG DATA FORMAT - TERMINATING");

		// create publishers
		switch(_offboardMode)
		{
			case 1:
				_quaternion_pub = this->create_publisher<geometry_msgs::msg::Quaternion>("PositionQuat", 10);
				break;
			case 2:
				_quaternion_pub = this->create_publisher<geometry_msgs::msg::Quaternion>("VelocityQuat", 10);
				break;
			default:
				rclcpp::shutdown();
		}

		_absolute_time = _time_s[0];
		_timer = this->create_wall_timer(500ms, std::bind(&MissionSupervisor::cyclicTimer, this));
	}

private:

	void cyclicTimer()
	{
		auto message = geometry_msgs::msg::Quaternion();
		message.x = _x[_sample_counter];
		message.y = _y[_sample_counter];
		message.z = _z[_sample_counter];
		message.w = _yaw[_sample_counter];
		_quaternion_pub->publish(message);

		if(_cycle_time >= _absolute_time)
		{
			_sample_counter++;
			_absolute_time += _time_s[_sample_counter];
		}

		if(_sample_counter >= _dataLength)
			rclcpp::shutdown();

		_cycle_time += 0.5;
	}

	std::vector<double> _x;
	std::vector<double> _y;
	std::vector<double> _z;
	std::vector<double> _yaw;
	std::vector<double> _time_s;

	int _offboardMode = 0;

	double _cycle_time = 0;
	double _absolute_time = 0;
	int _sample_counter = 0;
	int _dataLength = 0;

	rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr _quaternion_pub;

	rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char** argv)
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<MissionSupervisor>());
	rclcpp::shutdown();
	return 0;
}

