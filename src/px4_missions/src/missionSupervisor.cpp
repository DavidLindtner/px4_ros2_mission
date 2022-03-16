
#include <missions_interfaces/msg/rel_position.hpp>

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
      this->declare_parameter("time_s");

      _pos_X = this->get_parameter("posX_m").as_double_array();
      _pos_Y = this->get_parameter("posY_m").as_double_array();
      _pos_Z = this->get_parameter("posZ_m").as_double_array();
      _yaw_rad = this->get_parameter("yaw_rad").as_double_array();
      _time_s = this->get_parameter("time_s").as_double_array();

      if(_pos_X.size() == _pos_Y.size() && _pos_X.size() == _pos_Z.size() && _pos_X.size() == _yaw_rad.size() && _pos_X.size() == _time_s.size())
        _dataLength = _pos_X.size();
      else
        RCLCPP_INFO(this->get_logger(), "WRONG DATA FORMAT - TERMINATING");

      _absolute_time = _time_s[0];
      _publisher = this->create_publisher<missions_interfaces::msg::RelPosition>("relPosition", 10); 
      _timer = this->create_wall_timer(500ms, std::bind(&MissionSupervisor::cyclicTimer, this));
    }


private:

    void cyclicTimer()
    {
        auto message = missions_interfaces::msg::RelPosition();
        message.x = _pos_X[_sample_counter];
        message.y = _pos_Y[_sample_counter];
        message.z = _pos_Z[_sample_counter];
        message.yaw = _yaw_rad[_sample_counter];
        //RCLCPP_INFO(this->get_logger(), "Publishing: %f, %f, %f, %f, %d, %f", message.x, message.y, message.z, message.yaw, _sample_counter, _cycle_time);
        _publisher->publish(message);

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
    std::vector<double> _yaw_rad;
    std::vector<double> _time_s;

    double _cycle_time = 0;
    double _absolute_time = 0;
    int _sample_counter = 0;
    int _dataLength = 0;

    rclcpp::Publisher<missions_interfaces::msg::RelPosition>::SharedPtr _publisher;
    rclcpp::TimerBase::SharedPtr _timer;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MissionSupervisor>());
  rclcpp::shutdown();
  return 0;
}

