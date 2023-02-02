// This code is loosely based on code from ROBOTIS turtlebot3_node
#include "numsr_turtlebot/dynamixel_sdk_wrapper.hpp"
#include "numsr_turtlebot/control_table.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nuturtlebot_msgs/msg/wheel_commands.hpp"
#include "nuturtlebot_msgs/msg/sensor_data.hpp"

using robotis::turtlebot3::DynamixelSDKWrapper;
using robotis::turtlebot3::extern_control_table;

using namespace std::chrono_literals;
using std::placeholders::_1;

class NuTurtlebot : public rclcpp::Node
{
public:
    NuTurtlebot():
        Node("numsr_turtlebot"),
        opencr{"/dev/ttyACM0", 200, 1000000, 2.0f},
        dxl_sdk_wrapper{std::make_shared<DynamixelSDKWrapper>(opencr)}
    {
        // initialize the dynamixel sdk and connect
        dxl_sdk_wrapper->init_read_memory(
            extern_control_table.millis.addr,
            (extern_control_table.profile_acceleration_right.addr - extern_control_table.millis.addr) +
            extern_control_table.profile_acceleration_right.length);

        if(!dxl_sdk_wrapper->is_connected_to_device())
        {
            throw std::runtime_error("Failed To Connect");
        }
        std::string sdk_msg;
        uint8_t reset = 1;

        dxl_sdk_wrapper->set_data_to_device(
            extern_control_table.imu_re_calibration.addr,
            extern_control_table.imu_re_calibration.length,
            &reset,
            &sdk_msg);

        RCLCPP_INFO_STREAM(get_logger(), "Start Calibration of Gyro");
        rclcpp::sleep_for(std::chrono::seconds(5));
        RCLCPP_INFO_STREAM(get_logger(), "Calibration End");

        if(-1 == dxl_sdk_wrapper->get_data_from_device<int8_t>(
                extern_control_table.device_status.addr,
                extern_control_table.device_status.length)
            )
        {
            throw std::runtime_error("Failed to connect to motors");
        }

        subscriber = create_subscription<nuturtlebot_msgs::msg::WheelCommands>("wheel_cmd", 10, std::bind(&NuTurtlebot::wheel_cmd_callback, this, _1));
        publisher = create_publisher<nuturtlebot_msgs::msg::SensorData>("sensor_data", 10);
        timer = create_wall_timer(100ms,
                                  std::bind(&NuTurtlebot::timer_callback, this));
    }

    void timer_callback()
    {
        nuturtlebot_msgs::msg::SensorData sensor_data;
        sensor_data.stamp = get_clock()->now();

        dxl_sdk_wrapper->read_data_set();

        // read wheel encoders
        std::array<int32_t, 2> position =
            {dxl_sdk_wrapper->get_data_from_device<int32_t>(
                    extern_control_table.present_position_left.addr,
                    extern_control_table.present_position_left.length),
             dxl_sdk_wrapper->get_data_from_device<int32_t>(
                 extern_control_table.present_position_right.addr,
                 extern_control_table.present_position_right.length)};
        sensor_data.left_encoder = position.at(0);
        sensor_data.right_encoder = position.at(1);

        publisher->publish(sensor_data);

    }

    void wheel_cmd_callback(const nuturtlebot_msgs::msg::WheelCommands & wheel_cmd)
    {
    }

private:
    DynamixelSDKWrapper::Device opencr;
    std::shared_ptr<DynamixelSDKWrapper> dxl_sdk_wrapper;
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<nuturtlebot_msgs::msg::SensorData>::SharedPtr publisher;
    rclcpp::Subscription<nuturtlebot_msgs::msg::WheelCommands>::SharedPtr subscriber;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NuTurtlebot>());
    rclcpp::shutdown();
    return 0;
}
