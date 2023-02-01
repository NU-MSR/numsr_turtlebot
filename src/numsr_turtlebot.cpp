#include "numsr_turtlebot/dynamixel_sdk_wrapper.hpp"
#include "rclcpp/rclcpp.hpp"

#include "numsr_turtlebot/dynamixel_sdk_wrapper.hpp"
#include "rclcpp/rclcpp.hpp"

class NuTurtlebot : public rclcpp::Node
{
public:
    NuMSRTurtlebot():
        Node("numsr_turtlebot"),
        opencr{"/dev/ttyACM0", 200, 1000000, 2.0f},
        dxl_sdk_wrapper{std::make_shared<DynamixelSDKWrapper>(opencr)}
    {
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

        dxl_sdk_wrapper_->set_data_to_device(
            extern_control_table.imu_re_calibration.addr,
            extern_control_table.imu_re_calibration.length,
            &reset,
            &sdk_msg);

        RCLCPP_INFO_STREAM(get_logger(), "Start Calibration of Gyro");
        rclcpp::sleep_for(std::chrono::seconds(5));
        RCLCPP_INFO_STREAM(get_logger(), "Calibration End");
    }
private:
    DynamixelSDKWrapper::Device opencr;
    std::shared_ptr<DynamixelSDKWrapper> dxl_sdk_wrapper;
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NuTurtlebot>());
    rclcpp::shutdown();
    return 0;
}
