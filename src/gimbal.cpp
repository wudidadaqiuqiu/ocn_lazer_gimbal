#include "rclcpp/rclcpp.hpp"
// #include "connector/include/ connector.hpp"
#include "connector/connector.hpp"
#include "common/protocol/serialized_protocol.hpp"
#include "robot_msg/msg/imu_raw.hpp"
#include "robot_msg/msg/imu_raw.h"

using connector::Connector;
using connector::ConnectorType;
using connector::BaudRate;
using connector::ConnectorSingleRecvNode;
using connector::ConnectorSendNode;
using connector::IdPack;
using connector::TtyFrame;

using connector_common::Unpacker;
using connector_common::ProtocolConfig;
using connector_common::CRC16Config;
using connector_common::protocol_type_e;

using robot_msg::msg::ImuRaw;


inline void print_array(const uint8_t* arr, size_t length) {
    std::cout << std::hex;
    for (size_t i = 0; i < length; ++i) {
        std::cout << "0x" << static_cast<int>(arr[i]) << " ";
    }

    std::cout << std::dec;
    if (length != 0 && arr[length - 1] == 0x7e)
        std::cout << std::endl;
}

static constexpr uint8_t PEER_ID = 0x01;

class TtyNode : public rclcpp::Node {
public:
    TtyNode() 
        : Node("test_tty"), 
        connector(), crn(connector), cs(connector) {
        connector.con_open("/dev/ttyUSB0", BaudRate::BAUD_1M);
        std::cout << "open tty" << std::endl;
        publisher_ = this->create_publisher<ImuRaw>("/imu_raw", 10);
        std::map<uint8_t, std::function<void(uint8_t, const uint8_t*, uint16_t)>> 
            update_func_map;
        std::map<uint8_t, std::function<bool(uint8_t)>> check_id_func_map;
        check_id_func_map[PEER_ID] = [](uint8_t _) -> bool { (void)_; return true; };
        update_func_map[PEER_ID] = [&](uint8_t id, const uint8_t* data, uint16_t length) -> void {
            (void)id;
            // std::cout << "length: " << length << std::endl;
            if (length > sizeof(imu_data)) return;
            memcpy(&imu_data, data, length);
            imu_raw_data.accel_x = imu_data.accel_x;
            imu_raw_data.accel_y = imu_data.accel_y;
            imu_raw_data.accel_z = imu_data.accel_z;
            imu_raw_data.gyro_x = imu_data.gyro_x;
            imu_raw_data.gyro_y = imu_data.gyro_y;
            imu_raw_data.gyro_z = imu_data.gyro_z;
            imu_raw_data.temperature = imu_data.temperature;

            // 1000Hz
            publisher_->publish(imu_raw_data);
        };
        unpacker.change_map(update_func_map, check_id_func_map);
        crn.register_callback([&](const TtyFrame::MSGT& frame) -> void {
            // std::cout << "recv: ";
            unpacker.unpack(frame.data.data(), frame.data.size());
            // printArray(frame.data.data(), frame.data.size());
            // RCLCPP_INFO(this->get_logger(), "recv: %s", frame.data.c_str());
        });

        // 定时器
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            []() -> void {
            }
        );
    }

private:
    robot_msg__msg__ImuRaw imu_data;
    ImuRaw imu_raw_data;
    Connector<ConnectorType::TTY> connector;
    ConnectorSingleRecvNode<ConnectorType::TTY, TtyFrame> crn;
    ConnectorSendNode<ConnectorType::TTY, TtyFrame> cs;
    Unpacker<ProtocolConfig<CRC16Config<0xFFFF, 0x1021>, 
        protocol_type_e::protocol0>> unpacker;
    // rclcpp::Subscription<MotorRef>::SharedPtr subscription_;
    // rclcpp::Subscription<PidParamSet>::SharedPtr subscription_controller_param;
    rclcpp::Publisher<ImuRaw>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TtyNode>());
    rclcpp::shutdown();
    return 0;
}