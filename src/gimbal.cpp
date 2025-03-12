#include "rclcpp/rclcpp.hpp"
#include "connector/connector.hpp"
#include "common/protocol/serialized_protocol.hpp"
#include "robot_msg/msg/imu_euler.h"
#include "robot_msg/msg/imu_euler.hpp"

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

using robot_msg::msg::ImuEuler;

static constexpr uint8_t PEER_ID = 0x01;

class TtyNode : public rclcpp::Node {
public:
    TtyNode() 
        : Node("test_tty"), 
        connector(), crn(connector), cs(connector) {
        connector.con_open("/dev/ttyACM0", BaudRate::BAUD_1M);
        std::cout << "open tty" << std::endl;
        publisher_ = this->create_publisher<ImuEuler>("/imu_euler", 10);
        std::map<uint8_t, std::function<void(uint8_t, const uint8_t*, uint16_t)>> 
            update_func_map;
        std::map<uint8_t, std::function<bool(uint8_t)>> check_id_func_map;
        check_id_func_map[PEER_ID] = [](uint8_t _) -> bool { (void)_; return true; };
        update_func_map[PEER_ID] = [&](uint8_t id, const uint8_t* data, uint16_t length) -> void {
            (void)id;
            // std::cout << "length: " << length << std::endl;
            if (length > sizeof(imu_data)) return;
            memcpy(&imu_data, data, length);
            imu_msg.roll = imu_data.roll;
            imu_msg.pitch = imu_data.pitch;
            imu_msg.yaw = imu_data.yaw;

            // 1000Hz
            publisher_->publish(imu_msg);
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
    robot_msg__msg__ImuEuler imu_data;
    ImuEuler imu_msg;
    Connector<ConnectorType::TTY> connector;
    ConnectorSingleRecvNode<ConnectorType::TTY, TtyFrame> crn;
    ConnectorSendNode<ConnectorType::TTY, TtyFrame> cs;
    Unpacker<ProtocolConfig<CRC16Config<0xFFFF, 0x1021>, 
        protocol_type_e::protocol0>> unpacker;
    rclcpp::Publisher<ImuEuler>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TtyNode>());
    rclcpp::shutdown();
    return 0;
}