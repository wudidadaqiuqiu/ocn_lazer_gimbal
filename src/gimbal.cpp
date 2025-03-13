#include "rclcpp/rclcpp.hpp"
#include "connector/connector.hpp"
#include "common/protocol/serialized_protocol.hpp"
#include "motor/motor.hpp"
#include "depend_on_ros12/motor_node/motor_node.hpp"
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

using motor_node::MotorNode;
using motor_node::MotorType;

using robot_msg::msg::ImuEuler;

static constexpr uint8_t PEER_ID = 0x01;

class TtyNode : public rclcpp::Node {
public:
    TtyNode() 
        : Node("test_tty"), 
        connector(), crn(connector), cs(connector),
        can0_connector("can0"),
        can1_connector("can1"),
        can0_recv_node(can0_connector),
        can1_recv_node(can1_connector) {
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

        pitch = create_6020("pitch", can0_recv_node,1);
        yaw = create_6020("yaw", can1_recv_node, 3);
    }

    MotorNode<MotorType::DJI_6020>::SharedPtr create_6020(const std::string& name, 
        ConnectorSingleRecvNode<ConnectorType::CAN, CanFrame>& ccn, motor::MotorId id) {
        rclcpp::NodeOptions options;
        using Motor6020 = MotorNode<MotorType::DJI_6020>;
        options.use_intra_process_comms(true);  // 启用进程内通信
        // LOG_INFO(1, "name: %s, id: %d", std::to_string(id).c_str(), id);
        Motor6020::Config config = {
            name,
            {ccn, id},
            10, options
        };
        Motor6020::SharedPtr mn = std::make_shared<Motor6020>(config);
        return mn;
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

    Connector<ConnectorType::CAN> can0_connector;
    Connector<ConnectorType::CAN> can1_connector;
    
    ConnectorSingleRecvNode<ConnectorType::CAN, CanFrame> can0_recv_node;
    ConnectorSingleRecvNode<ConnectorType::CAN, CanFrame> can1_recv_node;
    
    MotorNode<MotorType::DJI_6020>::SharedPtr pitch;
    MotorNode<MotorType::DJI_6020>::SharedPtr yaw;
    
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TtyNode>());
    rclcpp::shutdown();
    return 0;
}