#include "rclcpp/rclcpp.hpp"

#include "connector/connector.hpp"
#include "common/protocol/serialized_protocol.hpp"
#include "motor/motor.hpp"
#include "controller/controller.hpp"
#include "observer/observer.hpp"
#include "depend_on_ros12/motor_node/motor_node.hpp"
#include "depend_on_ros12/attached_node/attached_node.hpp"
#include "depend_on_ros12/attached_node/watch_node.hpp"
#include "robot_msg/msg/imu_euler.h"
#include "robot_msg/msg/imu_euler.hpp"
#include "robot_msg/msg/imu_full_info.hpp"

#include "msg_convert.hpp"

using connector::Connector;
using connector::ConnectorType;
using connector::BaudRate;
using connector::ConnectorSingleRecvNode;
using connector::ConnectorSendNode;
using connector::IdPack;
using connector::TtyFrame;
using connector::CanFrame;

using connector_common::data_convert;
using connector_common::concat;
using connector_common::Unpacker;
using connector_common::ProtocolConfig;
using connector_common::CRC16Config;
using connector_common::protocol_type_e;

using motor_node::MotorNode;
using motor_node::MotorType;
using controller::Controller;
using controller::ControllerType;
using attached_node::AttachedNode;
using attached_node::WatchNode;
template <ControllerType ControllerTypeT, typename... ControllerArgs>
using ControllerNode = AttachedNode<Controller<ControllerTypeT>, ControllerArgs...>;

using observer::ObserverType;
using observer::Observer;
using observer::StateSpaceModel;
using observer::KalmanFilter;

template <ObserverType ObserverTypeT, typename... ObserverArgs>
using ObserverNode = AttachedNode<Observer<ObserverTypeT>, ObserverArgs...>;


using robot_msg::msg::ImuEuler;
using robot_msg::msg::ImuFullInfo;

static constexpr uint8_t PEER_ID = 0x01;

class GimbalNode : public rclcpp::Node {
public:
    GimbalNode() 
        : Node("gimbal"), 
        connector(), crn(connector), cs(connector),
        can0_connector("can0"),
        can1_connector("can1"),
        can0_recv_node(can0_connector),
        can1_recv_node(can1_connector),
        watch_node(*this) {
        declare_params();
        declare_watched_variables();
        pitch = create_6020("pitch", can0_recv_node,1);
        yaw = create_6020("yaw", can1_recv_node, 3);
        connector.con_open("/dev/ttyACM0", BaudRate::BAUD_1M);
        std::cout << "open tty" << std::endl;
        publisher_ = this->create_publisher<ImuEuler>("/imu_euler", 10);
        publisher_imu_full = this->create_publisher<ImuFullInfo>("/imu_full", 10);
        std::map<uint8_t, std::function<void(uint8_t, const uint8_t*, uint16_t)>> 
            update_func_map;
        std::map<uint8_t, std::function<bool(uint8_t)>> check_id_func_map;
        check_id_func_map[PEER_ID] = [](uint8_t _) -> bool { (void)_; return true; };
        update_func_map[PEER_ID] = [&](uint8_t id, const uint8_t* data, uint16_t length) -> void {
            (void)id;
            // std::cout << "length: " << length << std::endl;
            if (length != sizeof(imu_data)) {
                LOG_ERROR(1, "length %d != sizeof(imu_data) %ld", length, sizeof(imu_data));
                return;
            }
            memcpy(&imu_data, data, length);
            imu_msg.roll = imu_data.roll;
            imu_msg.pitch = imu_data.pitch;
            imu_msg.yaw = imu_data.yaw;
            imu_msg.pitch_gyro = imu_data.pitch_gyro;
            imu_msg.yaw_gyro = imu_data.yaw_gyro;
            data_convert(imu_msg, imu_full_info_msg);

            control_gimbal(imu_msg);
            observe_gimbal();
            // 1000Hz
            publisher_->publish(imu_msg);
            publisher_imu_full->publish(imu_full_info_msg);
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
            std::chrono::milliseconds(1),
            [this]() -> void {
            }
        );
    }

    auto create_6020(const std::string& name, 
            ConnectorSingleRecvNode<ConnectorType::CAN, CanFrame>& ccn, motor::MotorId id) -> 
            MotorNode<MotorType::DJI_6020>::SharedPtr {
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

    auto control_gimbal(const ImuEuler& data) -> void {
        // LOG_INFO(1, "control gimbal");
        pitch_controller.get().fdb(0) = data.pitch;
        pitch_controller.get().fdb(1) = data.pitch_gyro;

        pitch_controller.get().ref(0) = pitch_ref;
        pitch_controller.get().ref(1) = 0;
        
        yaw_controller.get().fdb(0) = data.yaw;
        yaw_controller.get().fdb(1) = data.yaw_gyro;

        yaw_controller.get().ref(0) = yaw_ref;
        yaw_controller.get().ref(1) = 0;

        pitch_controller.get().update();
        yaw_controller.get().update();

        // LOG_INFO(1, "control");
        CanFrame::MSGT id_pack;
        id_pack.data.resize(8);
        id_pack.id = pitch->get_motor().set_send_buf(pitch_controller.get().out, id_pack.data);
        if (can_send)
            pitch->get_motor().get_send_node().send(id_pack);
        
        id_pack.data.resize(8);
        id_pack.id = yaw->get_motor().set_send_buf(yaw_controller.get().out, id_pack.data);
        if (can_send)
            yaw->get_motor().get_send_node().send(id_pack);
    }

    auto observe_gimbal() -> void {
        using ObserverYawKf = std::remove_reference_t<decltype(yaw_kf.get())>;
        ObserverYawKf::UpdateData update_data = {
            .z = { yaw->get_motor().get_fdb().pos_zero_cross.rad.num, imu_full_info_msg.yaw_gyro.rad.num},
        };
        yaw_kf.get().predict(ObserverYawKf::PredictData{});
        yaw_kf.get().update(update_data);

        using ObserverPitchKf = std::remove_reference_t<decltype(pitch_kf.get())>;
        ObserverPitchKf::UpdateData update_data2 = {
            .z = { imu_full_info_msg.pitch_gyro.rad.num},
        };
        pitch_kf.get().predict(ObserverPitchKf::PredictData{});
        pitch_kf.get().update(update_data2);        
    }

    auto declare_params() -> void {
        this->declare_parameter("can_send", false);
        can_send = this->get_parameter("can_send").as_bool();
        pitch_controller.init(*this, concat("pitch"));
        yaw_controller.init(*this, concat("yaw"));

        yaw_kf.init(*this, concat("yaw_kf"));
        yaw_kf.get().config.model.A << 1, 0.001, 0, 1;
        yaw_kf.get().config.model.H << 1, 0, 0, 1;

        pitch_kf.init(*this, concat("pitch_kf"));
        pitch_kf.get().config.model.A << 1;
        pitch_kf.get().config.model.H << 1;
    }

    auto declare_watched_variables() -> void {
        watch_node.add_publisher(yaw_kf.get().get_state().x.at(0), 1, "observer/yaw/euler_rad");
        watch_node.add_publisher(yaw_kf.get().get_state().x.at(1), 1, "observer/yaw/gyro_rad");
        
        watch_node.add_publisher(pitch_kf.get().get_state().x.at(0), 1, "observer/pitch/gyro_rad");
    }

    ~GimbalNode() {
        crn.unregister();
        can0_recv_node.unregister();
        can1_recv_node.unregister();
    }

private:
    robot_msg__msg__ImuEuler imu_data;
    ImuEuler imu_msg;
    ImuFullInfo imu_full_info_msg;
    Connector<ConnectorType::TTY> connector;
    ConnectorSingleRecvNode<ConnectorType::TTY, TtyFrame> crn;
    ConnectorSendNode<ConnectorType::TTY, TtyFrame> cs;
    Unpacker<ProtocolConfig<CRC16Config<0xFFFF, 0x1021>, 
        protocol_type_e::protocol0>> unpacker;
    rclcpp::Publisher<ImuEuler>::SharedPtr publisher_;
    rclcpp::Publisher<ImuFullInfo>::SharedPtr publisher_imu_full;
    rclcpp::TimerBase::SharedPtr timer_;

    Connector<ConnectorType::CAN> can0_connector;
    Connector<ConnectorType::CAN> can1_connector;
    
    ConnectorSingleRecvNode<ConnectorType::CAN, CanFrame> can0_recv_node;
    ConnectorSingleRecvNode<ConnectorType::CAN, CanFrame> can1_recv_node;
    
    MotorNode<MotorType::DJI_6020>::SharedPtr pitch;
    MotorNode<MotorType::DJI_6020>::SharedPtr yaw;
    
    float pitch_ref = 0;
    float yaw_ref = 0;
    bool can_send;
    ControllerNode<controller::ControllerType::LQR> pitch_controller;
    ControllerNode<controller::ControllerType::LQR> yaw_controller;

    ObserverNode<ObserverType::KF, StateSpaceModel<2, 0, 2>> yaw_kf;
    ObserverNode<ObserverType::KF, StateSpaceModel<1, 0, 1>> pitch_kf;

    WatchNode watch_node;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GimbalNode>());
    rclcpp::shutdown();
    return 0;
}