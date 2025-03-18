#pragma once

#include "msg_layer/msg_layer.hpp"
#include "robot_msg/msg/imu_full_info.hpp"
#include "robot_msg/msg/imu_euler.hpp"


template <>
inline void connector_common::data_convert
    (const robot_msg::msg::ImuEuler& a, robot_msg::msg::ImuFullInfo& b) {
    using connector_common::Deg;
    using con_used_msg::AngleRelate;
    data_convert(Deg(a.roll), b.roll);
    data_convert(Deg(a.pitch), b.pitch);
    data_convert(Deg(a.yaw), b.yaw);
    data_convert(Deg(a.pitch_gyro), b.pitch_gyro);
    data_convert(Deg(a.yaw_gyro), b.yaw_gyro);
}



template <>
inline void connector_common::data_convert(const con_used_msg::AngleRelate& a, con_used_msg::AngleRelate& t) {
    t = a;
}
