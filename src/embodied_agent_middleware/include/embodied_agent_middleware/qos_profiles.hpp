#pragma once

#include <algorithm>
#include <cstddef>

#include <rclcpp/qos.hpp>

namespace embodied_agent_middleware
{

inline rclcpp::QoS command_qos(const std::size_t depth = 50)
{
  // 控制命令需要可靠传输，但必须保持 volatile；重启后重放旧运动命令不安全。
  return rclcpp::QoS(rclcpp::KeepLast(std::max<std::size_t>(1, depth)))
         .reliable().durability_volatile();
}

inline rclcpp::QoS event_qos(const std::size_t depth = 50)
{
  // ACK/result/BT 等生命周期事件应可靠且保持顺序，但不代表当前状态。
  return rclcpp::QoS(rclcpp::KeepLast(std::max<std::size_t>(1, depth)))
         .reliable().durability_volatile();
}

inline rclcpp::QoS state_qos(const std::size_t depth = 1)
{
  // 晚加入的监控应立即看到最新状态；只保留很浅历史，避免把状态误当事件流。
  return rclcpp::QoS(rclcpp::KeepLast(std::max<std::size_t>(1, depth)))
         .reliable().transient_local();
}

inline rclcpp::QoS sensor_qos(const std::size_t depth = 5)
{
  // 高频传感器允许丢旧帧，优先低延迟并兼容 Gazebo/真实雷达的 best-effort publisher。
  return rclcpp::QoS(rclcpp::KeepLast(std::max<std::size_t>(1, depth)))
         .best_effort().durability_volatile();
}

inline rclcpp::QoS audio_qos(const std::size_t depth = 5)
{
  // PCM 与传感器流相同：下游来不及消费时应丢旧帧，不能积压造成语音延迟。
  return sensor_qos(depth);
}

inline rclcpp::QoS diagnostics_qos(const std::size_t depth = 10)
{
  return rclcpp::QoS(rclcpp::KeepLast(std::max<std::size_t>(1, depth)))
         .reliable().durability_volatile();
}

}  // namespace embodied_agent_middleware
