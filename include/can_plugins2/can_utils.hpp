#pragma once

#include <array>
#include <bit>
#include <can_plugins2/msg/frame.hpp>

namespace can_utils
{

  inline constexpr int CAN_MTU = 8;

  template <typename T>
  inline void can_unpack(const std::array<uint8_t, CAN_MTU> &buf, T &data)
  {
    memcpy(&data, buf.data(), sizeof(T));
  }

  template <typename T>
  inline void can_pack(std::array<uint8_t, CAN_MTU> &buf, const T data)
  {
    memcpy(buf.data(), &data, sizeof(T));
  }

  template <typename T>
  inline std::unique_ptr<can_plugins2::msg::Frame> generate_frame(const uint16_t id, const T data)
  {
    can_plugins2::msg::Frame frame;
    frame.id = id;
    frame.is_rtr = false;
    frame.is_extended = false;
    frame.is_error = false;

    frame.dlc = sizeof(T);
    frame.data.fill(0);

    can_pack<T>(frame.data, data);

    return make_unique<can_plugins2::msg::Frame>(frame);
  }

  template <typename T>
  inline std::unique_ptr<can_plugins2::msg::Frame> generate_frame(const uint16_t id, const T data, const std::endian endian = std::endian::native)
  {
    can_plugins2::msg::Frame frame;
    frame.id = id;
    frame.is_rtr = false;
    frame.is_extended = false;
    frame.is_error = false;

    frame.dlc = sizeof(T);
    frame.data.fill(0);

    can_pack<T>(frame.data, data);

    switch (endian)
    {
    case std::endian::little:
      if (std::endian::native == std::endian::big)
      {
        std::reverse(frame.data.begin(), frame.data.begin() + sizeof(T));
      }
      break;
    case std::endian::big:
      if (std::endian::native == std::endian::little)
      {
        std::reverse(frame.data.begin(), frame.data.begin() + sizeof(T));
      }
      break;
    default:
      throw std::runtime_error("Unknown endian");
    }

    return make_unique<can_plugins2::msg::Frame>(frame);
  }

} // namespace can_utils
