#include <array>
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
    std::unique_ptr<can_plugins2::msg::Frame> frame;
    frame->id = id;
    frame->is_rtr = false;
    frame->is_extended = false;
    frame->is_error = false;

    frame->dlc = sizeof(T);
    frame->data.fill(0);

    can_pack<T>(frame->data, data);

    return frame;
  }

} // namespace can_utils
