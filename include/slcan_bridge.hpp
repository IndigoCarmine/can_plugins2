#include <rclcpp/rclcpp.hpp>
#include <boost/array.hpp>


namespace slcan_bridge
{
    class SlcanBridge : public rclcpp::Node
    {
    public:
        SlcanBridge(const rclcpp::NodeOptions &options);
        boost::array<unsigned char, 32> receive_api_frame_;
        // shutfdown process
        void onShutdown();
    };
} // namespace slcan_bridge
