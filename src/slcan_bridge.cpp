#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <string>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <chrono>
#include <future>
using namespace std::chrono_literals;
namespace slcan_bridge
{

    class SlcanBridge : public rclcpp::Node{

        private:
            boost::asio::io_service * io_service;
            boost::asio::serial_port * serial_port;
            rclcpp::TimerBase::SharedPtr timer_;
            bool is_connected = false;
            bool initializeSerialPort(std::string port_name);
            bool usbSend();
            void usbSendWithoutResponse();
        public:
            SlcanBridge(const rclcpp::NodeOptions & options);
            boost::array<unsigned char, 32> receive_api_frame;
            void onShutdown(){
                RCLCPP_INFO(this->get_logger(),"END");
                
            }
            
    };

    SlcanBridge::SlcanBridge(const rclcpp::NodeOptions & options):Node("slcan_bridge",options){
        rclcpp::on_shutdown([this](){this->onShutdown();});
        
        //initalize asio members
        io_service = new boost::asio::io_service();
        serial_port = new boost::asio::serial_port(*io_service);
        std::string port_name = "/dev/usbcan";

        auto a = std::async(&SlcanBridge::initializeSerialPort,this,port_name);
        RCLCPP_ERROR(get_logger(),"SlcanBridge is initialized.");
        // serial_port->async_read_some(boost::asio::buffer(receive_api_frame),boost::bind(read_handler, _1, boost::ref(receive_api_frame), _2));
    }

    bool SlcanBridge::initializeSerialPort(std::string port_name){
        rclcpp::WallRate rate(1s);
        while(true){
            try{
                serial_port->open(port_name);
                serial_port->set_option(boost::asio::serial_port_base::character_size(8));
                serial_port->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
                serial_port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
                serial_port->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
            }catch(boost::system::system_error e){
                RCLCPP_ERROR(get_logger(),"Cannot connect");
            }
            if(serial_port->is_open()){
                RCLCPP_INFO(get_logger(),"connected");
                is_connected = true;
                break;
            }
            rate.sleep();

        }
        return true;
    }

} // namespace slcan_bridge


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(slcan_bridge::SlcanBridge)