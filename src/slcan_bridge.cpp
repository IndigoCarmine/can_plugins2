#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <string>
#include <boost/array.hpp>
#include <boost/bind.hpp>
#include <chrono>
using namespace std::chrono_literals;
namespace slcan_bridge
{
    // void read_handler( const boost::system::error_code& error, boost::array<unsigned char, 32> buf, size_t length)
    // {
    //     std::cout << length << std::endl;
    //     for (size_t i = 0; i < length; ++i) {
    //         std::cout << std::hex << static_cast<unsigned int>(buf[i]) << " ";
    //     }
    //     std::cout << std::endl;
    // }

    class SlcanBridge : public rclcpp::Node{

        private:
            boost::asio::io_service * io_service;
            boost::asio::serial_port * serial_port;
            rclcpp::TimerBase::SharedPtr timer_;


            bool initializeSerialPort(std::string port_name);
            bool usbSend();
            void usbSendWithoutResponse();
        public:
            SlcanBridge();
            boost::array<unsigned char, 32> receive_api_frame;
            void onShutdown(){
                RCLCPP_INFO(this->get_logger(),"END");
            }
            
    };

    SlcanBridge::SlcanBridge():Node("slcan_bridge","slcan_bridge"){
        rclcpp::on_shutdown([this](){this->onShutdown()});
        
        //initalize asio members
        io_service = new boost::asio::io_service();
        serial_port = new boost::asio::serial_port(*io_service);
        
        rclcpp::WallRate rate(1s);
        while(true){
            try{
                serial_port->open("/dev/usbcan");
                serial_port->set_option(boost::asio::serial_port_base::character_size(8));
                serial_port->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
                serial_port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
                serial_port->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
            }catch(boost::system::system_error e){
                RCLCPP_ERROR(get_logger(),"Cannot connect");
            }
            if(serial_port->is_open()){
                RCLCPP_INFO(get_logger(),"connected");
                break;
            }
            rate.sleep();

        }




        // serial_port->async_read_some(boost::asio::buffer(receive_api_frame),boost::bind(read_handler, _1, boost::ref(receive_api_frame), _2));
        initializeSerialPort("TEST");
    }

    bool SlcanBridge::initializeSerialPort(std::string port_name){
        RCLCPP_INFO(this->get_logger(),port_name.c_str());
        return true;
    }

} // namespace slcan_bridge


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<slcan_bridge::SlcanBridge>());
    rclcpp::shutdown();
    return 0;
}