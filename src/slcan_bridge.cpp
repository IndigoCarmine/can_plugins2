#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
// #include <diagnostic_updater/diagnostic_updater.h>
#include "can_plugins2/msg/frame.hpp"

namespace slcan_bridge
{
    class SlcanBridge : public rclcpp::Node{
        public:
            virtual void onInit();
            ~SlcanBridge(){
                Stop();
                Close();
            }
        private:
            void canRxTask(void);
            void processRxFrame(const uint8_t * const str_buf, const uint16_t str_len);

            void canTxCallback(const can_plugins2::msg::Frame::ConstPtr &msg);

            bool waitForStatus(const rcl_duration_t& timeout);

            void send(std::string message);

            void onReceive(const boost::system::error_code& error);
            void receive(void);

            void Reset(void);
            
            bool Initialize(void);

            bool Open(void);
            bool Close(void);

            bool Start(void);
            bool Stop(void);

            bool SetBaudRate(const int baud);

            void Dispose(void);

            can_plugins2::msg::Frame can_rx_msg;
            rclcpp::Publisher<can_plugins2::msg::Frame> can_rx_pub;
            rclcpp::Subscription<can_plugins2::msg::Frame> can_tx_sub;

            boost::asio::serial_port* _port;
            boost::asio::io_service* _service;
            boost::asio::io_service::work* _w;

            boost::asio::io_service::strand* _write_strand;

            boost::thread_group _service_threads;

            boost::asio::streambuf _receive_buffer;
          
            int _status;
            bool _status_changed = false;
            bool _is_open;
            static constexpr int _error_count_threshold = 3;
            int _error_count = _error_count_threshold + 1;
            bool _error=true;

            std::string _port_name;
            int _baud;

            static constexpr int RX_STR_SIZE = 32;
            uint8_t _rx_str_buf[RX_STR_SIZE];
            uint16_t _rx_str_len = 0;

            static constexpr int CAN_MTU = 32;

            // // Diagnostic
            // diagnostic_updater::Updater _updater;
            // ros::Timer timer[2];
            // void diagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);

            std::deque<std::string> send_packet_queue;
            void queue_message(std::string message);
            void start_packet_send(void);
            void packet_send_done(boost::system::error_code const &error);

            int rx_count;
            int tx_count;
    };
    // void SlcanBridge::onInit()override{
    // // Diagnostic
    // _updater.setHardwareID("SlcanBridge");
    // _updater.add("SlcanBridge",std::bind(&SlcanBridge::diagnostic,this, std::placeholders::_1));
      
    // _private_nh.param<std::string>("port", _port_name,"/dev/usbcan");
    // _private_nh.param("baud",_baud,1000000);
    
    // this->_service = new boost::asio::io_service();
    // this->_port = new boost::asio::serial_port(*_service);
  
    // this->_w = new boost::asio::io_service::work(*_service);

    // this->_write_strand = new boost::asio::io_service::strand(*_service);
  
    // this->can_rx_pub = _nh.advertise<can_plugins::Frame>("can_rx", 1000);

    // // 2 threads for read/write
    // this->_service_threads.create_thread(boost::bind((std::size_t (boost::asio::io_service::*)())&boost::asio::io_service::run, _service));
    // this->_service_threads.create_thread(boost::bind((std::size_t (boost::asio::io_service::*)())&boost::asio::io_service::run, _service));

    // timer[0] = _nh.createTimer(ros::Duration(1),[&](const ros::TimerEvent &){
    //     _updater.update();
    // });
    // timer[1] = _nh.createTimer(ros::Duration(1),[&](const ros::TimerEvent &){
    //     if (_error_count > _error_count_threshold) Reset();
    // });
  
    // NODELET_INFO("usb_can_node has started.");

    // Reset();

    // this->can_tx_sub = this->

    // tx_count = 0;
    // rx_count = 0;
//   }
} // namespace slcan_bridge


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<slcan_bridge::SlcanBridge>());
    rclcpp::shutdown();
    return 0;
}