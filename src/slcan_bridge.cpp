#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <functional>

#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
// #include <diagnostic_updater/diagnostic_updater.h>
#include "can_plugins2/msg/frame.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;
namespace slcan_bridge
{
    class SlcanBridge : public rclcpp::Node{
        public:
            SlcanBridge();
            ~SlcanBridge(){
                Stop();
                Close();
            }
         
            void canTxCallback(const can_plugins2::msg::Frame::SharedPtr &msg)const;
        private:
            void canRxTask(void);
            void processRxFrame(const uint8_t * const str_buf, const uint16_t str_len);


            bool waitForStatus(const rcl_duration_t& timeout);

            void send(std::string message)const;

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
            rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr can_rx_pub;
            rclcpp::Subscription<can_plugins2::msg::Frame>::SharedPtr can_tx_sub;


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
            rclcpp::TimerBase::SharedPtr timer[2]{nullptr};
            // void diagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);

            std::deque<std::string> send_packet_queue;
            void queue_message(std::string message);
            void start_packet_send(void);
            void packet_send_done(boost::system::error_code const &error);

            int rx_count;
            int tx_count;
    };
    SlcanBridge::SlcanBridge():Node("slcan_bridge"){
    // // Diagnostic
    // _updater.setHardwareID("SlcanBridge");
    // _updater.add("SlcanBridge",std::bind(&SlcanBridge::diagnostic,this, std::placeholders::_1));
    

    try{
        rclcpp::Parameter port_name = this->get_parameter("port");
        _port_name = port_name.as_string();
    }catch(rclcpp::ParameterTypeException e){
        //parameter is not set or not valid.
        //set default parameter.
        _port_name = "/dev/usbcan";
    }

    try{
        rclcpp::Parameter baud = this->get_parameter("baud");
        _baud = baud.as_int();
    }catch(rclcpp::ParameterTypeException e){
        //parameter is not set or not valid.
        //set default parameter.
        _baud = 1000000;
    }

    
    this->_service = new boost::asio::io_service();
    this->_port = new boost::asio::serial_port(*_service);
  
    this->_w = new boost::asio::io_service::work(*_service);

    this->_write_strand = new boost::asio::io_service::strand(*_service);
  
    this->can_rx_pub = this->create_publisher<can_plugins2::msg::Frame>("can_rx",10);

    // 2 threads for read/write
    this->_service_threads.create_thread(boost::bind((std::size_t (boost::asio::io_service::*)())&boost::asio::io_service::run, _service));
    this->_service_threads.create_thread(boost::bind((std::size_t (boost::asio::io_service::*)())&boost::asio::io_service::run, _service));
    timer[0] = this->create_wall_timer(1s,[this](){
        //TODO
    });
    timer[1] = this->create_wall_timer(1s,[this](){
        if (_error_count > _error_count_threshold) Reset();
    });
    printf("usb_can_node has started.");

    Reset();

    this->can_tx_sub = this->create_subscription<can_plugins2::msg::Frame>("can_tx",10,std::bind(&SlcanBridge::canTxCallback,this,_1));

    tx_count = 0;
    rx_count = 0;
  }

  void SlcanBridge::Reset()
  {
    _status = 1;
    _status_changed = false;
    _is_open = false;
    _error_count = 0;

    printf("opening serial port...");
    while(this->Open() && rclcpp::ok())
    {
        printf("failed to open serial port.retrying evert second");
        rclcpp::sleep_for(1s);
    }
  
    while(this->Initialize() && rclcpp::ok())
    {
        printf("failed to initialize.retrying evert second");
        rclcpp::sleep_for(1s);
    }
  
    printf("initialized");
  
    while(this->SetBaudRate(_baud) && rclcpp::ok())
    {
        printf("failed to set baud rate.retrying evert second");
        rclcpp::sleep_for(1s);
    }

    printf("baud rate set");
  
    while(this->Start() && rclcpp::ok())
    {
        printf("failed to start session.retrying evert second");
        rclcpp::sleep_for(1s);
    }

    _error = false;

  }

  void SlcanBridge::send(std::string message)const
  {
    _service->post(_write_strand->wrap(std::bind(&SlcanBridge::queue_message,this,std::move(message))));
  }

  void SlcanBridge::queue_message(std::string message)
  {
    bool write_in_progress = !send_packet_queue.empty();
    send_packet_queue.push_back(std::move(message));

    if(!write_in_progress)
    {
      start_packet_send();
    }
  }

  void SlcanBridge::start_packet_send()
  {
    async_write(*_port
                ,boost::asio::buffer(send_packet_queue.front())
                ,_write_strand->wrap(std::bind(&SlcanBridge::packet_send_done,this, std::placeholders::_1)));
  }

  void SlcanBridge::packet_send_done(boost::system::error_code const &error)
  {
    if(!error)
    {
        send_packet_queue.pop_front();
        _error_count = 0;
        tx_count++;//debug
    }
    else
    {
        printf("failed to send: %s #%d", error.message().c_str() ,error.value());
        _error_count++;
        if(_error_count>_error_count_threshold)
        {
            _error = true;
            send_packet_queue.clear();
            Close();
            return;
        }
    }
    
    if(!send_packet_queue.empty()){start_packet_send();}
  }

  void SlcanBridge::canTxCallback(const can_plugins2::msg::Frame::SharedPtr &msg)const
  {
      if(_error) return;

      char str_buf[CAN_MTU];
  
      int i = 0;
  
      int id_len;
  
      if(msg->is_rtr)
      {
          str_buf[i] = 'r';
      }
      else
      {
          str_buf[i] = 't';
      }
  
      if(msg->is_extended)
      {
          str_buf[i] -= 32;
          id_len = 8;
      }
      else
      {
          id_len = 3;
      }
      i++;
  
      int tmp = msg->id;
      for(int j = id_len; j > 0; j--)
      {
          str_buf[j] = tmp & 0x0f;
          tmp = tmp >> 4;
          i++;
      }
  
      // add DLC to buffer
      str_buf[i++] = msg->dlc;
  
      if(!msg->is_rtr)
      {
          // add data bytes
          for (int j = 0; j < msg->dlc; j++)
          {
              str_buf[i++] = (msg->data[j] >> 4);
              str_buf[i++] = (msg->data[j] & 0x0F);
          }
      }
  
      // convert to ASCII (2nd character to end)
      for (int j = 1; j < i; j++)
      {
          if (str_buf[j] < 0xA)
          {
              str_buf[j] += 0x30;
          }
          else
          {
              str_buf[j] += 0x37;
          }
      }
  
      // add carriage return (slcan EOL)
      str_buf[i++] = '\r';
      str_buf[i++] = '\0';

      send(str_buf);
  }

  void SlcanBridge::receive(void)
  {
      boost::asio::async_read(
              *_port,
              _receive_buffer,
              boost::asio::transfer_at_least(1), // receive at least one byte
              boost::bind(&SlcanBridge::onReceive, this, boost::asio::placeholders::error));
  }

  void SlcanBridge::onReceive(const boost::system::error_code& error)
  {
      if(error)
      {
          printf("failed to read: %s #%d", error.message().c_str() ,error.value());
          return;
      }

      const std::string data(boost::asio::buffer_cast<const char*>(_receive_buffer.data()), _receive_buffer.size());
      _receive_buffer.consume(_receive_buffer.size());
  
      for(const char c : data)
      {
          if(c == 0x07)
          {
              this->_status = -1;
              this->_status_changed = true;
  
              printf("S_Bell");
              _rx_str_len = 0;
          }
          else if(c == '\r')
          {
              if(_rx_str_len == 0 || _rx_str_buf[0] == 'z' || _rx_str_buf[0] == 'Z')
              {
                  this->_status = 0;
                  this->_status_changed = true;
  
                  printf("S_OK");
  
                  _rx_str_len = 0;
              }
              else
              {
                  this->processRxFrame(_rx_str_buf, _rx_str_len);
                  _rx_str_len = 0;
              }
          }
          else
          {
              _rx_str_buf[_rx_str_len++] = c;
              if(_rx_str_len > RX_STR_SIZE)
              {
                  printf("eww");
                  _rx_str_len = 0;
              }
          }
      }
  
      receive();
  }
} // namespace slcan_bridge


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<slcan_bridge::SlcanBridge>());
    rclcpp::shutdown();
    return 0;
}