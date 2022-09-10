#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <functional>

#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include "can_plugins2/msg/frame.hpp"
using namespace std::chrono_literals;
using std::placeholders::_1;
namespace slcan_bridge
{
    class SlcanBridge : public rclcpp::Node{
        public:
            SlcanBridge(const std::string& name_space="", 
    const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
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

            // Diagnostic
            
            diagnostic_updater::Updater _updater{this};
            rclcpp::TimerBase::SharedPtr timer[2]{nullptr};
            void diagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);

            std::deque<std::string> send_packet_queue;
            void queue_message(std::string message);
            void start_packet_send(void);
            void packet_send_done(boost::system::error_code const &error);

            int rx_count;
            int tx_count;
    };
    SlcanBridge::SlcanBridge(const std::string& name_space, const rclcpp::NodeOptions& options):Node("slcan_bridge",name_space,options){
    // Diagnostic
    _updater.setHardwareID("SlcanBridge");
    _updater.add("SlcanBridge",std::bind(&SlcanBridge::diagnostic,this, std::placeholders::_1));
    

    try{
        rclcpp::Parameter port_name = get_parameter("port");
        _port_name = port_name.as_string();
    }catch(rclcpp::ParameterTypeException e){
        //parameter is not set or not valid.
        //set default parameter.
        _port_name = "/dev/usbcan";
    }

    try{
        rclcpp::Parameter baud = get_parameter("baud");
        _baud = baud.as_int();
    }catch(rclcpp::ParameterTypeException e){
        //parameter is not set or not valid.
        //set default parameter.
        _baud = 1000000;
    }

    
    _service = new boost::asio::io_service();
    _port = new boost::asio::serial_port(*_service);
  
    _w = new boost::asio::io_service::work(*_service);

    _write_strand = new boost::asio::io_service::strand(*_service);
  
    can_rx_pub = create_publisher<can_plugins2::msg::Frame>("can_rx",10);

    // 2 threads for read/write
    _service_threads.create_thread(boost::bind((std::size_t (boost::asio::io_service::*)())&boost::asio::io_service::run, _service));
    _service_threads.create_thread(boost::bind((std::size_t (boost::asio::io_service::*)())&boost::asio::io_service::run, _service));
    timer[0] = create_wall_timer(1s,[this](){
        //TODO
    });
    timer[1] = create_wall_timer(1s,[this](){
        if (_error_count > _error_count_threshold) Reset();
    });
    RCLCPP_INFO(this->get_logger(),"usb_can_node has started.");

    Reset();

    can_tx_sub = create_subscription<can_plugins2::msg::Frame>("can_tx",10,std::bind(&SlcanBridge::canTxCallback,this,_1));

    tx_count = 0;
    rx_count = 0;
  }

  void SlcanBridge::Reset()
  {
    _status = 1;
    _status_changed = false;
    _is_open = false;
    _error_count = 0;

    RCLCPP_INFO(this->get_logger(),"opening serial port...");
    while(Open() && rclcpp::ok())
    {
        RCLCPP_INFO(this->get_logger(),"failed to open serial port.retrying evert second");
        rclcpp::sleep_for(1s);
    }
  
    while(Initialize() && rclcpp::ok())
    {
        RCLCPP_INFO(this->get_logger(),"failed to initialize.retrying evert second");
        rclcpp::sleep_for(1s);
    }
  
    RCLCPP_INFO(this->get_logger(),"initialized");
  
    while(SetBaudRate(_baud) && rclcpp::ok())
    {
        RCLCPP_INFO(this->get_logger(),"failed to set baud rate.retrying evert second");
        rclcpp::sleep_for(1s);
    }

    RCLCPP_INFO(this->get_logger(),"baud rate set");
  
    while(Start() && rclcpp::ok())
    {
        RCLCPP_INFO(this->get_logger(),"failed to start session.retrying evert second");
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
        RCLCPP_INFO(this->get_logger(),"failed to send: %s #%d", error.message().c_str() ,error.value());
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
          RCLCPP_INFO(this->get_logger(),"failed to read: %s #%d", error.message().c_str() ,error.value());
          return;
      }

      const std::string data(boost::asio::buffer_cast<const char*>(_receive_buffer.data()), _receive_buffer.size());
      _receive_buffer.consume(_receive_buffer.size());
  
      for(const char c : data)
      {
          if(c == 0x07)
          {
              _status = -1;
              _status_changed = true;
  
              RCLCPP_INFO(this->get_logger(),"S_Bell");
              _rx_str_len = 0;
          }
          else if(c == '\r')
          {
              if(_rx_str_len == 0 || _rx_str_buf[0] == 'z' || _rx_str_buf[0] == 'Z')
              {
                  _status = 0;
                  _status_changed = true;
  
                  RCLCPP_INFO(this->get_logger(),"S_OK");
  
                  _rx_str_len = 0;
              }
              else
              {
                  processRxFrame(_rx_str_buf, _rx_str_len);
                  _rx_str_len = 0;
              }
          }
          else
          {
              _rx_str_buf[_rx_str_len++] = c;
              if(_rx_str_len > RX_STR_SIZE)
              {
                  RCLCPP_INFO(this->get_logger(),"eww");
                  _rx_str_len = 0;
              }
          }
      }
  
      receive();
  }
  void SlcanBridge::processRxFrame(const uint8_t * const str_buf, const uint16_t str_len)
  {
      uint8_t buf[32];
      int i = 0;
  
      buf[i] = str_buf[i];
      i++;
  
      // convert from ASCII (2nd character to end)
      for (i = 1; i < str_len; i++)
      {
          if(str_buf[i] >= 'a')
          {
              // lowercase letters
              buf[i] = str_buf[i] - 'a' + 10;
          }
          else if(str_buf[i] >= 'A')
          {
              // uppercase letters
              buf[i] = str_buf[i] - 'A' + 10;
          }
          else
          {
              // numbers
              buf[i] = str_buf[i] - '0';
          }
      }
  
      if (buf[0] == 't' || buf[0] == 'T')
      {
          // transmit data frame command
          can_rx_msg.is_rtr = false;
      }
      else if (buf[0] == 'r' || buf[0] == 'R')
      {
          // transmit remote frame command
          can_rx_msg.is_rtr = true;
      }
      else
      {
          // error, unknown command
          return;// -1;
      }
  
      uint8_t id_len;
  
      if (buf[0] == 't' || buf[0] == 'r')
      {
          can_rx_msg.is_extended = false;
          id_len = 3;
      }
      else if (buf[0] == 'T' || buf[0] == 'R')
      {
          can_rx_msg.is_extended = true;
          id_len = 8;
      }
      else
      {
          // error
          return;// -1;
      }
      can_rx_msg.is_error = false;
  
      i = 1;
      can_rx_msg.id = 0;
      while (i <= id_len)
      {
          can_rx_msg.id <<= 4;
          can_rx_msg.id += buf[i++];
      }
  
      can_rx_msg.dlc = buf[i++];
      if (can_rx_msg.dlc < 0 || can_rx_msg.dlc > 8)
      {
          return;// -1;
      }
  
      uint8_t j;
      for (j = 0; j < can_rx_msg.dlc; j++)
      {
          can_rx_msg.data[j] = (buf[i++] * 16) + buf[i++];
      }
  
      // send the message
      can_rx_pub->publish(can_rx_msg);

      rx_count++;
  }

  //wait 1s or break when detect _status_changed.
  bool SlcanBridge::waitForStatus(const rcl_duration_t &timeout = rcl_duration_t())
  {
//       rcltime timeout_time = ros::Time::now() + timeout;

//       while(!_status_changed)
//       {
//         // Determine how long we should wait
//         ros::Duration time_left = timeout_time - ros::Time::now();
   
//         // Check if we're past the timeout time
//         if (time_left <= ros::Duration(0,0) )
//           break;
//       }
  
//       _status_changed = false;
  
      return false;
  }
  
  bool SlcanBridge::Initialize(void)
  {
      if(!_port->is_open())
      {
          RCLCPP_INFO(get_logger(),"serial port is not open");
          return true;
      }
  
      // flush and close current connection
      send("\r");
  
      waitForStatus();
  
      RCLCPP_INFO(get_logger(),"closing last session");
  
      send("C\r");
      waitForStatus();
      
      if(_status)
      {
          return true;
      }
  
      return false;
  }
  
  bool SlcanBridge::Open(void)
  {
      boost::system::error_code error;
  
      if(_port->is_open())
      {
          RCLCPP_ERROR(get_logger(),"tried to open port while it's already open");
  
          return true;
      }
  
      _port->open(_port_name, error);
  
      if(error)
      {
          RCLCPP_ERROR(get_logger(),"failed to open port name: %s #%d", error.message().c_str() ,error.value());
  
          return true;
      }

      _port->set_option(boost::asio::serial_port_base::character_size(8));
      _port->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
      _port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
      _port->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  
      receive();
  
      RCLCPP_INFO(get_logger(),"successfully opened serial port");
  
      return false;
  }
  
  bool SlcanBridge::Close(void)
  {
      boost::system::error_code error;
      if(!_port->is_open())
      {
          // pretend nothing happend
          RCLCPP_INFO(get_logger(),"successfully closed serial port : port already closed");
          return false;
      }
  
      _port->cancel();
  
      _port->close(error);
  
      if(error)
      {
        RCLCPP_ERROR(get_logger(),"failed to CLOSE port: %s #%d", error.message().c_str() ,error.value());
  
        return true;
      }
  
      RCLCPP_INFO(get_logger(),"successfully closed serial port");
      return false;
  }
  
  bool SlcanBridge::Start(void)
  {
      if(!_port->is_open())
      {
          RCLCPP_ERROR(get_logger(),"serial port is not open");
          return true;
      }
  
      send("O\r");
  
      waitForStatus();
      if(_status)
      {
          _is_open = false;
          return true;
      }
  
      _is_open = true;
      return false;
  }
  
  bool SlcanBridge::Stop(void)
  {
      if(!_port->is_open())
      {
          RCLCPP_ERROR(get_logger(),"serial port is not open");
          return true;
      }
  
      send("C\r");
      _is_open = false;
  
      waitForStatus();
  
      bool result = _status;
  
      return result;
  }
  
  bool SlcanBridge::SetBaudRate(const int baud)
  {
      if(!_port->is_open())
      {
          RCLCPP_ERROR(get_logger(),"serial port is not open");
          return true;
      }
  
      if(baud == 10000)
      {
          send("S0\r");
      }
      else if(baud == 20000)
      {
          send("S1\r");
      }
      else if(baud == 50000)
      {
          send("S2\r");
      }
      else if(baud == 100000)
      {
          send("S3\r");
      }
      else if(baud == 125000)
      {
          send("S4\r");
      }
      else if(baud == 250000)
      {
          send("S5\r");
      }
      else if(baud == 500000)
      {
          send("S6\r");
      }
      else if(baud == 800000)
      {
          send("S7\r");
      }
      else if(baud == 1000000)
      {
          send("S8\r");
      }
      else
      {
          return false;
      }
  
      waitForStatus();
      if(_status)
      {
          return true;
      }
  
      return false;
  }

  void SlcanBridge::diagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    if (_error_count > _error_count_threshold || _error)
    {
      stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "SlcanBridge:ERROR");
    }
    else
    {
      stat.summaryf(diagnostic_msgs::msg::DiagnosticStatus::OK, "SlcanBridge:OK");
    }
  }
} // namespace slcan_bridge


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<slcan_bridge::SlcanBridge>());
    rclcpp::shutdown();
    return 0;
}