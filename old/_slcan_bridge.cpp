#include <ros/ros.h>

#include <can_plugins/Frame.h>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <diagnostic_updater/diagnostic_updater.h>

namespace can_plugins{

  class SlcanBridge : public nodelet::Nodelet
  {
  public:
    virtual void onInit();
    ~SlcanBridge(){
        Stop();
        Close();
    }
  private:
    ros::NodeHandle _nh;
    ros::NodeHandle _private_nh;

    void canRxTask(void);
    void processRxFrame(const uint8_t * const str_buf, const uint16_t str_len);

    void canTxCallback(const can_plugins::Frame::ConstPtr &msg);

    bool waitForStatus(const ros::Duration& timeout);

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

    can_plugins::Frame can_rx_msg;
    ros::Publisher can_rx_pub;
    ros::Subscriber can_tx_sub;

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
    diagnostic_updater::Updater _updater;
    ros::Timer timer[2];
    void diagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat);

    std::deque<std::string> send_packet_queue;
    void queue_message(std::string message);
    void start_packet_send(void);
    void packet_send_done(boost::system::error_code const &error);

    int rx_count;
    int tx_count;
  };

  void SlcanBridge::onInit(){
    // _nh = getNodeHandle();
    // _private_nh = getPrivateNodeHandle();
    _nh = getMTNodeHandle();
    _private_nh = getMTPrivateNodeHandle();

    // Diagnostic
    _updater.setHardwareID("SlcanBridge");
    _updater.add("SlcanBridge",std::bind(&SlcanBridge::diagnostic,this, std::placeholders::_1));
      
    _private_nh.param<std::string>("port", _port_name,"/dev/usbcan");
    _private_nh.param("baud",_baud,1000000);
    
    this->_service = new boost::asio::io_service();
    this->_port = new boost::asio::serial_port(*_service);
  
    this->_w = new boost::asio::io_service::work(*_service);

    this->_write_strand = new boost::asio::io_service::strand(*_service);
  
    this->can_rx_pub = _nh.advertise<can_plugins::Frame>("can_rx", 1000);

    // 2 threads for read/write
    this->_service_threads.create_thread(boost::bind((std::size_t (boost::asio::io_service::*)())&boost::asio::io_service::run, _service));
    this->_service_threads.create_thread(boost::bind((std::size_t (boost::asio::io_service::*)())&boost::asio::io_service::run, _service));

    timer[0] = _nh.createTimer(ros::Duration(1),[&](const ros::TimerEvent &){
        _updater.update();
    });
    timer[1] = _nh.createTimer(ros::Duration(1),[&](const ros::TimerEvent &){
        if (_error_count > _error_count_threshold) Reset();
    });
  
    NODELET_INFO("usb_can_node has started.");

    Reset();

    this->can_tx_sub = _nh.subscribe<can_plugins::Frame>("can_tx", 1000, &SlcanBridge::canTxCallback, this);

    tx_count = 0;
    rx_count = 0;
  }

  void SlcanBridge::Reset()
  {
    _status = 1;
    _status_changed = false;
    _is_open = false;
    _error_count = 0;

    NODELET_DEBUG("opening serial port...");
    while(this->Open() && ros::ok())
    {
        NODELET_ERROR("failed to open serial port.retrying evert second");
        ros::Duration(1).sleep();
    }
  
    while(this->Initialize() && ros::ok())
    {
        NODELET_ERROR("failed to initialize.retrying evert second");
        ros::Duration(1).sleep();
    }
  
    NODELET_INFO("initialized");
  
    while(this->SetBaudRate(_baud) && ros::ok())
    {
        NODELET_ERROR("failed to set baud rate.retrying evert second");
        ros::Duration(1).sleep();
    }

    NODELET_INFO("baud rate set");
  
    while(this->Start() && ros::ok())
    {
        NODELET_ERROR("failed to start session.retrying evert second");
        ros::Duration(1).sleep();
    }

    _error = false;

  }

  void SlcanBridge::send(std::string message)
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
        NODELET_ERROR("failed to send: %s #%d", error.message().c_str() ,error.value());
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

  void SlcanBridge::canTxCallback(const can_plugins::Frame::ConstPtr &msg)
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

      this->send(str_buf);
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
          NODELET_ERROR("failed to read: %s #%d", error.message().c_str() ,error.value());
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
  
              NODELET_INFO("S_Bell");
              _rx_str_len = 0;
          }
          else if(c == '\r')
          {
              if(_rx_str_len == 0 || _rx_str_buf[0] == 'z' || _rx_str_buf[0] == 'Z')
              {
                  this->_status = 0;
                  this->_status_changed = true;
  
                  NODELET_INFO("S_OK");
  
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
                  NODELET_ERROR("eww");
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
          this->can_rx_msg.is_rtr = false;
      }
      else if (buf[0] == 'r' || buf[0] == 'R')
      {
          // transmit remote frame command
          this->can_rx_msg.is_rtr = true;
      }
      else
      {
          // error, unknown command
          return;// -1;
      }
  
      uint8_t id_len;
  
      if (buf[0] == 't' || buf[0] == 'r')
      {
          this->can_rx_msg.is_extended = false;
          id_len = 3;
      }
      else if (buf[0] == 'T' || buf[0] == 'R')
      {
          this->can_rx_msg.is_extended = true;
          id_len = 8;
      }
      else
      {
          // error
          return;// -1;
      }
      this->can_rx_msg.is_error = false;
  
      i = 1;
      this->can_rx_msg.id = 0;
      while (i <= id_len)
      {
          this->can_rx_msg.id <<= 4;
          this->can_rx_msg.id += buf[i++];
      }
  
      this->can_rx_msg.dlc = buf[i++];
      if (this->can_rx_msg.dlc < 0 || this->can_rx_msg.dlc > 8)
      {
          return;// -1;
      }
  
      uint8_t j;
      for (j = 0; j < this->can_rx_msg.dlc; j++)
      {
          this->can_rx_msg.data[j] = (buf[i++] * 16) + buf[i++];
      }
  
      // send the message
      this->can_rx_pub.publish(this->can_rx_msg);

      rx_count++;
  }

  bool SlcanBridge::waitForStatus(const ros::Duration& timeout = ros::Duration(1.0))
  {
      ros::Time timeout_time = ros::Time::now() + timeout;

      while(!this->_status_changed)
      {
        // Determine how long we should wait
        ros::Duration time_left = timeout_time - ros::Time::now();
   
        // Check if we're past the timeout time
        if (time_left <= ros::Duration(0,0) )
          break;
      }
  
      this->_status_changed = false;
  
      return false;
  }
  
  bool SlcanBridge::Initialize(void)
  {
      if(!this->_port->is_open())
      {
          NODELET_ERROR("serial port is not open");
          return true;
      }
  
      // flush and close current connection
      this->send("\r");
  
      this->waitForStatus();
  
      NODELET_INFO("closing last session");
  
      this->send("C\r");
      this->waitForStatus();
      
      if(this->_status)
      {
          return true;
      }
  
      return false;
  }
  
  bool SlcanBridge::Open(void)
  {
      boost::system::error_code error;
  
      if(this->_port->is_open())
      {
          NODELET_ERROR("tried to open port while it's already open");
  
          return true;
      }
  
      this->_port->open(_port_name, error);
  
      if(error)
      {
          NODELET_ERROR("failed to open port name: %s #%d", error.message().c_str() ,error.value());
  
          return true;
      }

      _port->set_option(boost::asio::serial_port_base::character_size(8));
      _port->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
      _port->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
      _port->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
  
      this->receive();
  
      NODELET_INFO("successfully opened serial port");
  
      return false;
  }
  
  bool SlcanBridge::Close(void)
  {
      boost::system::error_code error;
      if(!this->_port->is_open())
      {
          // pretend nothing happend
          NODELET_INFO("successfully closed serial port : port already closed");
          return false;
      }
  
      this->_port->cancel();
  
      this->_port->close(error);
  
      if(error)
      {
        NODELET_ERROR("failed to CLOSE port: %s #%d", error.message().c_str() ,error.value());
  
        return true;
      }
  
      NODELET_INFO("successfully closed serial port");
      return false;
  }
  
  bool SlcanBridge::Start(void)
  {
      if(!this->_port->is_open())
      {
          NODELET_ERROR("serial port is not open");
          return true;
      }
  
      this->send("O\r");
  
      this->waitForStatus();
      if(this->_status)
      {
          this->_is_open = false;
          return true;
      }
  
      this->_is_open = true;
      return false;
  }
  
  bool SlcanBridge::Stop(void)
  {
      if(!this->_port->is_open())
      {
          NODELET_ERROR("serial port is not open");
          return true;
      }
  
      this->send("C\r");
      this->_is_open = false;
  
      this->waitForStatus();
  
      bool result = this->_status;
  
      return result;
  }
  
  bool SlcanBridge::SetBaudRate(const int baud)
  {
      if(!this->_port->is_open())
      {
          NODELET_ERROR("serial port is not open");
          return true;
      }
  
      if(baud == 10000)
      {
          this->send("S0\r");
      }
      else if(baud == 20000)
      {
          this->send("S1\r");
      }
      else if(baud == 50000)
      {
          this->send("S2\r");
      }
      else if(baud == 100000)
      {
          this->send("S3\r");
      }
      else if(baud == 125000)
      {
          this->send("S4\r");
      }
      else if(baud == 250000)
      {
          this->send("S5\r");
      }
      else if(baud == 500000)
      {
          this->send("S6\r");
      }
      else if(baud == 800000)
      {
          this->send("S7\r");
      }
      else if(baud == 1000000)
      {
          this->send("S8\r");
      }
      else
      {
          return false;
      }
  
      this->waitForStatus();
      if(this->_status)
      {
          return true;
      }
  
      return false;
  }

  void SlcanBridge::diagnostic(diagnostic_updater::DiagnosticStatusWrapper& stat)
  {
    if (_error_count > _error_count_threshold || _error)
    {
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::ERROR, "SlcanBridge:ERROR");
    }
    else
    {
      stat.summaryf(diagnostic_msgs::DiagnosticStatus::OK, "SlcanBridge:OK");
    }
  }

}// namespace can_plugins
PLUGINLIB_EXPORT_CLASS(can_plugins::SlcanBridge, nodelet::Nodelet);