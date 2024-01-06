#include "can_plugins2/slcan_bridge.hpp"

namespace slcan_bridge
{

    SlcanBridge::SlcanBridge(const rclcpp::NodeOptions &options) : Node("slcan_bridge", options)
    {
        rclcpp::on_shutdown([this]()
                            { this->onShutdown(); });

        // get parameters
        this->get_parameter_or("rx_topic_name", rx_topic_name_, rx_topic_name_);
        this->get_parameter_or("tx_topic_name", tx_topic_name_, tx_topic_name_);
        this->get_parameter_or("port_name", port_name_, port_name_);


        // initialize publisher and subscriber
        can_rx_pub_ = this->create_publisher<can_plugins2::msg::Frame>(SlcanBridge::rx_topic_name_, 10);
        can_tx_sub_ = this->create_subscription<can_plugins2::msg::Frame>(SlcanBridge::tx_topic_name_, 10, std::bind(&SlcanBridge::canTxCallback, this, _1));

        // initalize asio members
        io_context_ = std::make_shared<boost::asio::io_context>();
        serial_port_ = std::make_shared<boost::asio::serial_port>(io_context_->get_executor());
        work_guard_ = std::make_unique<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>>(io_context_->get_executor());
        // start io_context thread
        io_context_thread_ = std::thread([this]()
                                         {
                                             io_context_->run();
                                             RCLCPP_INFO(this->get_logger(), "io_context_->run() is finished."); });

        RCLCPP_INFO(get_logger(), "SlcanBridge is initialized.");

        initializeSerialPort(port_name_);
        asyncRead();
        handshake();
    }

    void SlcanBridge::canTxCallback(const can_plugins2::msg::Frame::SharedPtr msg)
    {
        if (!is_active_)
            return;

        asyncWrite(msg);
    }

    // port open and setting.
    void SlcanBridge::initializeSerialPort(const std::string port_name)
    {
        rclcpp::WallRate rate(10ms);
        while (!is_shutdown_)
        {
            try
            {
                serial_port_->open(port_name);
                serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
                serial_port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
                serial_port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
                serial_port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
                serial_port_->set_option(boost::asio::serial_port_base::baud_rate(115200));
            }
            catch (boost::system::system_error &e)
            {
                switch (e.code().value())
                {
                case 2:
                    RCLCPP_ERROR(get_logger(), "Cannot connect. No such file or directory");
                    break;
                case 13:
                    RCLCPP_ERROR(get_logger(), "Cannot connect. Permission denied");
                    break;
                default:
                    RCLCPP_ERROR(get_logger(), "Cannot connect. %s", e.what());
                    serial_port_->close();
                    break;
                }
            }
            if (serial_port_->is_open())
            {
                RCLCPP_INFO(get_logger(), "connected");
                is_connected_ = true;
                break;
            }
            rate.sleep();
        }
        return;
    }

    void SlcanBridge::asyncWrite(const std::vector<uint8_t> data)
    {
        io_context_->post([this, data]()
                          { boost::asio::async_write(*serial_port_, boost::asio::buffer(data),
                                                     boost::bind(&SlcanBridge::writeHandler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred)); });
        return;
    }

    void SlcanBridge::asyncWrite(const can_plugins2::msg::Frame::SharedPtr msg)
    {
        // data structure
        /*
        uint8_t command & frame_type: (command: if it is normal can frame, it is 0x00.)<<4 | is_rtr << 2 | is_extended << 1 | is_error
        uint8_t id[4] : can id
        uint8_t dlc : data length
        uint8_t data[8] : data
        */
        std::vector<uint8_t> data(6 + 8);
        data[0] = (msg->is_rtr << 2) | (msg->is_extended << 1) | (msg->is_error);
        data[5] = msg->dlc;
        data[1] = (msg->id >> 24) & 0xff;
        data[2] = (msg->id >> 16) & 0xff;
        data[3] = (msg->id >> 8) & 0xff;
        data[4] = msg->id & 0xff;
        for (int i = 0; i < 8; i++)
        {
            data[6 + i] = msg->data[i];
        }

        std::vector<uint8_t> output = cobs::encode(data);

        asyncWrite(output);
    }

    void SlcanBridge::asyncWrite(const slcan_command::Command command, const std::vector<uint8_t> data)
    {
        if (command == slcan_command::Normal)
            RCLCPP_ERROR(get_logger(), "asyncWrite(Command) can not use normal. you need to use asyncWrite(Frame)");

        // data structure
        /*
        uint8_t command & frame_type: (command: if it is normal can frame, it is 0x00.)<<4 | is_rtr << 2 | is_extended << 1 | is_error
        uint8_t id[] : data
        */
        std::vector<uint8_t> raw_data(1 + data.size());
        raw_data[0] = (command << 4);
        for (std::size_t i = 0; i < data.size(); i++)
        {
            raw_data[1 + i] = data[i];
        }
        std::vector<uint8_t> output = cobs::encode(raw_data);

        asyncWrite(output);
    }

    void SlcanBridge::readingProcess(const std::vector<uint8_t> data)
    {
        std::vector<uint8_t> cobs_output_buffer_ = cobs::decode(data);

        RCLCPP_INFO(get_logger(), "readingProcess %s", test::hex_to_string(cobs_output_buffer_).c_str());

        // check it is handshake. USBCAN will send "HelloSlcan" when the connection is established.
        constexpr uint8_t HelloSlcan[] = {slcan_command::Negotiation << 4, 'H', 'e', 'l', 'l', 'o', 'S', 'L', 'C', 'A', 'N'};
        if (cobs_output_buffer_.size() == 10 + 1)
        {
            bool is_handshake = true;
            for (int i = 0; i < 10; i++)
            {
                if (cobs_output_buffer_[i] != HelloSlcan[i])
                {
                    is_handshake = false;
                    break;
                }
            }
            if (is_handshake)
            {
                RCLCPP_INFO(get_logger(), "negotiation success");
                is_active_ = true;
                return;
            }
        }

        // publish the data to the topic.
        if (data.size() < 12)
        {
            RCLCPP_ERROR(get_logger(), "data size is too small");
            return;
        }

        // data structure
        /*
        uint8_t command & frame_type: (command: if it is normal can frame, it is 0x00.)<<4 | is_rtr << 2 | is_extended << 1 | is_error
        uint8_t id[4] : can id
        uint8_t dlc : data length
        uint8_t data[8] : data
        */
        auto msg = std::make_unique<can_plugins2::msg::Frame>();
        msg->is_error = cobs_output_buffer_[0] & 0x1;
        msg->is_extended = cobs_output_buffer_[0] >> 1 & 0x1;
        msg->is_rtr = cobs_output_buffer_[0] >> 2 & 0x1;
        msg->id = cobs_output_buffer_[1] << 24 | cobs_output_buffer_[2] << 16 | cobs_output_buffer_[3] << 8 | cobs_output_buffer_[4];
        msg->dlc = cobs_output_buffer_[5];
        for (int i = 0; i < 8; i++)
        {
            msg->data[i] = cobs_output_buffer_[6 + i];
        }
        can_rx_pub_->publish(std::move(msg));
        return;
    }

    bool SlcanBridge::handshake()
    {
        rclcpp::WallRate rate(10ms);
        while (!is_active_ && !is_shutdown_)
        {
            const std::vector<uint8_t> HelloUSBCAN = {'H', 'e', 'l', 'l', 'o', 'U', 'S', 'B', 'C', 'A', 'N'};
            asyncWrite(slcan_command::Negotiation, HelloUSBCAN);
            RCLCPP_INFO(get_logger(), "Waiting for negotiation...");
            rate.sleep();
        }
        return true;
    }

    void SlcanBridge::readOnceHandler(const boost::system::error_code &error, std::size_t bytes_transferred)
    {
        if (error)
        {
            RCLCPP_ERROR(get_logger(), "readOnceHandler error %s", error.message().c_str());
            return;
        }

        std::vector<uint8_t> data(bytes_transferred);

        // it can use iostream but
        const uint8_t *data_ptr = (const uint8_t *)boost::asio::buffer_cast<const char *>(read_streambuf_.data());
        for (std::size_t i = 0; i < bytes_transferred; i++)
        {
            data[i] = data_ptr[i];
        }

        SlcanBridge::readingProcess(data);

        // RCLCPP_INFO(get_logger(),"readOnceHandler %s",test::hex_to_string(data,bytes_transferred).c_str());
        read_streambuf_.consume(bytes_transferred);
        return;
    }

    void SlcanBridge::readHandler(const boost::system::error_code &error, std::size_t bytes_transferred)
    {
        // end of file; it means usb disconnect.
        if (error == boost::asio::error::eof)
        {
            is_active_ = false;
            is_connected_ = false;
            // reconnect
            serial_port_->close();
            io_context_->reset();
            serial_port_.reset();
            work_guard_.reset();
            io_context_thread_.detach();
            // initalize asio members
            serial_port_ = std::make_shared<boost::asio::serial_port>(io_context_->get_executor());
            work_guard_ = std::make_unique<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>>(io_context_->get_executor());
            // start io_context thread
            io_context_thread_ = std::thread([this]()
                                             {
                                                 io_context_->run();
                                                 RCLCPP_INFO(this->get_logger(), "io_context_->run() is finished."); });

            initializeSerialPort(port_name_);
            asyncRead();
            handshake();
            return;
        }
        readOnceHandler(error, bytes_transferred);
        asyncRead();

        return;
    }

    // write data to the serial port. it calls asyncReadOnce() after reading.
    void SlcanBridge::asyncReadOnce()
    {
        // read and write functions can worl in the same time.
        // so, it is not necessary to use io_context_->post()      (this is a strand.)
        boost::asio::async_read_until(*serial_port_, read_streambuf_, '\0',
                                      boost::bind(&SlcanBridge::readOnceHandler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

        return;
    }

    void SlcanBridge::writeHandler(const boost::system::error_code &error, std::size_t bytes_transferred)
    {
        if (error)
        {
            RCLCPP_ERROR(get_logger(), "writeHandler error: tried to write %ld byte", bytes_transferred);

            // the followings are generated by copilot.
            // TODO:CHECK IT!
            switch (error.value())
            {
            case boost::system::errc::no_such_device_or_address:
                RCLCPP_ERROR(get_logger(), "no_such_device_or_address");
                break;
            case boost::system::errc::no_such_file_or_directory:
                RCLCPP_ERROR(get_logger(), "no_such_file_or_directory");
                break;
            case boost::system::errc::permission_denied:
                RCLCPP_ERROR(get_logger(), "permission_denied");
                break;
            case boost::system::errc::bad_file_descriptor:
                RCLCPP_ERROR(get_logger(), "bad_file_descriptor");
                break;
            case boost::system::errc::resource_unavailable_try_again:
                RCLCPP_ERROR(get_logger(), "resource_unavailable_try_again");
                break;
            default:
                RCLCPP_ERROR(get_logger(), "unknown error");
                break;
            }
        }
    }

    // write data to the serial port. it calls asyncRead() after reading.
    void SlcanBridge::asyncRead()
    {
        // read and write functions can worl in the same time.
        // so, it is not necessary to use io_context_->post()      (this is a strand.)
        boost::asio::async_read_until(*serial_port_, read_streambuf_, '\0',
                                      boost::bind(&SlcanBridge::readHandler, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));
        return;
    }

} // namespace slcan_bridge

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(slcan_bridge::SlcanBridge)