#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>
#include <string>
#include <boost/array.hpp>
#include <boost/bind/bind.hpp>
#include <chrono>
#include <future>
#include "cobs.hpp"

#include "can_plugins2/msg/frame.hpp"
using namespace std::chrono_literals;
using namespace std::placeholders;


namespace slcan_command
{
    enum Command:uint8_t{
        Normal=0,
        Negotiation=1,

    };
} // namespace slcan_command



namespace slcan_bridge
{

    class SlcanBridge : public rclcpp::Node{
        private:
            /////////////Slacan Status///////////////////
            //serial port connected but "HelloSlcan" has not been returned yet.
            bool is_connected_ = false;
            //sconection with usbcan is active. the serial port is new usbcan.
            bool is_active_ = false;
            /////////////Slacan Status///////////////////



            std::shared_ptr<boost::asio::io_context> io_context_;
            std::shared_ptr<boost::asio::serial_port> serial_port_;
            //it will prohabit the io_context to stop. 
            std::unique_ptr<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>> work_guard_;

            boost::asio::streambuf read_streambuf_;

            //thread fot running io_context
            std::thread io_context_thread_; 

            std::unique_ptr<std::thread> reading_thread_;

            rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr can_rx_pub_;
            rclcpp::Subscription<can_plugins2::msg::Frame>::SharedPtr can_tx_sub_;


            void canRxCallback(const can_plugins2::msg::Frame::SharedPtr msg);

            const int initialize_timeout_ = 1000;//ms
            //port open and setting.
            bool initializeSerialPort(const std::string port_name);



            const int handshake_timeout_ = 1000;//ms
            //check the serial deveice is usbcan.
            bool handshake();


            // convert message from usbcan, and process it.
            void readingProcess(const std::string data);


            void asyncWrite(const can_plugins2::msg::Frame::SharedPtr msg);
            void asyncWrite(const slcan_command::Command command,const std::string data);

            //Directly use is deprecated.
            void asyncWrite(const std::string data);
            void asyncWrite(const uint8_t data[],int len);

            //you should call this function at once after the connection is established.
            void asyncRead();

            void asyncReadOnce();

            //these function will be called when the data is read from the serial port.
            void readOnceHandler(const boost::system::error_code& error, std::size_t bytes_transferred);
            void readHandler(const boost::system::error_code& error, std::size_t bytes_transferred);


        public:
            SlcanBridge(const rclcpp::NodeOptions & options);
            boost::array<unsigned char, 32> receive_api_frame_;

            //shutfdown process
            void onShutdown(){
                //it generates copilot. so, it may have some problems. TODO:::CHECK
                is_active_ = false;
                io_context_->stop();
                io_context_thread_.join();
                serial_port_->close();
                RCLCPP_INFO(this->get_logger(),"END");
            }
            
    };

    SlcanBridge::SlcanBridge(const rclcpp::NodeOptions & options):Node("slcan_bridge",options){
        rclcpp::on_shutdown([this](){this->onShutdown();});
        can_rx_pub_ = this->create_publisher<can_plugins2::msg::Frame>("can_rx",10);
        can_tx_sub_ = this->create_subscription<can_plugins2::msg::Frame>("can_tx",10,std::bind(&SlcanBridge::canRxCallback,this,_1));

        std::string port_name = "/dev/usbcan";

        //initalize asio members
        io_context_ = std::make_shared<boost::asio::io_context>();
        serial_port_ = std::make_shared<boost::asio::serial_port>(io_context_->get_executor());
        work_guard_ = std::make_unique<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>>(io_context_->get_executor());


        //start io_context thread
        io_context_thread_ = std::thread([this](){
            io_context_->run();
        });


        RCLCPP_INFO(get_logger(),"SlcanBridge is initialized.");

        bool i =  initializeSerialPort(port_name);
        if(i){
            asyncRead();

        }
        
    }

    void SlcanBridge::canRxCallback(const can_plugins2::msg::Frame::SharedPtr msg){
        if(!is_active_){
            return;
        }
        asyncWrite(msg);
    }



    //port open and setting. 
    bool SlcanBridge::initializeSerialPort(const std::string port_name){
        rclcpp::WallRate rate(1s);
        while(true){
            try{
                serial_port_->open(port_name);
                serial_port_->set_option(boost::asio::serial_port_base::character_size(8));
                serial_port_->set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
                serial_port_->set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
                serial_port_->set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
                serial_port_->set_option(boost::asio::serial_port_base::baud_rate(115200));
            }catch(boost::system::system_error e){
                switch (e.code().value()){
                    case 2:
                        RCLCPP_ERROR(get_logger(),"Cannot connect. No such file or directory");
                        break;
                    case 13:
                        RCLCPP_ERROR(get_logger(),"Cannot connect. Permission denied");
                        break;
                    default:
                        RCLCPP_ERROR(get_logger(),"Cannot connect. Unknown error");
                        break;
                }
            }
            if(serial_port_->is_open()){
                RCLCPP_INFO(get_logger(),"connected");
                is_connected_ = true;
                break;
            }
            rate.sleep();

        }
        return true;
    }



    void SlcanBridge::asyncWrite(const std::string data){
        io_context_->post([this,data](){
            boost::asio::async_write(*serial_port_,boost::asio::buffer(data),
            boost::bind(&SlcanBridge::readHandler,this,boost::asio::placeholders::error,boost::asio::placeholders::bytes_transferred));
        });
        return;
    }
    void SlcanBridge::asyncWrite(const uint8_t data[],int len){
        io_context_->post([this,data,len](){
            boost::asio::async_write(*serial_port_,boost::asio::buffer(data,len),
            boost::bind(&SlcanBridge::readHandler,this,boost::asio::placeholders::error,boost::asio::placeholders::bytes_transferred));
        });
        return;
    }


    void SlcanBridge::asyncWrite(const can_plugins2::msg::Frame::SharedPtr msg){
        // data structure
        /*
        uint8_t command & frame_type: (command: if it is normal can frame, it is 0x00.)<<4 | is_rtr << 2 | is_extended << 1 | is_error
        uint8_t id[4] : can id
        uint8_t dlc : data length
        uint8_t data[8] : data
        */
        uint8_t data[6+8];
        data[0] = (msg->is_rtr << 2) | (msg->is_extended << 1) | (msg->is_error);
        data[6] = msg->dlc;
        data[1] = (msg->id >> 24) & 0xff;
        data[2] = (msg->id >> 16) & 0xff;
        data[3] = (msg->id >> 8) & 0xff;
        data[4] = msg->id & 0xff;
        for(int i = 0; i < 8; i++){
            data[6+i] = msg->data[i];
        }
        uint8_t output[6+8+2];
        cobs::encode(data,output,6+8);
        asyncWrite(std::string(output,output+6+8+2));
    }

    void SlcanBridge::asyncWrite(const slcan_command::Command command,const std::string data){
        if(command == slcan_command::Normal)
            RCLCPP_ERROR(get_logger(),"asyncWrite(Command) can not use normal. you need to use asyncWrite(Frame)");

        // data structure
        /*
        uint8_t command & frame_type: (command: if it is normal can frame, it is 0x00.)<<4 | is_rtr << 2 | is_extended << 1 | is_error
        uint8_t id[4] : data
        */
        uint8_t raw_data[100];
        raw_data[0]= command<<4;
        int counter = 1;
        for(char a : data){
            raw_data[counter] = static_cast<uint8_t>(a);
            counter++;
        }
        uint8_t output[101];
        cobs::encode(raw_data,output,data.length()+1+2);
        asyncWrite(std::string(output,output+data.length()+1+2));
    }


    void SlcanBridge::readingProcess(const std::string data){
        RCLCPP_INFO(get_logger(),"readingProcess");

        static uint8_t cobs_input_buffer_[128];
        static uint8_t cobs_output_buffer_[128];
        for(uint8_t i = 0;i<128 && i < data.size(); i++){
            cobs_input_buffer_[i] = data[i];
        }
        cobs_input_buffer_[data.size()] = 0;//the data lost the last byte. so, we add 1.

        cobs::decode(cobs_input_buffer_,cobs_output_buffer_,data.size()+1);

        //check it is handshake. USBCAN will send "HelloSlcan" when the connection is established.
        static const uint8_t HelloSlcan[] ={slcan_command::Negotiation<<4, 'H','e','l','l','o','S','l','c','a','n'};
        if(data.size()==11+1){
            bool is_handshake = true;
            for(int i = 0; i < 10; i++){
                if(cobs_output_buffer_[i] != HelloSlcan[i]){
                    is_handshake = false;
                    break;
                }
            }
            if(is_handshake){
                RCLCPP_INFO(get_logger(),"handshake");
                is_active_ = true;
                return;
            }
        }





        //publish the data to the topic.
        if(data.size()<12){
            RCLCPP_ERROR(get_logger(),"data size is too small");
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
        msg->is_error = cobs_output_buffer_[0]&0x1;
        msg->is_extended = cobs_output_buffer_[0]>>1&0x1;
        msg->is_rtr = cobs_output_buffer_[0]>>2&0x1;
        msg->id = cobs_output_buffer_[1]<<12|cobs_output_buffer_[2]<<8|cobs_output_buffer_[3]<<4|cobs_output_buffer_[4];
        msg->dlc = cobs_output_buffer_[5];
        for(int i = 0; i < 8; i++){
            msg->data[i] = cobs_output_buffer_[4+i];
        }
        can_rx_pub_->publish(std::move(msg));
        return;
    }



    bool SlcanBridge::handshake(){
        rclcpp::WallRate rate(1s);
        while(!is_active_){
            asyncWrite(slcan_command::Negotiation,"HelloUSBCAN");
            rate.sleep();
        }
        return true;
    }


    void SlcanBridge::readOnceHandler(const boost::system::error_code& error, std::size_t bytes_transferred){
        if(error){
            RCLCPP_ERROR(get_logger(),"readOnceHandler error");
            return;
        }
        std::string data = boost::asio::buffer_cast<const char*>(read_streambuf_.data());
        SlcanBridge::readingProcess(data);
        RCLCPP_INFO(get_logger(),"readOnceHandler %s",data.c_str());
        read_streambuf_.consume(bytes_transferred);
        return;
    }

    void SlcanBridge::readHandler(const boost::system::error_code& error, std::size_t bytes_transferred){
        readOnceHandler(error,bytes_transferred);
        asyncRead();
        return;
    }

    //write data to the serial port. it calls asyncReadOnce() after reading.
    void SlcanBridge::asyncReadOnce(){
        //read and write functions can worl in the same time.
        //so, it is not necessary to use io_context_->post()      (this is a strand.)
        boost::asio::async_read_until(*serial_port_,read_streambuf_,'\0',
            boost::bind(&SlcanBridge::readOnceHandler,this,boost::asio::placeholders::error,boost::asio::placeholders::bytes_transferred));

        return;
    }

    //write data to the serial port. it calls asyncRead() after reading.
    void SlcanBridge::asyncRead(){
        //read and write functions can worl in the same time.
        //so, it is not necessary to use io_context_->post()      (this is a strand.)
        boost::asio::async_read_until(*serial_port_,read_streambuf_,'\0',
            boost::bind(&SlcanBridge::readHandler,this,boost::asio::placeholders::error,boost::asio::placeholders::bytes_transferred));
        return;
    }

    

} // namespace slcan_bridge


#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(slcan_bridge::SlcanBridge)