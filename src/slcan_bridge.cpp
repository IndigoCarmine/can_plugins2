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
namespace slcan_bridge
{

    class SlcanBridge : public rclcpp::Node{

        private:
            std::shared_ptr<boost::asio::io_context> io_context_;
            std::shared_ptr<boost::asio::serial_port> serial_port_;
            //it will prohabit the io_context to stop. 
            std::unique_ptr<boost::asio::executor_work_guard<boost::asio::io_context::executor_type>> work_guard_;


            boost::asio::streambuf read_streambuf_;


            //thread fot running io_context
            std::thread io_context_thread_; 

            //serial port connected
            bool is_connected_ = false;

            //sconection with usbcan is active
            bool is_active_ = false;

            std::unique_ptr<std::thread> reading_thread_;

            rclcpp::Publisher<can_plugins2::msg::Frame>::SharedPtr can_rx_pub_;
            rclcpp::Subscription<can_plugins2::msg::Frame>::SharedPtr can_tx_sub_;

            void canRxCallback(const can_plugins2::msg::Frame::SharedPtr msg);

            const int initialize_timeout_ = 1000;//ms
            //port open and setting.
            bool initializeSerialPort(std::string port_name);

            const int handshake_timeout_ = 1000;//ms
            //check the serial deveice is usbcan.
            bool handshake();

            void asyncWrite(std::string data);

            // convert message from usbcan, and prosece it.
            void readingProcess(std::string data);

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
        serial_port_ = std::make_shared<boost::asio::serial_port>(io_context_->get_executor(),port_name);
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

        // data structure
        /*
        uint8_t command : if it is normal can frame, it is 0x00.
        uint8_t id[4] : can id
        uint8_t frame_type :  is_rtr << 2 | is_extended << 1 | is_error
        uint8_t dlc : data length
        uint8_t data[8] : data
        */

        uint8_t data[7+8];
        data[0] = 0x00;
        data[1] = (msg->id >> 24) & 0xff;
        data[2] = (msg->id >> 16) & 0xff;
        data[3] = (msg->id >> 8) & 0xff;
        data[4] = msg->id & 0xff;
        data[5] = (msg->is_rtr << 2) | (msg->is_extended << 1) | (msg->is_error);
        data[6] = msg->dlc;
        for(int i = 0; i < 8; i++){
            data[7+i] = msg->data[i];
        }


        uint8_t output[7+8+2];
        cobs::encode(data,output,7+8);

        asyncWrite(std::string(output,output+7+8+2));
    }



    //port open and setting. 
    bool SlcanBridge::initializeSerialPort(std::string port_name){
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



    void SlcanBridge::asyncWrite(std::string data){
        io_context_->post([this,data](){
            boost::asio::async_write(*serial_port_,boost::asio::buffer(data),
            boost::bind(&SlcanBridge::readHandler,this,boost::asio::placeholders::error,boost::asio::placeholders::bytes_transferred));
        });
        return;
    }


    void SlcanBridge::readingProcess(std::string data){
        RCLCPP_INFO(get_logger(),"readingProcess");

        static uint8_t cobs_input_buffer_[128];
        static uint8_t cobs_output_buffer_[128];
        for(uint8_t i = 0;i<128 && i < data.size(); i++){
            cobs_input_buffer_[i] = data[i];
        }
        cobs_input_buffer_[data.size()] = 0;//the data lost the last byte. so, we add 1.

        cobs::decode(cobs_input_buffer_,cobs_output_buffer_,data.size()+1);

        //check it is handshake. USBCAN will send "HelloSlcan" when the connection is established.
        static const uint8_t HelloSlcan[] ={'H','e','l','l','o','S','l','c','a','n'};
        if(data.size()==10+1){
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
        auto msg = std::make_unique<can_plugins2::msg::Frame>();
        msg->id = cobs_output_buffer_[0];
        msg->is_error = cobs_output_buffer_[1];
        msg->is_extended = cobs_output_buffer_[2];
        msg->dlc = cobs_output_buffer_[3];
        for(int i = 0; i < 8; i++){
            msg->data[i] = cobs_output_buffer_[4+i];
        }
        can_rx_pub_->publish(std::move(msg));
        return;
    }



    bool SlcanBridge::handshake(){
        rclcpp::WallRate rate(1s);
        while(!is_active_){
            asyncWrite("HelloUSBCAN");
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