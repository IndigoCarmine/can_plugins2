#include <ros/ros.h>
#include "std_msgs/Float64.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt8.h"
#include "can_utils.hpp"
#include <can_plugins/Frame.h>

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace can_plugins{
  
  class Ping : public nodelet::Nodelet
  {
  public:
    virtual void onInit();
  private:
    double ctrl_freq;
    int count_rx;
    int count_tx;
    int id_tx;
    int id_rx;
    int num=0;
    float avg=0;

    void timer_callback(const ros::TimerEvent &);
    void timeout_callback(const ros::TimerEvent &);
    void canRxCallback(const can_plugins::Frame::ConstPtr &msg);
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Publisher tx_pub_;
    ros::Subscriber rx_sub_;

    ros::Publisher _can_tx_pub;
    ros::Subscriber _can_rx_sub;

    ros::Timer timer;
    ros::Timer timeout;
  };

  void Ping::onInit()
  {
    nh_ = getMTNodeHandle();
    private_nh_ = getMTPrivateNodeHandle();

    count_tx = 0;
    count_rx = 0;
    ctrl_freq = 1000;
    id_tx = 0;
    private_nh_.getParam("ctrl_freq", ctrl_freq);
    private_nh_.getParam("id_tx", id_tx);

    id_rx = id_tx + 2;

    _can_tx_pub				    = nh_.advertise<can_plugins::Frame>("can_tx", 1000);
    _can_rx_sub				    = nh_.subscribe<can_plugins::Frame>("can_rx", 1000, &Ping::canRxCallback, this);

    timeout = nh_.createTimer(ros::Duration(1),&Ping::timeout_callback, this);
  }

  void Ping::timer_callback(const ros::TimerEvent &)
{
  uint8_t GetStatus = 0x11;
  _can_tx_pub.publish(get_frame(id_tx,GetStatus));
  count_tx += 1;
  if (count_tx >= 1000)
    timer.stop();
}

void Ping::timeout_callback(const ros::TimerEvent &)
{

  if(0 < num){
    NODELET_INFO("%d packs transmitted, %d received", count_tx,count_rx);
    avg += (double)count_rx/(double)count_tx;
    count_rx = 0;
    count_tx = 0;
  }
  if(num < 10){
    timer = nh_.createTimer(ros::Duration(1 / ctrl_freq), boost::bind(&Ping::timer_callback, this, _1));
    num++;
  }
  else
  {
    NODELET_INFO("%f%% packet lost", (1-avg/10)*100);
    timeout.stop();
  }
  
}

void Ping::canRxCallback(const can_plugins::Frame::ConstPtr &msg)
{
  if(msg->id == id_rx)
  {
    count_rx += 1;
  }
}

}// namespace can_plugins
PLUGINLIB_EXPORT_CLASS(can_plugins::Ping, nodelet::Nodelet);