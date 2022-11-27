can_plugins 
====
## Description
yskharaさんのUSBCAN基板を採用しつつ、よりよいコードでかけるようにします。通信方式に互換性がありません。
### やったこと
- ros2 Humble Hawksbill

### やりたいこと
- 


Attention!!!!!!!!!!
The following is the can_plugins content

## Usage
```xml
<?xml version="1.0" encoding="UTF-8"?>
<launch>
  <arg name="manager_name" default="nodelet_manager" />
  <arg name="nodelet_mode" default="standalone" /><!-- set to standalone if you want to use as node-->
  <!-- Nodelet Manager -->
  <group if="$(eval nodelet_mode=='load')">
    <node pkg="nodelet" type="nodelet" name="$(arg manager_name)" args="manager" output="screen"/>
  </group>
  <!-- CAN -->
  <node pkg="nodelet" type="nodelet" name="slcan_bridge" 
  args="$(arg nodelet_mode) can_plugins/SlcanBridge $(arg manager_name)" output="screen"/>


</launch>
```

## Install
依存ファイルのダウンロード
```
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src
```

udevファイルをコピーする
```
sudo cp ~/catkin_ws/src/can_plugins/udev/60-usbcan.rules /etc/udev/rules.d/60-usbcan.rules
sudo udevadm control --reload-rules && udevadm trigger
```
