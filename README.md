can_plugins2
====
## Description
It is for ysk's USBCAN board, is incompatible with the old version.
It works on ros2 Humble Hawksbill.
can_plugins2 is for new version of usbcan_fw. It is not compatible with old version.
So, you should change new udev rule. See Install section.




## Usage

you should refer to [omuni_example](https://github.com/IndigoCarmine/omuni_example).
you write launch file like omuni.launch.

## Install

copy udev rule
```
sudo cp ~/catkin_ws/src/can_plugins2/udev/60-usbcan.rules /etc/udev/rules.d/60-usbcan.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```
