#!/bin/bash

echo "remap the device serial port(ttyACM*) to  ttyCAN"
echo "rplidar usb connection as /dev/rplidar , check it using the command : ls -l /dev|grep ttyACM"
echo "start copy can.rules to  /etc/udev/rules.d/"
echo "`rospack find charlie`/scripts/can_motor/can.rules"
sudo cp `rospack find charlie`/scripts/can_motor/can.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
sudo udevadm control --reload && sudo udevadm trigger
echo "finish "
