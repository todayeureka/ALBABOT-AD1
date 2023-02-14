#!/bin/bash

echo "remap the device serial port(ttyUSBX) to  albabot"
echo "albabot usb connection as /dev/albabot , check it using the command : ls -l /dev|grep ttyUSB"
SCRIPT_PATH=$(pwd)
echo "$SCRIPT_PATH/scripts/90-usbserial-albabot.rules"
sudo cp $SCRIPT_PATH/90-usbserial-albabot.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo service udev reload
sudo service udev restart
echo "Please restart computer"
