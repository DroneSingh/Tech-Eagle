#!/bin/bash

pip3 uninstall opencv-python==4.5.1.48 -y
pip3 uninstall opencv-contrib-python==4.5.1.48 -y
sudo pip3 uninstall pymavlink -y
pip3 uninstall Flask -y
pip3 uninstall logging -y
sudo rm -r -f dronekit-python 
sudo rm -r -f LANDING

