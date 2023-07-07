#!/bin/bash

pip3 install opencv-python==4.5.1.48
pip3 install opencv-contrib-python==4.5.1.48
pip3 install pymavlink
pip3 install Flask
#pip3 install logging
git clone https://github.com/dronekit/dronekit-python.git
cd ./dronekit-python
sudo python setup.py build
sudo python setup.py install
git clone https://github.com/shikhar2624/LANDING.git

