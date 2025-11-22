#!/bin/bash

sudo apt-get update && sudo apt-get install raspi-config i2c-tools -y 
pip install lgpio adafruit-circuitpython-gps RPi.GPIO adafruit-circuitpython-bno055 digi-xbee pyubx2 sparkfun‑ublox_gps --break-system-packages
sudo raspi-config
