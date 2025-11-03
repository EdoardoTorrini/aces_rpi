#!/bin/bash

sudo apt update
sudo apt install -y ros-`ls /opt/ros`-etsi-its-msgs \
	tshark
