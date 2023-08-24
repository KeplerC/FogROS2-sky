#!/bin/bash

sudo apt-get update -y
sudo apt-get upgrade -y
sudo apt-get install git -y
mkdir -p ~/sky_ws/src
cd ~/sky_ws/src
git clone https://kushtimusPrime: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@github.com/KeplerC/FogROS2-sky.git
git clone https://kushtimusPrime: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@github.com/KeplerC/fogros2-sgc.git
cd ~/sky_ws/src/FogROS2-sky
git checkout feature/optimzation
cd ~/sky_ws
colcon build
pip install skypilot
pip3 install wgconfig boto3 paramiko scp
sudo apt update
sudo apt install wireguard-tools # TODO: remove dependency of VPN 
sudo apt install rsync -y
