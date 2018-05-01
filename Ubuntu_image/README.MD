## Prepare Ubuntu image

1. Download Lubuntu 16.04: https://ubuntu-pi-flavour-maker.org/download/

2. Configuration with first boot with HDMI:
```sh
raspi-config
```
Enable ssh, disable GUI

3. Delete GUI:
```sh
sudo apt purge --remove lubuntu-*; sudo apt autoremove -y
```

4. Enable UART. Add to /boot/config.txt:
```sh
enable_uart=1
core_freq=250
dtoverlay=pi3-disable-bt
```
Delete in /boot/cmdline.txt:
```sh
console=serial0,115200
```

5. WiFi AP setup:
```sh
./rPi3-ap-setup.sh
```

6. ROS install:<br>
http://wiki.ros.org/kinetic/Installation/Ubuntu<br>
http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

7. Install IPFS:
```sh
git clone https://github.com/claudiobizzotto/ipfs-rpi.git
```

8. Install cjdns and NodeJS:<br>
https://github.com/cjdelisle/cjdns/blob/master/doc/install/ubuntu-16.04.md
```sh
curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash -
sudo apt-get install -y nodejs
git clone https://github.com/cjdelisle/cjdns.git
NO_TEST=1 Seccomp_NO=1 ./do
```
Add peer to cjdroute.conf.

9. Drone Employee air pollution sensors installation:
```sh
cd ~/catkin_ws/src
git clone https://github.com/tuuzdu/DJI-Air-pollution-sensors-project
git submdule init
git submodule update
pip install pyserial ipfsapi
```
Build dji_sdk: <br>
http://wiki.ros.org/dji_sdk/Tutorials/Getting%20Started
