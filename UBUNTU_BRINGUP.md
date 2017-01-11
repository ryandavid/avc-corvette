# Setting up x86_64 Ubuntu
Start with Ubuntu 16.04.1 Server. (http://releases.ubuntu.com/16.04/ubuntu-16.04.1-server-amd64.iso)

`sudo apt-get update`
`sudo apt-get upgrade`

`sudo apt-get install python-catkin-tools libsdl2-dev libncurses5-dev`

## Install ROS
From (http://wiki.ros.org/kinetic/Installation/Ubuntu)
`sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
`sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116`
`sudo apt-get install ros-kinetic-robot ros-kinetic-geographic-msgs ros-kinetic-roslint ros-kinetic-tf2-geometry-msgs`
`sudo rosdep init`
`rosdep update`

## Add packagecloud repo
`curl -s https://packagecloud.io/install/repositories/ryandavid773/avc-corvette/script.deb.sh | sudo bash`

## Install Qt 5.7.1
Connect via SSH with X-Forwarding enabled, and then:
`wget http://qt.mirror.constant.com/official_releases/qt/5.7/5.7.1/qt-opensource-linux-x64-5.7.1.run`
`sudo mkdir /opt/Qt`
`sudo chmod 777 /opt/Qt`
`chmod +x qt-opensource-linux-x64-5.7.1.run`

Set the install location to `/opt/Qt`

Install the following:
 - Qt 5.7
     + Desktop gcc 64-bit
     + Sources
     + Qt Charts
     + Qt Data Visualization
     + Qt Web Engine
     + Qt Gamepad

