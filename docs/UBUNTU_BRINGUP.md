# Setting up x86_64 Ubuntu
Start with Ubuntu 16.04.1 Server. (http://releases.ubuntu.com/16.04/ubuntu-16.04.1-server-amd64.iso)

## Install ROS
From (http://wiki.ros.org/kinetic/Installation/Ubuntu)
`sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`
`sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116`
`sudo apt-get install ros-kinetic-robot ros-kinetic-geographic-msgs ros-kinetic-roslint ros-kinetic-tf2-geometry-msgs`
`sudo rosdep init`
`rosdep update`

`sudo apt-get update`
`sudo apt-get upgrade`

`sudo apt-get install python-catkin-tools libsdl2-dev libncurses5-dev`

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

## Connecting Ubuntu console to Intel AMT SOL
Edit `/etc/default/grub`:
`GRUB_CMDLINE_LINUX_DEFAULT=""`
`GRUB_TERMINAL='serial console'`
`GRUB_CMDLINE_LINUX="console=tty0 console=ttyS4,115200n8"`
`GRUB_SERIAL_COMMAND="serial --speed=115200 --unit=0 --word=8 --parity=no --stop=1"`

And then run `sudo update-grub`

## Installing TI PRU C Compiler
32-bit libraries may need to be installed before the installer can run:
`sudo apt-get install libc6:i386`

Now we can get and run the installer.
`wget http://software-dl.ti.com/codegen/esd/cgt_public_sw/PRU/2.1.4/ti_cgt_pru_2.1.4_linux_installer_x86.bin`
`chmod +x ti_cgt_pru_2.1.4_linux_installer_x86.bin`
`sudo ./ti_cgt_pru_2.1.4_linux_installer_x86.bin --prefix /opt/ti --mode text`

## Install NVIDIA Driver
`sudo add-apt-repository ppa:graphics-drivers/ppa`
`sudo apt-get update`
`sudo apt-get install nvidia-375`

# Jenkins only info
## Cross compiling for BBB
`sudo apt-get install lxc`
`sudo lxc-create -t download -n jenkins-armhf-slave-0`
And specify `ubuntu`, `trusty`, `armhf`

`sudo lxc-start -n jenkins-armhf-slave-0`
