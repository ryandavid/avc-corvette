# Setting up x86_64 Ubuntu
Start with Ubuntu 16.04.1 Server. (http://releases.ubuntu.com/16.04/ubuntu-16.04.2-server-amd64.iso)

## Install ROS and related dependencies
From (http://wiki.ros.org/kinetic/Installation/Ubuntu)
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get upgrade -y
sudo apt-get install -y ros-kinetic-robot ros-kinetic-geographic-msgs ros-kinetic-roslint ros-kinetic-tf2-geometry-msgs build-essential python-catkin-tools libsdl2-dev libncurses5-dev libc6:i386
sudo rosdep init && rosdep update
```

## Add packagecloud repo
```
curl -s https://packagecloud.io/install/repositories/ryandavid773/avc-corvette/script.deb.sh | sudo bash
```

## Install Qt 5.7.1
Install Qt to `/opt/Qt` and install the following:
Install the following:
 - Qt 5.7
     + Desktop gcc 64-bit
     + Sources
     + Qt Charts
     + Qt Data Visualization
     + Qt Web Engine
     + Qt Gamepad

```
wget http://qt.mirror.constant.com/official_releases/qt/5.7/5.7.1/qt-opensource-linux-x64-5.7.1.run
sudo mkdir /opt/Qt
sudo chmod 777 /opt/Qt
chmod +x qt-opensource-linux-x64-5.7.1.run 

# Note the path to script, path below is relative to project root.
./qt-opensource-linux-x64-5.7.1.run --script docker/ubuntu-16.04-x86/qt-installer-noninteractive.qs -platform minimal
```


## Connecting Ubuntu console to Intel AMT SOL
Edit `/etc/default/grub`:
```
GRUB_CMDLINE_LINUX_DEFAULT=""
GRUB_TERMINAL='serial console'
GRUB_CMDLINE_LINUX="console=tty0 console=ttyS4,115200n8"
GRUB_SERIAL_COMMAND="serial --speed=115200 --unit=0 --word=8 --parity=no --stop=1"
```

And then run `sudo update-grub`

## Installing TI PRU C Compiler
```
wget http://software-dl.ti.com/codegen/esd/cgt_public_sw/PRU/2.1.4/ti_cgt_pru_2.1.4_linux_installer_x86.bin
chmod +x ti_cgt_pru_2.1.4_linux_installer_x86.bin
sudo ./ti_cgt_pru_2.1.4_linux_installer_x86.bin --prefix /opt/ti --mode unattended
```

## Install NVIDIA Driver

Install NVIDIA Driver and CUDA 8.0
```
sudo sh -c 'echo "blacklist nouveau" >> /etc/modprobe.d/blacklist.conf'
sudo sh -c 'echo "blacklist lbm-nouveau" >> /etc/modprobe.d/blacklist.conf'
sudo sh -c 'echo "options nouveau modeset=0" >> /etc/modprobe.d/blacklist.conf'
sudo sh -c 'echo "alias nouveau off" >> /etc/modprobe.d/blacklist.conf'
sudo sh -c 'echo "alias lbm-nouveau off" >> /etc/modprobe.d/blacklist.conf'
sudo update-initramfs -u
```
You will then need to reboot.

```
wget https://developer.nvidia.com/compute/cuda/8.0/Prod2/local_installers/cuda_8.0.61_375.26_linux-run
sudo chmod +x ./cuda_8.0.61_375.26_linux-run
sudo ./cuda_8.0.61_375.26_linux-run  --silent --driver --toolkit --verbose
```

# Jenkins only info
## Cross compiling for BBB
WIP! Setting up a LXC container to cross-compile for BBB.
```
sudo apt-get install lxc
sudo lxc-create -t download -n jenkins-armhf-slave-0
# And specify `ubuntu`, `trusty`, `armhf`
```

Start the container via `sudo lxc-start -n jenkins-armhf-slave-0`
