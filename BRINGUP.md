# Setting up BeagleBone Black for motor control.
Use the following Ubuntu image.  I couldn't get the newer Ubuntu 16.04 images to work nicely with the SPI kernel modules.
(https://rcn-ee.online/rootfs/2014-06-05/flasher/BBB-eMMC-flasher-ubuntu-14.04-console-2014-06-05-2gb.img.xz)

### Disable the HDMI 'cape'
`sudo nano /boot/uboot/uEnv.txt` and uncomment:
`capemgr.disable_partno=BB-BONELT-HDMI,BB-BONELT-HDMIN`

### Disable USB gadget
`sudo nano /opt/scripts/boot/am335x_evm.sh`

### Compile the Device Tree Overlays
`wget -c https://raw.github.com/RobertCNelson/tools/master/pkgs/dtc.sh`
`chmod +x dtc.sh` 
`./dtc.sh`
`wget -O BB-SPI0-01-00A0.dts http://elinux.org/images/1/1f/BB-SPI0-01-00A0.txt`
`wget -O BB-SPI1-01-00A0.dts http://elinux.org/images/b/b5/BB-SPI1-STD-01-00A0.txt`

`dtc -O dtb -o BB-SPI0-01-00A0.dtbo -b 0 -@ BB-SPI0-01-00A0.dts`
`dtc -O dtb -o BB-SPI1-01-00A0.dtbo -b 0 -@ BB-SPI1-01-00A0.dts`

`sudo cp *.dtbo /lib/firmware`

### Enable SPIDEV on boot
The right way to do this is add it to the uEnv.txt, but I can't seem to compile the DTCs into the initramfs. Instead:
`sudo nano /etc/rc.local`
and add the following:
`sudo sh -c "echo BB-SPI0-01 > /sys/devices/bone_capemgr.9/slots"`
`sudo sh -c "echo BB-SPI1-01 > /sys/devices/bone_capemgr.9/slots"`

## Update and install deps
`sudo apt-get update && sudo apt-get upgrade -y && sudo apt-get install -y python-pip sbcl-source sbcl-doc liburiparser1`

## GPIO udev
Set permissions of the GPIO so root is not required. From (http://www.embeddedhobbyist.com/2016/05/beaglebone-black-gpio-permissions/)

`sudo nano /etc/udev/rules.d/99-gpio.rules`
and add the following:
`SUBSYSTEM=="gpio*", PROGRAM="/bin/sh -c 'chown -R root:gpio /sys/class/gpio;chmod -R 770 /sys/class/gpio; chown -R root:gpio /sys/devices/virtual/gpio/;chmod -R 770 /sys/devices/virtual/gpio/'"`

## Free up space
ROS takes up a ton of room. Remove unecessary packages.
`sudo apt-get purge wireless-tools wpasupplicant wvdial rfkill ppp manpages* hdparm apache2* x11-common avahi-daemon`
`sudo apt-get autoremove`

### Delete locales, man pages, and docs
`nano /etc/dpkg/dpkg.cfg.d/01_nodoc` and add the following:
`path-exclude=/usr/share/locale/*`
`path-exclude=/usr/share/man/*`
`path-exclude=/usr/share/doc/*`
`path-include=/usr/share/doc/*/copyright`

Remove all existing docs, man pages, and locales.
`sudo rm -rf /usr/share/doc/*`
`sudo rm -rf /usr/share/man/*`
`sudo rm -rf /usr/share/locale/*`

## Install ROS
From http://wiki.ros.org/kinetic/Installation/Source

`sudo pip install -U rosdep rosinstall_generator wstool rosinstall rospkg catkin_tools`
`sudo rosdep init`
`rosdep update`

`mkdir ~/ros_catkin_ws`
`cd ~/ros_catkin_ws`

Only install the robot variant in order to save space.
`rosinstall_generator robot tf2_geometry_msgs uuid_msgs geographic_msgs --rosdistro kinetic --deps --wet-only --tar > kinetic-robot-wet.rosinstall`
`wstool init -j2 src kinetic-robot-wet.rosinstall`
`rosdep install --from-paths src --ignore-src --rosdistro kinetic -y --skip-keys="python-qt5-bindings sbcl python-rospkg python-rosdep python-catkin-pkg collada-dom"`
`rm -rf src/robot_model src/robot_state_publisher`
`sudo mkdir -p /opt/ros/kinetic`
`sudo chown ubuntu /opt/ros/kinetic`
`./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic`

Add ROS env to bashrc
`echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc`
