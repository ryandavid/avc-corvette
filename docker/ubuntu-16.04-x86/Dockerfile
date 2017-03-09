FROM ubuntu:16.04

# Need to install lsb-release before doing ROS install.
RUN apt-get update && \
    apt-get install -y lsb-release

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116 && \
    apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y ros-kinetic-robot ros-kinetic-geographic-msgs ros-kinetic-roslint ros-kinetic-tf2-geometry-msgs \
                            python-catkin-tools libsdl2-dev libncurses5-dev libc6-i386

RUN rosdep init && \
    rosdep update && \
    echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

## Install Qt 5.7.1
ADD http://qt.mirror.constant.com/official_releases/qt/5.7/5.7.1/qt-opensource-linux-x64-5.7.1.run /tmp
ADD qt-installer-noninteractive.qs /tmp
RUN mkdir -p /opt/Qt && \
    chmod 777 /opt/Qt && \
    chmod +x /tmp/qt-opensource-linux-x64-5.7.1.run && \
    /tmp/qt-opensource-linux-x64-5.7.1.run --script /tmp/qt-installer-noninteractive.qs -platform minimal && \
    rm /tmp/qt-opensource-linux-x64-5.7.1.run /tmp/qt-installer-noninteractive.qs


# Install TI PRU C Compiler
ADD http://software-dl.ti.com/codegen/esd/cgt_public_sw/PRU/2.1.4/ti_cgt_pru_2.1.4_linux_installer_x86.bin /tmp
RUN mkdir -p /opt/ti && \
    chmod +x /tmp/ti_cgt_pru_2.1.4_linux_installer_x86.bin && \
    /tmp/ti_cgt_pru_2.1.4_linux_installer_x86.bin --prefix /opt/ti --mode unattended

