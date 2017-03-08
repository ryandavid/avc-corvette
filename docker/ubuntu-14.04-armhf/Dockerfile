FROM armhf/ubuntu:14.04

RUN sudo apt-get update && \
    sudo apt-get upgrade -y && \
    sudo apt-get install -y sbcl-source sbcl-doc liburiparser1 libsdl2-dev libncurses5-dev build-essential

ADD https://bootstrap.pypa.io/get-pip.py /tmp
RUN sudo python /tmp/get-pip.py && \
    rm /tmp/get-pip.py

RUN sudo pip install -U rosdep rosinstall_generator wstool rosinstall rospkg catkin_tools
RUN rosdep init && \
    rosdep update

RUN mkdir /ros_catkin_ws && \
    cd /ros_catkin_ws

RUN rosinstall_generator robot tf2_geometry_msgs uuid_msgs geographic_msgs \
    --rosdistro kinetic --deps --wet-only --tar > kinetic-robot-wet.rosinstall && \
    wstool init -j2 src kinetic-robot-wet.rosinstall && \
    rosdep install --from-paths src --ignore-src --rosdistro kinetic -y \
    --skip-keys="python-qt5-bindings sbcl python-rospkg python-rosdep python-catkin-pkg collada-dom" && \
    rm -rf src/robot_model src/robot_state_publisher

RUN sudo mkdir -p /opt/ros/kinetic

RUN ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release --install-space /opt/ros/kinetic

RUN echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc

ADD http://software-dl.ti.com/codegen/esd/cgt_public_sw/PRU/2.1.4/ti_cgt_pru_2.1.4_armlinuxa8hf_busybox_installer.sh /tmp
RUN chmod +x /tmp/ti_cgt_pru_2.1.4_armlinuxa8hf_busybox_installer.sh && \
    /tmp/ti_cgt_pru_2.1.4_armlinuxa8hf_busybox_installer.sh && \
    rm /tmp/ti_cgt_pru_2.1.4_armlinuxa8hf_busybox_installer.sh
