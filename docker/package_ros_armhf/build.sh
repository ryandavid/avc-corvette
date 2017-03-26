#!/bin/bash

INSTALL_TARGET=/opt/ros/kinetic
DEBIAN_SCRATCH=/tmp/ros-packaging

# Make sure the Debian output exists and empty.
rm -rf $DEBIAN_SCRATCH
mkdir -p $DEBIAN_SCRATCH/DEBIAN

DATESTAMP=`date +"%Y%m%d"`
MAINTAINER=`whoami`

# Create a control file to be used with the debian.
echo "Package: ros-kinetic" >> $DEBIAN_SCRATCH/DEBIAN/control
echo "Version: $DATESTAMP" >> $DEBIAN_SCRATCH/DEBIAN/control
echo "Section: base" >> $DEBIAN_SCRATCH/DEBIAN/control
echo "Priority: optional" >> $DEBIAN_SCRATCH/DEBIAN/control
echo "Architecture: armhf" >> $DEBIAN_SCRATCH/DEBIAN/control
echo "Depends: cmake, curl, graphviz, hddtemp, libapr1-dev, libaprutil1-dev, libassimp-dev, libboost-all-dev, libbz2-dev, libconsole-bridge-dev, libcppunit-dev, libcurl4-openssl-dev, libeigen3-dev, libgtest-dev, liblog4cxx10-dev, liblz4-dev, libpoco-dev, libqhull-dev, libtinyxml-dev, pkg-config, python-coverage, python-defusedxml, python-dev, python-empy, python-imaging, python-mock, python-netifaces, python-nose, python-numpy, python-paramiko, python-sip-dev, python-urlgrabber, python-yaml, qtbase5-dev, uuid-dev" >> $DEBIAN_SCRATCH/DEBIAN/control
echo "Maintainer: $MAINTAINER" >> $DEBIAN_SCRATCH/DEBIAN/control
echo "Description: ROS Kinetic for armhf" >> $DEBIAN_SCRATCH/DEBIAN/control

# Add an postinst script.
echo "#!/bin/bash" >> $DEBIAN_SCRATCH/DEBIAN/postinst
echo "chmod 777 $INSTALL_TARGET/log" >> $DEBIAN_SCRATCH/DEBIAN/postinst
chmod 0755 $DEBIAN_SCRATCH/DEBIAN/postinst

# Copy the install output to the scratch space.
mkdir -p $DEBIAN_SCRATCH/`dirname $INSTALL_TARGET`
cp -ar $INSTALL_TARGET $DEBIAN_SCRATCH/$INSTALL_TARGET

# Export the Build commit information to root
echo "Build Version: $DATESTAMP" >> $DEBIAN_SCRATCH/$INSTALL_TARGET/BUILD_INFO

# Make a 'log' directory. The postinst script will make sure it has the correct permissions set.
mkdir $DEBIAN_SCRATCH/$INSTALL_TARGET/log

# Name the Debian.
DEBIAN_NAME="ros-kinetic-armhf-${DATESTAMP}.deb"
#CURL_JSON_NAME="avc-corvette-${GIT_BRANCH#*/}-${GIT_COMMIT:0:7}-$ARCH_STRING.json"

# Actually build it.
dpkg-deb --build $DEBIAN_SCRATCH $DEBIAN_NAME

# If its the master branch, also push it up to packagecloud
#if [ "${GIT_BRANCH#*/}" == "master" ]; then
#    curl -X POST https://$PACKAGECLOUD_PASS:@packagecloud.io/api/v1/repos/$PACKAGECLOUD_USER/avc-corvette/packages.json \
#        -F "package[distro_version_id]=$PC_DISTRO"     \
#        -F "package[package_file]=@$DEBIAN_NAME" -o $CURL_JSON_NAME
#fi
