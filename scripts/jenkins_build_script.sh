#!/bin/bash

# Lazy way to fail.
set -e

source /opt/ros/kinetic/setup.bash

export ARCH=`uname -m`
export TERM=xterm-256color
export PYTHONIOENCODING=UTF-8
export INSTALL_TARGET=/usr/local/avc
export DEBIAN_SCRATCH=/tmp/avc-corvette

# Only build the required nodes on the Beaglebones.
if [ "$ARCH" == "armv7l" ]; then
    CATKIN_CONFIG_FLAGS="--whitelist motor_control"
    ARCH_STRING="armhf"
    # Packagecloud Distro ID 20 = Ubuntu 14.04.
    PC_DISTRO=20
else
    CATKIN_CONFIG_FLAGS=""
    ARCH_STRING="amd64"
    # Packagecloud Distro ID 165 = Ubuntu 16.04.
    PC_DISTRO=165
fi

# Empty the contents of the install output.
rm -rf $INSTALL_TARGET/*

# Make sure the Debian output exists and empty.
rm -rf $DEBIAN_SCRATCH
mkdir -p $DEBIAN_SCRATCH/DEBIAN

echo "Found architecture: $ARCH_STRING"
echo "Configuring catkin with: $CATKIN_CONFIG_FLAGS"

catkin config --install -i $INSTALL_TARGET $CATKIN_CONFIG_FLAGS
catkin build --no-status

# Create a control file to be used with the debian.
echo "Package: avc-corvette" >> $DEBIAN_SCRATCH/DEBIAN/control
echo "Version: $BUILD_ID" >> $DEBIAN_SCRATCH/DEBIAN/control
echo "Section: base" >> $DEBIAN_SCRATCH/DEBIAN/control
echo "Priority: optional" >> $DEBIAN_SCRATCH/DEBIAN/control
echo "Architecture: $ARCH_STRING" >> $DEBIAN_SCRATCH/DEBIAN/control
echo "Depends: " >> $DEBIAN_SCRATCH/DEBIAN/control
echo "Maintainer: jenkins" >> $DEBIAN_SCRATCH/DEBIAN/control
echo "Description: Sparkfun AVC Entry, Powerwheels Corvette" >> $DEBIAN_SCRATCH/DEBIAN/control
echo "Git-Branch: ${GIT_BRANCH#*/}" >> scripts/control
echo "Git-Commit: $GIT_COMMIT" >> scripts/control
echo "" >> scripts/control

# Copy the install output to the scratch space.
mkdir -p $DEBIAN_SCRATCH/`dirname $INSTALL_TARGET`
cp -ar $INSTALL_TARGET $DEBIAN_SCRATCH/$INSTALL_TARGET

# Name the Debian.
DEBIAN_NAME="avc-corvette-${GIT_BRANCH#*/}-${GIT_COMMIT:0:7}-$ARCH_STRING.deb"
CURL_JSON_NAME="avc-corvette-${GIT_BRANCH#*/}-${GIT_COMMIT:0:7}-$ARCH_STRING.json"

# Actually build it.
dpkg-deb --build $DEBIAN_SCRATCH $DEBIAN_NAME

#If we just built master, then push up to packagecloud.
if [ "${GIT_BRANCH#*/}" == "master" ]; then
    curl -X POST https://$PACKAGECLOUD_PASS:@packagecloud.io/api/v1/repos/$PACKAGECLOUD_USER/avc-corvette/packages.json \
        -F "package[distro_version_id]=$PC_DISTRO"     \
        -F "package[package_file]=@$DEBIAN_NAME" -o $CURL_JSON_NAME
fi
