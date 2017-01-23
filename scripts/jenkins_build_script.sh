#!/bin/bash

# Lazy way to fail.
set -e

source /opt/ros/kinetic/setup.bash

export TERM=xterm-256color
export PYTHONIOENCODING=UTF-8

ARCH=`uname -m`
OS_TYPE=`uname`
INSTALL_TARGET=/usr/local/avc
DEBIAN_SCRATCH=/tmp/avc-corvette
PKG_SCRATCH=/tmp/avc-corvette

# BeagleBone Black, Ubuntu 14.04
if [[ ( "$ARCH" == "armv7l" ) && ( "$OS_TYPE" == "Linux" ) ]]; then
    CATKIN_CONFIG_FLAGS="--whitelist motor_control pru_utils pru_example"
    ARCH_STRING="armhf"
    # Packagecloud Distro ID 20 = Ubuntu 14.04.
    PC_DISTRO=20

# x86_64 Desktop, Ubuntu 16.04
elif [[ ( "$ARCH" == "x86_64" ) && ( "$OS_TYPE" == "Linux" ) ]]; then
    CATKIN_CONFIG_FLAGS=""
    ARCH_STRING="amd64"
    # Packagecloud Distro ID 165 = Ubuntu 16.04.
    PC_DISTRO=165
    
    # Explicitly add Qt5 libs the CMAKE_PREFIX_PATH
    export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:/opt/Qt/5.7/gcc_64/lib/cmake/

# x86_64 Desktop, macOS Sierra
elif [[ ( "$ARCH" == "x86_64" ) && ( "$OS_TYPE" == "Darwin" ) ]]; then
    CATKIN_CONFIG_FLAGS=""
    ARCH_STRING="x86_64"
fi

echo "Found architecture: $ARCH_STRING, $OS_TYPE"
echo "Using additional catkin config flags: $CATKIN_CONFIG_FLAGS"

catkin config --install $CATKIN_CONFIG_FLAGS
catkin build --no-status

#If we just built on an Ubuntu target, then create a debian.
if [ "$OS_TYPE" == "Linux" ]; then
    # Make sure the Debian output exists and empty.
    rm -rf $DEBIAN_SCRATCH
    mkdir -p $DEBIAN_SCRATCH/DEBIAN

    DATESTAMP=`date +"%Y%m%d"`
    FORMATTED_BUILD_ID=`printf %05d $BUILD_ID`
    MAINTAINER=`whoami`

    # Create a control file to be used with the debian.
    echo "Package: avc-corvette" >> $DEBIAN_SCRATCH/DEBIAN/control
    echo "Version: $DATESTAMP.$FORMATTED_BUILD_ID" >> $DEBIAN_SCRATCH/DEBIAN/control
    echo "Section: base" >> $DEBIAN_SCRATCH/DEBIAN/control
    echo "Priority: optional" >> $DEBIAN_SCRATCH/DEBIAN/control
    echo "Architecture: $ARCH_STRING" >> $DEBIAN_SCRATCH/DEBIAN/control
    echo "Depends: " >> $DEBIAN_SCRATCH/DEBIAN/control
    echo "Maintainer: $MAINTAINER" >> $DEBIAN_SCRATCH/DEBIAN/control
    echo "Git-Branch: ${GIT_BRANCH#*/}" >> scripts/control
    echo "Git-Commit: $GIT_COMMIT" >> scripts/control
    echo "Description: Sparkfun AVC Entry, Powerwheels Corvette" >> $DEBIAN_SCRATCH/DEBIAN/control

    # Add an postinst script.
    echo "#!/bin/bash" >> $DEBIAN_SCRATCH/DEBIAN/postinst
    echo "chmod 777 $INSTALL_TARGET/log" >> $DEBIAN_SCRATCH/DEBIAN/postinst
    chmod 0755 $DEBIAN_SCRATCH/DEBIAN/postinst

    # Copy the install output to the scratch space.
    mkdir -p $DEBIAN_SCRATCH/`dirname $INSTALL_TARGET`
    cp -ar install $DEBIAN_SCRATCH/$INSTALL_TARGET

    # Export the Git commit information to root
    echo "Branch: $GIT_BRANCH" >> $DEBIAN_SCRATCH/$INSTALL_TARGET/BUILD_INFO
    echo "Commit: $GIT_COMMIT" >> $DEBIAN_SCRATCH/$INSTALL_TARGET/BUILD_INFO
    echo "Build Version: $DATESTAMP.$FORMATTED_BUILD_ID" >> $DEBIAN_SCRATCH/$INSTALL_TARGET/BUILD_INFO

    # Make a 'log' directory. The postinst script will make sure it has the correct permissions set.
    mkdir $DEBIAN_SCRATCH/$INSTALL_TARGET/log

    # Name the Debian.
    DEBIAN_NAME="avc-corvette-${GIT_BRANCH#*/}-${GIT_COMMIT:0:7}-$ARCH_STRING.deb"
    CURL_JSON_NAME="avc-corvette-${GIT_BRANCH#*/}-${GIT_COMMIT:0:7}-$ARCH_STRING.json"

    # Actually build it.
    dpkg-deb --build $DEBIAN_SCRATCH $DEBIAN_NAME

    # If its the master branch, also push it up to packagecloud
    if [ "${GIT_BRANCH#*/}" == "master" ]; then
        curl -X POST https://$PACKAGECLOUD_PASS:@packagecloud.io/api/v1/repos/$PACKAGECLOUD_USER/avc-corvette/packages.json \
            -F "package[distro_version_id]=$PC_DISTRO"     \
            -F "package[package_file]=@$DEBIAN_NAME" -o $CURL_JSON_NAME
    fi

# Otherwise, create a macOS package installer.
elif [ "$OS_TYPE" == "Darwin" ]; then
    # Make sure the scratch space is empty, and re-create the desired structure.
    rm -rf $PKG_SCRATCH
    mkdir -p $PKG_SCRATCH/`dirname $INSTALL_TARGET`
    
    # Copy the newly built project into the scratch space.
    cp -r install $PKG_SCRATCH/$INSTALL_TARGET
    
    # Make a 'log' directory. The postinst script will make sure it has the correct permissions set.
    mkdir -p $PKG_SCRATCH/$INSTALL_TARGET/log

    # Make a post-install script.
    mkdir -p $PKG_SCRATCH/scripts
    echo "#!/bin/bash" >> $PKG_SCRATCH/scripts/postinstall
    echo "chmod 777 $INSTALL_TARGET/log" >> $PKG_SCRATCH/scripts/postinstall
    chmod 0755 $PKG_SCRATCH/scripts/postinstall
    
    DATESTAMP=`date +"%Y%m%d"`
    FORMATTED_BUILD_ID=`printf %05d $BUILD_ID`
    PKG_NAME="avc-corvette-${GIT_BRANCH#*/}-${GIT_COMMIT:0:7}-$ARCH_STRING.pkg"

    # Export the Git commit information to root
    echo "Branch: $GIT_BRANCH" >> $PKG_SCRATCH/$INSTALL_TARGET/BUILD_INFO
    echo "Commit: $GIT_COMMIT" >> $PKG_SCRATCH/$INSTALL_TARGET/BUILD_INFO
    echo "Build Version: $DATESTAMP.$FORMATTED_BUILD_ID" >> $PKG_SCRATCH/$INSTALL_TARGET/BUILD_INFO
    
    pkgbuild --root $PKG_SCRATCH --scripts $PKG_SCRATCH/scripts --identifier com.avc-corvette.${GIT_BRANCH#*/} --version 0.$FORMATTED_BUILD_ID $PKG_NAME
fi
