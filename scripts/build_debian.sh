#!/bin/bash

# Fancy-smancy way to get the absolute path to the parent folder,
# regardless where the script is called.
SCRIPT_DIR=$(pwd)
DIRNAME=$(dirname $0)

if [ $DIRNAME != '.' ]
then
SCRIPT_DIR=$SCRIPT_DIR/$DIRNAME
fi

PROJECT_ROOT=$(dirname $SCRIPT_DIR)


WORKDIR=$PROJECT_ROOT/debian
DESTDIR=/usr/local/avc

DEBIANDIR=$WORKDIR/DEBIAN

# Make a temporary directory and make sure its empty.
mkdir -p $WORKDIR
rm -rf $WORKDIR/*

# Make the install path underneath the temp dir.
mkdir -p $WORKDIR/$DESTDIR

# Copy everying over from the install directory.
cp -r $PROJECT_ROOT/install/* $WORKDIR/$DESTDIR

# Make the DEBIAN folder and required control file.
mkdir -p $DEBIANDIR
cp $SCRIPT_DIR/control $DEBIANDIR

# Actually build the debian.
dpkg-deb --build $WORKDIR $PROJECT_ROOT/avc-corvette.deb

# Clean up after ourselves.
rm -rf $WORKDIR
