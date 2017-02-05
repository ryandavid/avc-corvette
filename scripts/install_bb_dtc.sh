#!/bin/bash

set -e

dtc -O dtb -I dts -o /lib/firmware/BB-PRU-GPIO-00A0.dtbo -b 0 -@ ./device-tree-overlays/BB-PRU-GPIO-00A0.dts
dtc -O dtb -I dts -o /lib/firmware/BB-SPI0-01-00A0.dtbo -b 0 -@ ./device-tree-overlays/BB-SPI0-01-00A0.dts
dtc -O dtb -I dts -o /lib/firmware/BB-SPI1-01-00A0.dtbo -b 0 -@ ./device-tree-overlays/BB-SPI1-01-00A0.dts

echo "Successfully installed Device Tree Overlays."
