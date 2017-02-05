From http://elinux.org/Ti_AM33XX_PRUSSv2#Beaglebone_PRU_connections_and_modes

BB Header|PRU|R30 (Output) bit|Pinmux Mode|R31 (Input) bit|Pinmux Mode|Offset Register
---------|---|----------------|-----------|---------------|-----------|---------------
P8_11|0|15|Mode 6|N/A|N/A|0x834
P8_12|0|14|Mode 6|N/A|N/A|0x830
P8_15|0|N/A|N/A|15|Mode 6|0x83c
P8_16|0|N/A|N/A|14|Mode 6|0x838


### Verifying Pinmux
sudo cat /sys/kernel/debug/pinctrl/44e10800.pinmux/pinmux-pins | grep pruss
