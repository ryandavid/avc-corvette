#!/usr/bin/env python

from amt import client, wsman

# Hosts
# richard - Beaglebone Black
# james - BeagleBone Black
# jeremy - x86

HOSTS = {
    "jeremy": {
        "nice_name": "Jeremy",
        "architecture": "x86",
        "hostname": "jeremy",
        "ipv4_address": "192.168.1.1",
        "amt": {
            "username": "sparkfun",
            "password": "Sparkfun1!"
        },
        "ssh": {
            "username": "ubuntu",
            "password": "ubuntu"
        }
    },
    "richard": {
        "nice_name": "Richard",
        "architecture": "armhf",
        "hostname": "richard",
        "ipv4_address": "192.168.1.2",
        "amt": None,
        "ssh": {
            "username": "ubuntu",
            "password": "ubuntu"
        }
    }
}

client = client.Client(HOSTS["jeremy"]["hostname"],
                       HOSTS["jeremy"]["amt"]["password"],
                       HOSTS["jeremy"]["amt"]["username"])

pwr_status = int(client.power_status())
print wsman.friendly_power_state(pwr_status)

if pwr_status == wsman.POWER_STATES["off"]:
    print "Turning On!"
    result = client.power_on()

