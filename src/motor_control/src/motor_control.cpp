#include <string>
#include <sstream>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>

#include "motor_control/gpio.h"
#include "motor_control/motor_control.h"

motor_control::motor_control() {
    spi_dev_right_dac_ = (char*)kDefaultRightDacSpiDev;
    spi_dev_left_dac_ = (char*)kDefaultLeftDacSpiDev;

    gpio_motor_right_fwd_ = kDefaultRightMotorFwdGPIO;
    gpio_motor_right_rev_ = kDefaultRightMotorRevGPIO;
    gpio_motor_left_fwd_ = kDefaultLeftMotorFwdGPIO;
    gpio_motor_left_rev_ = kDefaultLeftMotorRevGPIO;
}


motor_control::motor_control(const char* spi_dev_left_dac, const char* spi_dev_right_dac,
    const int gpio_motor_right_fwd, const int gpio_motor_right_rev, const int gpio_motor_left_fwd,
    const int gpio_motor_left_rev) {

    spi_dev_right_dac_ = (char*)spi_dev_right_dac;
    spi_dev_left_dac_ = (char*)spi_dev_left_dac;

    gpio_motor_right_fwd_ = gpio_motor_right_fwd;
    gpio_motor_right_rev_ = gpio_motor_right_rev;
    gpio_motor_left_fwd_ = gpio_motor_left_fwd;
    gpio_motor_left_rev_ = gpio_motor_left_rev;
}

motor_control::~motor_control() {
    // Make sure both motors are turned off.
    set_motor_speed(MOTOR_RIGHT, 0);
    set_motor_speed(MOTOR_LEFT, 0);
}

tMotorControlReturnCode motor_control::init() {
    // Open up handles to the SPI DACs.
    fd_right_spi_dev_ = open(spi_dev_right_dac_, O_RDWR);
    if (fd_right_spi_dev_ <= 0) {
        return MOTOR_ERROR_SPI_DEV;
    }

    // Ensure that the GPIOs are exported into userland.
    if(gpio_export(gpio_motor_right_fwd_) || gpio_export(gpio_motor_right_rev_) ||
       gpio_export(gpio_motor_left_fwd_) || gpio_export(gpio_motor_left_rev_)) {
        return MOTOR_ERROR_GPIO;
    }

    // Set the pins to all outputs.
    if(gpio_set_dir(gpio_motor_right_fwd_, GPIO_DIR_OUTPUT) || gpio_set_dir(gpio_motor_right_rev_, GPIO_DIR_OUTPUT) ||
       gpio_set_dir(gpio_motor_left_fwd_, GPIO_DIR_OUTPUT) || gpio_set_dir(gpio_motor_left_rev_, GPIO_DIR_OUTPUT)) {
        return MOTOR_ERROR_GPIO;
    }

    // And put them all low, turning off the FETs.
    if(gpio_set_value(gpio_motor_right_fwd_, false) || gpio_set_value(gpio_motor_right_rev_, false) ||
       gpio_set_value(gpio_motor_left_fwd_, false) || gpio_set_value(gpio_motor_left_rev_, false)) {
        return MOTOR_ERROR_GPIO;
    }

    return MOTOR_SUCCESS;
}

tMotorControlReturnCode motor_control::set_motor_speed(tMotorSelection motor, int16_t motor_speed) {
    int fd_spi_dev, gpio_fwd, gpio_rev;

    // Santize the motor speed provided to between +/-100%.
    if(motor_speed > kMotorSpeedMaxPercent) {
        motor_speed = kMotorSpeedMaxPercent;
    } else if(motor_speed < kMotorSpeedMinPercent) {
        motor_speed = kMotorSpeedMinPercent;
    }
    
    if(motor == MOTOR_LEFT) {
        fd_spi_dev = fd_left_spi_dev_;
        gpio_fwd = gpio_motor_left_fwd_;
        gpio_rev = gpio_motor_left_rev_;
    } else {
        fd_spi_dev = fd_right_spi_dev_;
        gpio_fwd = gpio_motor_right_fwd_;
        gpio_rev = gpio_motor_right_rev_;
    }

    // TODO: Need to set these only when a direction change occurs.
    // Forward.
    if(motor_speed > 0) {
        gpio_set_value(gpio_fwd, true);
        gpio_set_value(gpio_rev, false);

    // Setting a speed of zero will clear both direction pins, inhibiting the motor.
    } else if (motor_speed == 0) {
        gpio_set_value(gpio_fwd, false);
        gpio_set_value(gpio_rev, false);

    // Reverse.
    } else {
        gpio_set_value(gpio_fwd, false);
        gpio_set_value(gpio_rev, true);
    }

    // We need to convert the desired motor speed to DAC counts.
    // Since the direction control is handled by GPIO, use the absolute value.
    uint16_t counts = (abs(motor_speed) / kMotorSpeedMaxPercent) * kMaxDacRange;

    if(write_to_dac(fd_spi_dev, true, true, true, counts) == true) {
        return MOTOR_SUCCESS;
    } else {
        return MOTOR_ERROR_SPI_DEV;
    }
}


bool motor_control::write_to_dac(int fd_spi_dac, bool buffered, bool gain, bool shutdown, uint16_t count) {
    // Assembling it bass-ackwards due to clocking out LSB first.
    // TODO: Actually use the other flags.

    uint16_t cmd = ((0x70 + ((count >> 6) & 0x0F)) << 0) + ((count & 0x3F) << 10);

    if (write(fd_spi_dac, &cmd, sizeof(cmd)) != sizeof(cmd)) {
        return false;
    }

    return true;
}
