#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <stdint.h>

const char* kDefaultRightDacSpiDev = "/dev/spidev1.0";
const char* kDefaultLeftDacSpiDev = "/dev/spidev2.0";

const int kDefaultRightMotorFwdGPIO = 60;   // Pin P9-12
const int kDefaultRightMotorRevGPIO = 50;   // Pin P9-14
const int kDefaultLeftMotorFwdGPIO = 48;    // Pin P9-15
const int kDefaultLeftMotorRevGPIO = 51;    // Pin P9-16

enum tMotorControlReturnCode {
    MOTOR_SUCCESS,
    MOTOR_ERROR_SPI_DEV,
    MOTOR_ERROR_GPIO
};

// Based on the low gearbox setting, every 1% in motor speed is approx 5rpm.
const int kMotorSpeedMaxPercent = 100;
const int kMotorSpeedMinPercent = -100;

// The DAC is 10-bit, so... 2^10 - 1
const int kMaxDacRange = 1023;

enum tMotorSelection {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT = 1,
    MOTOR_LAST = 2
};

class motor_control {
    
public:
    motor_control();
    motor_control(const char* spi_dev_left_dac, const char* spi_dev_right_dac,
    const int gpio_motor_right_fwd, const int gpio_motor_right_rev, const int gpio_motor_left_fwd,
    const int gpio_motor_left_rev);
    ~motor_control();

    tMotorControlReturnCode init();

    // Set the speed and direction for a motor.
    // motor        tMotorSelection     Enum to specify which motor to control.
    // motor_speed  int16_t             Speed from -100 to +100 percent.
    tMotorControlReturnCode set_motor_speed(tMotorSelection motor, int16_t motor_speed);

private:
    char* spi_dev_right_dac_;
    char* spi_dev_left_dac_;

    int gpio_motor_right_fwd_;
    int gpio_motor_right_rev_;
    int gpio_motor_left_fwd_;
    int gpio_motor_left_rev_;

    int fd_right_spi_dev_;
    int fd_left_spi_dev_;

    bool write_to_dac(int fd_spi_dac, bool buffered, bool gain, bool shutdown, uint16_t count);
};


#endif  // MOTOR_CONTROL_H
