/***************************************************************************************
 * MAIN.C
 *
 * Description: main source file for PRU development
 *
 * Rafael de Souza
 * (C) 2015 Texas Instruments, Inc
 * 
 * Built with Code Composer Studio v6
 **************************************************************************************/

#include <stdint.h>
#include <motor_control/resource_table_empty.h>

volatile register unsigned int __R31, __R30;

int main(void) {
    unsigned int loops;
    for (loops = 0; loops < 10; loops++) {
        __R30 = __R30 | (1 << 15); // Turn on the LED
        __R30 = __R30 & ~(1 << 15); // Turn off the LED
    }

    // Send interrupt to host and halt
    __R31 = 32 | 3;
    __halt();
    return 0;
}
