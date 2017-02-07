#include <string>
#include <sstream>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include <fcntl.h>

#include "ros/ros.h"
#include "sensor_msgs/Joy.h"

// Driver header file
#include <pru_utils/prussdrv.h>
#include <pru_utils/pruss_intc_mapping.h>

#include <generated_headers/pru_main_text.h>
#include <generated_headers/pru_main_data.h>

#include "motor_control/motor_control_node.h"
#include "motor_control/motor_control.h"


#define PRU_NUM   0

motor_control* ctrl;

static void rxJoystickCallback(const sensor_msgs::Joy::ConstPtr& data) {
  static int8_t last_motor_direction = -1;

  // Axes for Xbox controller:
  //  0 : Left Stick Left/Right,    Right is positive
  //  1 : Left Stick Up/Down,       Down is positive
  //  2 : Left Trigger,             Out is -2^15, In is 2^15
  //  3 : Right Trigger,            Out is -2^15, In is 2^15
  //  4 : Right Stick Left/Right,   Right is positive
  //  5 : Right Stick Up/Down,      Down is positive

  // Buttons for Xbox controller:
  //  0 : A
  //  1 : B
  //  2 : X
  //  3 : Y
  //  4 : Left Bumper
  //  5 : Right Bumper

  // Use the left stick as our input and flip the polarity.
  int32_t rx_value = data->axes[1] * -1;


  // Probably not needed, but clamp the rx'ed values to our rails.
  if(rx_value > kJoystickAxisMax) {
    rx_value = kJoystickAxisMax;
  } else if(rx_value < kJoystickAxisMin) {
    rx_value = kJoystickAxisMin;
  }

  // Introduce some deadband since the joysticks never return to zero.
  if(abs(rx_value) < kJoystickDeadband) {
    rx_value = 0;
  }

  // Now scale it to +/-100% range.
  int16_t motor_speed = (int16_t)(((double)rx_value / (double)kJoystickAxisMax) * 100.0f);
  ctrl->set_motor_speed(MOTOR_RIGHT, motor_speed);
  ctrl->set_motor_speed(MOTOR_LEFT, motor_speed);
}


int main(int argc, char **argv) {
  uint32_t count = 0;
  uint32_t count_to_write;
  uint16_t dac_cmd;

  ros::init(argc, argv, "motor_controller_node");
  ros::NodeHandle node("~");

  ROS_INFO("Initializing PRU.");
  tpruss_intc_initdata pruss_intc_initdata = PRUSS_INTC_INITDATA;
  prussdrv_init();

  /* Open PRU Interrupt */
  if (prussdrv_open(PRU_EVTOUT_0)) {
      ROS_FATAL("Failed to open PRU.");
      return -1;
  }

  /* Get the interrupt initialized */
  prussdrv_pruintc_init(&pruss_intc_initdata);


  ROS_INFO("Setting up GPIO");
  ctrl = new motor_control();

  tMotorControlReturnCode success = ctrl->init();
  switch(success) {
    case(MOTOR_SUCCESS):
      ROS_INFO("Successfully set up motor control.");
      break;

    case(MOTOR_ERROR_SPI_DEV):
      ROS_FATAL("Failed setting up motor control - SPI Device.");
      return -1;
      break;

    case(MOTOR_ERROR_GPIO):
      ROS_FATAL("Failed setting up motor control - GPIO.");
      return -1;
      break;

    default:
      ROS_FATAL("Failed setting up motor control - Unknown Error.");
      return -1;
      break;
  }

  ROS_INFO("Subscribing to the joystick topic.");
  ros::Subscriber joyTopic = node.subscribe<sensor_msgs::Joy>("/joy0", 10, rxJoystickCallback);

  ROS_INFO("Running PRU.");
  prussdrv_exec_code(PRU_NUM, (unsigned int *)&pru_main_text_bin[0], pru_main_text_bin_len);

  ROS_INFO("Spinning.");
  ros::spin();

  ROS_INFO("Cleaning up!");
  delete ctrl;

  ROS_INFO("Waiting for PRU HALT command.");
  prussdrv_pru_wait_event(PRU_EVTOUT_0);
  ROS_INFO("Tearing down PRU.\r\n");
  prussdrv_pru_clear_event (PRU_EVTOUT_0, PRU0_ARM_INTERRUPT);
  prussdrv_pru_disable (PRU_NUM);
  prussdrv_exit ();

  return 0;
}
