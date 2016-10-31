#include <string>
#include <sstream>
#include <iostream>
#include <cstdio>
#include <unistd.h>

#include "arpa/inet.h"
#include "ros/ros.h"

#include "ahrs400_driver/ahrs400_driver.h"

xbow_ahrs400::xbow_ahrs400(const std::string& port, uint32_t baudrate) : _port(port, baudrate, serial::Timeout::simpleTimeout(100)) {
  memset(&(_errors),   0x00,   sizeof(_errors));
  memset(&(_version),  0x00,   sizeof(_version));
  _serialNumber = 0;

  _measureMode  = ANGLE_MODE;
  _reportMode   = POLLED_MODE;

  connect();
}

xbow_ahrs400::~xbow_ahrs400() {
  _port.close();
}

bool xbow_ahrs400::connect() {
  bool success;

  success = _port.isOpen();

  if(success == true) {
    reset();
    getVersion();
    getSerialNumber();
  }
    
  return success;
}

bool xbow_ahrs400::txCommand(AHRS400_COMMANDS command) {
  uint8_t result = _port.write(std::string(1, command));

  if(result == 0){
    ROS_ERROR("Failed to write command: %d", command);
  }

  return result;
}

unsigned int xbow_ahrs400::rxPacket(unsigned char* buffer, unsigned int maxRxSize) {
  unsigned char tempBuffer[AHRS400_MAX_RESPONSE_SIZE];
  unsigned int numBytes;
  unsigned char checksum = 0;

  // Add in 2B for header and checksum
  maxRxSize += 2;

  // Prevent Buffer overflow
  if(maxRxSize > sizeof(tempBuffer)) {
    maxRxSize = sizeof(tempBuffer);
  }

  // Poll the comm port
  numBytes = _port.read(&(tempBuffer[0]), maxRxSize);

  if(numBytes > 2) {
    // Perform CRC calculation; Sum of all bytes
    for(unsigned int i = 1; i < numBytes - 1; i++) {  // Skip header and checksum bytes
      checksum += tempBuffer[i];
    }

    // Compare calculated checksum against the received checksum
    if(checksum == tempBuffer[numBytes-1]) {
      numBytes = numBytes - 2;  // Remove header and checksum bytes
      memcpy(buffer, &(tempBuffer[1]), numBytes);
    } else {
      // Failed CRC
      numBytes = 0;
    }
  }

  return numBytes;  
}

unsigned char* xbow_ahrs400::getVersion(bool forceUpdate) {
  int numBytes = 0;

  if((_version[0] == 0x00) || (forceUpdate == true)) {
    txCommand(QUERY_VERSION);

    xbow_ahrs400_response_sleep();
    numBytes = rxPacket(&(_version[0]), sizeof(_version));
    _version[numBytes] = '\0';
  }

  return &(_version[0]);
}

unsigned int xbow_ahrs400::getSerialNumber(bool forceUpdate) {
  unsigned char tempBuff[AHRS400_SERIAL_NUMBER_LEN];
  int numBytes = 0;

  if((_serialNumber == 0x00) || (forceUpdate == true)) {
    txCommand(QUERY_SERIAL_NUMBER);

    xbow_ahrs400_response_sleep();
    numBytes = rxPacket(&(tempBuff[0]), sizeof(tempBuff));

    // Check to make sure we received the full serial number
    if(numBytes == sizeof(tempBuff)) {
      _serialNumber = ((tempBuff[0] << 24) + \
               (tempBuff[1] << 16) + \
               (tempBuff[2] <<  8) + \
               (tempBuff[3] <<  0));
    }
  }

  return _serialNumber;
}

bool xbow_ahrs400::setMode(AHRS400_MEASURE_MODES measure_mode, AHRS400_REPORT_MODE report_mode) {
  bool success = true;
  unsigned char response = 0xFF;

  switch(measure_mode) {
    case(VOLTAGE_MODE):
      _measureMode = VOLTAGE_MODE;
      txCommand(SET_VOLTAGE_MODE);
      xbow_ahrs400_response_sleep();
      rxPacket(&response, sizeof(response));
      success &= (response == RESP_SET_VOLTAGE_MODE);
      break;

    case(SCALED_MODE):
      _measureMode = SCALED_MODE;
      txCommand(SET_SCALED_MODE);
      xbow_ahrs400_response_sleep();
      rxPacket(&response, sizeof(response));
      success &= (response == RESP_SET_SCALED_MODE);
      break;

    case(ANGLE_MODE):
      _measureMode = ANGLE_MODE;
      txCommand(SET_ANGLE_MODE);
      xbow_ahrs400_response_sleep();
      rxPacket(&response, sizeof(response));
      success &= (response == RESP_SET_ANGLE_MODE);
      break;

    default:
      success &= false;
      break;
  }

  switch(report_mode) {
    case(CONTINUOUS_MODE):
      _reportMode = CONTINUOUS_MODE;
      txCommand(SET_CONTINUOUS_MODE);
      xbow_ahrs400_response_sleep();
      // Response is continuous stream of packets
      break;

    case(POLLED_MODE):
      _reportMode = POLLED_MODE;
      txCommand(SET_POLLED_MODE);
      xbow_ahrs400_response_sleep();
      // No response
      break;

    default:
      success &= false;
      break;
  }

  return success;
}

bool xbow_ahrs400::getMeasurement() {
  unsigned int numBytes;
  unsigned char buffer[AHRS400_VG_MODE_RESP_LEN];
  bool success = false;

  // Only request a measurement if we are in polled mode.
  if(_reportMode == POLLED_MODE) {
    txCommand(REQUEST_DATA);
    xbow_ahrs400_response_sleep();
  }
  
  numBytes = rxPacket(&(buffer[0]), sizeof(buffer));

  if(sizeof(raw_vg_mode_response) != numBytes) {
    // Error in Rx, Flush the serial port.
    ROS_ERROR("getPolledMeasurement received %d bytes, expecting %lu.", numBytes, sizeof(raw_vg_mode_response));
    _port.flushInput();

    success = false;
  } else {
    // Time to interpret the result
    switch(_measureMode) {
    case(ANGLE_MODE):
      parseVgPacket(&(buffer[0]));
      success = true;
      break;

    case(SCALED_MODE):
    case(VOLTAGE_MODE):
    default:
      //TODO : parse these out!
      success = false;
      break;
    }
  }

  return success;
}

bool xbow_ahrs400::reset() {
  bool success = true;
  unsigned char buffer[AHRS400_MAX_RESPONSE_SIZE];

  txCommand(SET_POLLED_MODE);
  xbow_ahrs400_response_sleep();
  txCommand(RESET);
  xbow_ahrs400_response_sleep();
  _port.flushInput();

  // Do a dummy read.
  //rxPacket(&(buffer[0]), sizeof(buffer));

  return success;
}

void xbow_ahrs400::parseVgPacket(unsigned char* buffer){
  memcpy(&raw_vg_mode_response, buffer, sizeof(raw_vg_mode_response));
  measurements.timestamp = raw_vg_mode_response.time;

  measurements.attitude[0] = (int16_t)ntohs(raw_vg_mode_response.rollAngle) * ANGLE_MEASUREMENT_FACTOR;
  measurements.attitude[1] = (int16_t)ntohs(raw_vg_mode_response.pitchAngle) * ANGLE_MEASUREMENT_FACTOR;
  measurements.attitude[2] = (int16_t)ntohs(raw_vg_mode_response.headingAngle) * ANGLE_MEASUREMENT_FACTOR;

  measurements.angularRate[0] = (int16_t)ntohs(raw_vg_mode_response.rollRate) * ANGULAR_MEASUREMENT_FACTOR;
  measurements.angularRate[1] = (int16_t)ntohs(raw_vg_mode_response.pitchRate) * ANGULAR_MEASUREMENT_FACTOR;
  measurements.angularRate[2] = (int16_t)ntohs(raw_vg_mode_response.yawRate) * ANGULAR_MEASUREMENT_FACTOR;

  measurements.accel[0] = (int16_t)ntohs(raw_vg_mode_response.xAccel) * ACCEL_MEASUREMENT_FACTOR;
  measurements.accel[1] = (int16_t)ntohs(raw_vg_mode_response.yAccel) * ACCEL_MEASUREMENT_FACTOR;
  measurements.accel[2] = (int16_t)ntohs(raw_vg_mode_response.zAccel) * ACCEL_MEASUREMENT_FACTOR;

  measurements.magnetometer[0] = (int16_t)ntohs(raw_vg_mode_response.xMag) * MAGNETOMETER_MEASUREMENT_FACTOR;
  measurements.magnetometer[1] = (int16_t)ntohs(raw_vg_mode_response.yMag) * MAGNETOMETER_MEASUREMENT_FACTOR;
  measurements.magnetometer[2] = (int16_t)ntohs(raw_vg_mode_response.zMag) * MAGNETOMETER_MEASUREMENT_FACTOR;

  measurements.temperature = raw_vg_mode_response.tempSensor;
}
