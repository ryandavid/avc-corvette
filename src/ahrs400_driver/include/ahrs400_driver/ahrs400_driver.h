#ifndef XBOW_AHRS400
#define XBOW_AHRS400

#include <string>
#include <math.h>

#include "serial/serial.h"

#define VERSION_STR_MAX_LEN                 40

#define AHRS400_MAX_RESPONSE_SIZE           4095
#define AHRS400_VG_MODE_RESP_LEN            28
#define AHRS400_SCALED_MODE_RESP_LEN        22
#define AHRS400_VOLTAGE_MODE_RESP_LEN       22
#define AHRS400_SERIAL_NUMBER_LEN           4
#define AHRS400_DEFAULT_BAUD                38400

#define xbow_ahrs400_response_sleep()       usleep(10 * 1000)   // Amount of time to wait for a response

#define ANGLE_MEASUREMENT_FACTOR            (float)(M_PI / pow(2.0f, 15.0f))

#define G_RANGE_ACCEL                       (2.0f * 9.81f)
#define ACCEL_MEASUREMENT_FACTOR            (float)((G_RANGE_ACCEL * 1.5f) / pow(2.0f, 15.0f))

#define ANGULAR_RATE_RANGE                  (100.0f / (180.0f / M_PI))
#define ANGULAR_MEASUREMENT_FACTOR          (float)((ANGULAR_RATE_RANGE * 1.5f) / pow(2.0f, 15.0f))

#define MAGNETOMETER_RANGE                  (1.25f * 10000) // Scale is 1.25 gauss, convert to Tesla.
#define MAGNETOMETER_MEASUREMENT_FACTOR     (float)((ANGULAR_RATE_RANGE * 1.5f) / pow(2.0f, 15.0f))

enum AHRS400_COMMANDS
{
    RESET               = 'R',
    SET_VOLTAGE_MODE    = 'r',
    SET_SCALED_MODE     = 'c',
    SET_ANGLE_MODE      = 'a',
    SET_POLLED_MODE     = 'P',
    SET_CONTINUOUS_MODE = 'C',
    REQUEST_DATA        = 'G',
    QUERY_VERSION       = 'v',
    QUERY_SERIAL_NUMBER = 'S',
    REQUEST_AUTO_BAUD   = 'b',
    AUTOBAUD_MAGIC      = 'a',
};

enum AHRS400_RESPONSES
{
    RESP_RESET                  = 'H',
    RESP_SET_VOLTAGE_MODE       = 'R',
    RESP_SET_SCALED_MODE        = 'C',
    RESP_SET_ANGLE_MODE         = 'A',
    //RESP_SET_POLLED_MODE      = '',   // No response
    //RESP_SET_CONTINUOUS_MODE  = '',   // No response
    //RESP_REQUEST_DATA         = '',   // Data packet
    //RESP_QUERY_VERSION        = '',   // Data packet
    //RESP_QUERY_SERIAL_NUMBER  = '',   // Data packet
    RESP_REQUEST_AUTO_BAUD      = 'B',
};

enum AHRS400_MEASURE_MODES
{
    VOLTAGE_MODE,
    SCALED_MODE,
    ANGLE_MODE
};

enum AHRS400_REPORT_MODE
{
    CONTINUOUS_MODE,
    POLLED_MODE
};

struct raw_vg_mode_response_t
{
    int16_t rollAngle;
    int16_t pitchAngle;
    int16_t headingAngle;
    int16_t rollRate;
    int16_t pitchRate;
    int16_t yawRate;
    int16_t xAccel;
    int16_t yAccel;
    int16_t zAccel;
    int16_t xMag;
    int16_t yMag;
    int16_t zMag;
    int16_t tempSensor;
    int16_t time;
}; 

struct raw_scaled_mode_response_t
{
    int16_t rollRate;
    int16_t pitchRate;
    int16_t yawRate;
    int16_t xAccel;
    int16_t yAccel;
    int16_t zAccel;
    int16_t xMag;
    int16_t yMag;
    int16_t zMag;
    int16_t tempSensor;
    int16_t time;
};

// These are voltages!!
struct raw_voltage_mode_response_t
{
    int16_t rollRate;
    int16_t pitchRate;
    int16_t yawRate;
    int16_t xAccel;
    int16_t yAccel;
    int16_t zAccel;
    int16_t xMag;
    int16_t yMag;
    int16_t zMag;
    int16_t tempSensor;
    int16_t time;
};

// These are in engineering units!
struct scaled_measurements_t
{
    float attitude[3];      // Roll, Pitch, and Heading Angles
    float angularRate[3];   // Roll, Pitch, and Yaw Angular Rates
    float accel[3];         // X, Y, and Z accel
    float magnetometer[3];  // X, Y, and Z magnetic field
    float temperature;      // Temperature of sensor

    unsigned int timestamp;
};

struct ahrs_error_t
{
    unsigned int rxCommErrors;
    unsigned int txCommErrors;

    unsigned int crcErrors;
};

class xbow_ahrs400
{
    private:
        raw_vg_mode_response_t      raw_vg_mode_response;
        raw_scaled_mode_response_t  raw_scaled_mode_response;
        raw_voltage_mode_response_t raw_voltage_mode_response;

        AHRS400_MEASURE_MODES _measureMode;
        AHRS400_REPORT_MODE _reportMode;

        unsigned char _version[VERSION_STR_MAX_LEN];
        unsigned int _serialNumber;
        serial::Serial _port;

        ahrs_error_t _errors;

        bool txCommand(AHRS400_COMMANDS);
        unsigned int rxPacket(unsigned char*, unsigned int);
        void parseVgPacket(unsigned char*);

    public:
        scaled_measurements_t measurements;

        xbow_ahrs400(const std::string&, uint32_t = AHRS400_DEFAULT_BAUD);
        ~xbow_ahrs400();

        bool connect();
        unsigned char* getVersion(bool = false);
        unsigned int getSerialNumber(bool = false);
        bool getMeasurement();
        bool setMode(AHRS400_MEASURE_MODES, AHRS400_REPORT_MODE);
        bool reset();
};

#endif
