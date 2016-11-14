#ifndef bd950_H
#define bd950_H

#include <string>
#include <math.h>

#include "serial/serial.h"

class GSOF_2 {
public:
    double latitude;    // Latitude in WGS-84 datum
    double longitude;   // Longitude in WGS084 datum
    double height;      // Height in WGS-84 datum

    GSOF_2(){
        latitude = 0;
        longitude = 0;
        height = 0;
    }
};

class GSOF_8 {
public:
    bool velocityValid;
    bool computeMethod;

    float horizontalVelocity;   // Horizontal velocity in m/s
    float heading;              // WGS-84 referenced true north heading
    float verticalVelocity;     // Vertical velocity in m/s

    GSOF_8(){
        velocityValid = false;
        computeMethod = false;

        horizontalVelocity = 0;
        heading = 0;
        verticalVelocity = 0;
    }
};

class GSOF_11 {
public:
    float positionRMS;

    float xx;
    float xy;
    float xz;

    float yy;
    float yz;

    float zz;

    float unitVariance;

    int numEpochs;

    GSOF_11(){
        positionRMS = 0;

        xx = 0.0f;
        xy = 0.0f;
        xz = 0.0f;

        yy = 0.0f;
        yz = 0.0f;

        zz = 0.0f;

        unitVariance = 0.0f;

        numEpochs = 0;
    }
};

class GSOF_12 {
public:
    float rms;
    float sigmaEast;
    float sigmaNorth;
    float covarianceEastNorth;
    float sigmaUp;
    float majorAxis;
    float minorAxis;
    float orientation;
    float unitVariance;
    int numEpochs;

    GSOF_12(){
        rms = 0.0f;
        sigmaEast = 0.0f;
        sigmaNorth = 0.0f;
        covarianceEastNorth = 0.0f;
        sigmaUp = 0.0f;
        majorAxis = 0.0f;
        minorAxis = 0.0f;
        orientation = 0.0f;
        unitVariance = 0.0f;
        numEpochs = 0;
    }
};

class GSOF_14 {
public:
    int numSv;
    int prn;

    bool satelliteAboveHorizon;
    bool assignedToChannel;
    bool trackedL1;
    bool trackedL2;
    bool trackedAtBaseL1;
    bool trackedAtBaseL2;
    bool usedInSolution;
    bool usedInRTK;

    bool trackedPcodeL1;
    bool trackedPcodeL2;

    int elevation;
    int azimuth;
    float snrL1;
    float snrL2;

    GSOF_14(){
        numSv = 0;
        prn = 0;
        satelliteAboveHorizon = false;
        assignedToChannel = false;
        trackedL1 = false;
        trackedL2 = false;
        trackedAtBaseL1 = false;
        trackedAtBaseL2 = false;
        usedInSolution = false;
        usedInRTK = false;
        trackedPcodeL1 = false;
        trackedPcodeL2 = false;
        elevation = 0;
        azimuth = 0;
        snrL1 = 0;
        snrL2 = 0;
    }

};

class GSOF_26 {
public:
    int millisecondsOfWeek;
    int gpsWeekNumber;
    int numOfSV;

    bool newPosition;
    bool newClockThisSolution;
    bool newHorizontalThisSolution;
    bool newHeightThisSolution;
    bool usesLeastSquaresPosition;
    bool usesFilteredL1;

    bool isDifferential;
    bool isPhase;
    bool isFixedInteger;
    bool isOmnistar;
    bool isStatic;
    bool isNetworkRTK;
    bool isLocationRTK;
    bool isBeaconDGPS;

    int initCounter;

    GSOF_26(){
        millisecondsOfWeek = 0;
        gpsWeekNumber = 0;
        numOfSV = 0;

        newPosition = 0;
        newClockThisSolution = 0;
        newHorizontalThisSolution = 0;
        newHeightThisSolution = 0;
        usesLeastSquaresPosition = 0;
        usesFilteredL1 = 0;

        isDifferential = 0;
        isPhase = 0;
        isFixedInteger = 0;
        isOmnistar = 0;
        isStatic = 0;
        isNetworkRTK = 0;
        isLocationRTK = 0;
        isBeaconDGPS = 0;
        initCounter = 0;
    }
};


class bd950 {

public:
    bd950(const std::string& port, uint32_t baudrate);
    ~bd950();

    bool isConnected();
    void connect();
    void disconnect();
    bool showSettingsDialog();
    int getRxPackets();
    int getRxRecords();
    bool rawDataAvailable();
    void writeCorrectionData(std::vector<uint8_t> data);
    void queryReceiverInfo();

    GSOF_2 llh;
    GSOF_12 sigma;
    GSOF_14 svInfo[24];      // SV Information
    GSOF_26 positionTimeUTC;
    GSOF_8 velocity;
    GSOF_11 varianceCovariance;

    typedef union _un_double_
    {
            unsigned char bDat[8];
            double dData;
    }UN_DOUBLE;

    typedef union _un_float_
    {
            unsigned char bDat[4];
            float fData;
    }UN_FLOAT;


private:
    serial::Serial _port;;
    std::vector<uint8_t> rawDataBuffer;

    int parserStateMachine;
    int numReceivedRecords;
    int numReceivedPackets;

    uint8_t rxRcvrStatus;
    uint8_t rxPacketLength;
    uint8_t rxPacketType;
    uint8_t rxTransNumber;
    uint8_t rxPageIndex;
    uint8_t rxMaxPageIndex;
    uint8_t rxRecordType;
    uint8_t rxRecordLength;
    std::vector<uint8_t> rxRecordData;
    uint8_t rxChecksum;
    uint8_t calculatedChecksum;


    enum gsof {
        PositionTime        = 1,
        LLH                 = 2,
        ECEF                = 3,
        LocalDatum          = 4,
        LocalZone           = 5,
        ECEFDetla           = 6,
        TangentPlaneDelta   = 7,
        Velocity            = 8,
        PDOP                = 9,
        Clock               = 10,
        PositionVCV         = 11,
        PositionSigma       = 12,
        SVBrief             = 13,
        SVDetail            = 14,
        ReceiverSerial      = 15,
        CurrentTime         = 16,
        PositionTimeUTC     = 26,
        Attitude            = 27,
        AllSVBrief          = 33,
        AllDVDetailed       = 34,
        ReceivedBaseInfo    = 35,
        BasePositionQuality = 41
    };

    void processPacket40();


};



#endif // bd950_H
