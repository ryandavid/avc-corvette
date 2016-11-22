#include <string>
#include <sstream>
#include <iostream>
#include <cstdio>
#include <unistd.h>

#include "ros/ros.h"

#include "bd950_driver/bd950_driver.h"

bd950::bd950(const std::string& port, uint32_t baudrate) : _port(port, baudrate, serial::Timeout::simpleTimeout(10)) {
    parserStateMachine = 0;
    numReceivedPackets = 0;
    numReceivedRecords = 0;

    rxMsgs.clear();

    //queryReceiverInfo();
}

bd950::~bd950(){
}

bool bd950::isConnected(){
    return _port.isOpen();
}

void bd950::connect(){
    //_port.connect();
}

void bd950::writeCorrectionData(std::vector<uint8_t> data){
    _port.write(data);
}

void bd950::queryReceiverInfo() {
    std::vector<uint8_t> request;

    request.push_back(0x02);
    request.push_back(0x00);
    request.push_back(0x06);
    request.push_back(0x00);
    request.push_back(0x06);
    request.push_back(0x03);

    _port.write(request);
}

void bd950::disconnect(){
    _port.close();
}

int bd950::getRxPackets(){
    return numReceivedPackets;
}

int bd950::getRxRecords(){
    return numReceivedRecords;
}

bool bd950::process(){
    bool newDataAvailable = false;
    uint8_t rxData;

    _port.read(rawDataBuffer, 100);

    // Lets loop through the bytes until we run the buffer dry.
    while(rawDataBuffer.size() > 0){

        // Basically 'Popping' the data
        rxData = rawDataBuffer.front();
        rawDataBuffer.erase(rawDataBuffer.begin());

        switch(parserStateMachine){
            // Initialization state... waiting for message
            case(0):
                if(rxData == 0x03){
                    parserStateMachine = 1;
                }else{
                    parserStateMachine = 0;
                }
                break;

            case(1):
                if(rxData == 0x02){
                    parserStateMachine = 2;
                }else{
                    parserStateMachine = 0;
                }
                break;

            // Stash away the potential 'STATUS' byte
            case(2):
                rxRcvrStatus = rxData;
                calculatedChecksum = rxData;
                parserStateMachine = 3;
                break;

            // Check for packet type (fixed 0x40 = GENOUT)
            case(3):
                rxPacketType = rxData;
                calculatedChecksum += rxData;
                parserStateMachine = 4;
                break;

            case(4):
                rxPacketLength = rxData;
                calculatedChecksum += rxData;
                rxRecordData.clear();
                parserStateMachine = 5;
                break;

            case(5):
                if(rxRecordData.size() < rxPacketLength){
                    rxRecordData.push_back(rxData);
                    calculatedChecksum += rxData;
                }else{
                    // This byte is the checksum
                    rxChecksum = rxData;

                    if(calculatedChecksum != rxChecksum){
                        ROS_ERROR("Invalid checksum calculated! Rx = %d, Calc = %d", rxChecksum, calculatedChecksum);
                        
                    } else {
                        numReceivedPackets++;

                        ROS_DEBUG("Processing packet (Type: 0x%02x, length: %d).", rxPacketType, rxPacketLength);
                        switch(rxPacketType) {
                            case(0x40):
                            processPacket40();
                            newDataAvailable = true;
                            break;

                            default:
                            ROS_WARN("Unknown packet type 0x%02X!", rxPacketType);
                        }
                    }

                    parserStateMachine = 0;
                }
                break;

            default:
                // Should never reach this
                parserStateMachine = 0;
                break;
        }
    }

    return newDataAvailable;
}


float bd950::convertToFloat(std::vector<uint8_t> data, uint8_t starting_index) {
    UN_FLOAT tempFloat;

    tempFloat.bDat[0] = data.at(starting_index + 3);
    tempFloat.bDat[1] = data.at(starting_index + 2);
    tempFloat.bDat[2] = data.at(starting_index + 1);
    tempFloat.bDat[3] = data.at(starting_index + 0);

    return tempFloat.fData;
}

double bd950::convertToDouble(std::vector<uint8_t> data, uint8_t starting_index) {
    UN_DOUBLE tempDouble;

    tempDouble.bDat[0] = data.at(starting_index + 7);
    tempDouble.bDat[1] = data.at(starting_index + 6);
    tempDouble.bDat[2] = data.at(starting_index + 5);
    tempDouble.bDat[3] = data.at(starting_index + 4);
    tempDouble.bDat[4] = data.at(starting_index + 3);
    tempDouble.bDat[5] = data.at(starting_index + 2);
    tempDouble.bDat[6] = data.at(starting_index + 1);
    tempDouble.bDat[7] = data.at(starting_index + 0);

    return tempDouble.dData;
}

bool bd950::getBool(std::vector<uint8_t> data, uint8_t starting_index, uint8_t bit_index) {
    uint8_t bitmask = 1 << bit_index;

    return (data.at(starting_index) & bitmask) == bitmask;
}

void bd950::processPacket40(){
    int index = 0;
    gsof recordType;
    int recordLength;
    UN_DOUBLE tempDouble;
    UN_FLOAT tempFloat;
    uint8_t numSv;

    uint8_t transmissionNumber = rxRecordData.at(0);
    uint8_t pageIndex = rxRecordData.at(1);
    uint8_t maxPageIndex = rxRecordData.at(2);
    rxRecordData.erase(rxRecordData.begin(), rxRecordData.begin() + 3);
    //ROS_INFO("Transmission Number: %d", transmissionNumber);
    //ROS_INFO("Page Index: %d / %d", pageIndex, maxPageIndex);

    while(index < rxRecordData.size()){
        numReceivedRecords += 1;

        recordType = (gsof)rxRecordData.at(index);
        recordLength = rxRecordData.at(index+1);

        if(recordLength > rxRecordData.size() - index) {
            ROS_ERROR("Invalid message length!");
            break;
        }

        switch(recordType){
        case(PositionTime):
            ROS_DEBUG("PositionTime");
            rxMsgs[PositionTime] = true;
            break;

        case(LLH):
            ROS_DEBUG("LLH");
            llh.latitude = convertToDouble(rxRecordData, index + 2) * kRadianToDegrees;
            llh.longitude = convertToDouble(rxRecordData, index + 10) * kRadianToDegrees;
            llh.height = convertToDouble(rxRecordData, index + 18);
            rxMsgs[LLH] = true;
            break;

        case(SVBrief):
            ROS_DEBUG("SVBrief");
            numSv = rxRecordData.at(index + 2) & 0xFF;
            svInfo.resize(numSv);

            for(int i = 0; i < numSv; i++){
                svInfo[i].prn = rxRecordData.at( index + 3 + (i*3) ) & 0xFF;

                svInfo[i].satelliteAboveHorizon = getBool(rxRecordData, index + 4 + (i*3), 0);
                svInfo[i].assignedToChannel = getBool(rxRecordData, index + 4 + (i*3), 1);
                svInfo[i].trackedL1 = getBool(rxRecordData, index + 4 + (i*3), 2);
                svInfo[i].trackedL2 = getBool(rxRecordData, index + 4 + (i*3), 3);
                svInfo[i].trackedAtBaseL1 = getBool(rxRecordData, index + 4 + (i*3), 4);
                svInfo[i].trackedAtBaseL2 = getBool(rxRecordData, index + 4 + (i*3), 5);
                svInfo[i].usedInSolution = getBool(rxRecordData, index + 4 + (i*3), 6);
                svInfo[i].usedInRTK = getBool(rxRecordData, index + 4 + (i*3), 7);

                svInfo[i].trackedPcodeL1 = getBool(rxRecordData, index + 5 + (i*3), 0);
                svInfo[i].trackedPcodeL2 = getBool(rxRecordData, index + 5 + (i*3), 1);
                svInfo[i].elevation = 0;    // Not supported in this message
                svInfo[i].azimuth = 0;      // Not supported in this message
                svInfo[i].snrL1 = 0;        // Not supported in this message
                svInfo[i].snrL2 = 0;        // Not supported in this message
            }
            rxMsgs[SVBrief] = true;
            break;

        case(SVDetail):
            ROS_DEBUG("SVDetail");
            numSv = rxRecordData.at(index + 2) & 0xFF;
            svInfo.resize(numSv);

            for(int i = 0; i < numSv; i++){
                svInfo[i].prn = rxRecordData.at( index + 3 + (i*8) ) & 0xFF;

                svInfo[i].satelliteAboveHorizon = getBool(rxRecordData, index + 4 + (i*8), 0);
                svInfo[i].assignedToChannel = getBool(rxRecordData, index + 4 + (i*8), 1);
                svInfo[i].trackedL1 = getBool(rxRecordData, index + 4 + (i*8), 2);
                svInfo[i].trackedL2 = getBool(rxRecordData, index + 4 + (i*8), 3);
                svInfo[i].trackedAtBaseL1 = getBool(rxRecordData, index + 4 + (i*8), 4);
                svInfo[i].trackedAtBaseL2 = getBool(rxRecordData, index + 4 + (i*8), 5);
                svInfo[i].usedInSolution = getBool(rxRecordData, index + 4 + (i*8), 6);
                svInfo[i].usedInRTK = getBool(rxRecordData, index + 4 + (i*8), 7);

                svInfo[i].trackedPcodeL1 = getBool(rxRecordData, index + 5 + (i*8), 0);
                svInfo[i].trackedPcodeL2 = getBool(rxRecordData, index + 5 + (i*8), 1);

                svInfo[i].elevation = rxRecordData.at(index + 6 + (i*8));
                svInfo[i].azimuth = (rxRecordData.at(index + 7 + (i*8)) << 8) + rxRecordData.at(index + 8 + (i*8));
                svInfo[i].snrL1 = ((float)rxRecordData.at(index + 9 + (i*8))) / 4.0f;
                svInfo[i].snrL2 = ((float)rxRecordData.at(index + 10 + (i*8))) / 4.0f;
            }
            rxMsgs[SVDetail] = true;
            break;

        case(PositionTimeUTC):
            ROS_DEBUG("PositionTimeUTC");
            positionTimeUTC.millisecondsOfWeek =
                        (rxRecordData.at(index + 2) << 24) +
                        (rxRecordData.at(index + 3) << 16) +
                        (rxRecordData.at(index + 4) << 8) +
                        (rxRecordData.at(index + 5) << 0);

            positionTimeUTC.gpsWeekNumber =
                    ((int)rxRecordData.at(index + 6) << 8) +
                    ((int)rxRecordData.at(index + 7) << 0);

            positionTimeUTC.numOfSV = rxRecordData.at(index + 8);

            positionTimeUTC.newPosition = getBool(rxRecordData, index + 9, 0);
            positionTimeUTC.newClockThisSolution = getBool(rxRecordData, index + 9, 1);
            positionTimeUTC.newHorizontalThisSolution = getBool(rxRecordData, index + 9, 2);
            positionTimeUTC.newHeightThisSolution = getBool(rxRecordData, index + 9, 3);
            positionTimeUTC.usesLeastSquaresPosition = getBool(rxRecordData, index + 9, 5);
            positionTimeUTC.usesFilteredL1 = getBool(rxRecordData, index + 9, 7);

            positionTimeUTC.isDifferential = getBool(rxRecordData, index + 10, 0);
            positionTimeUTC.isPhase = getBool(rxRecordData, index + 10, 1);
            positionTimeUTC.isFixedInteger = getBool(rxRecordData, index + 10, 2);
            positionTimeUTC.isOmnistar = getBool(rxRecordData, index + 10, 3);
            positionTimeUTC.isStatic = getBool(rxRecordData, index + 10, 4);
            positionTimeUTC.isNetworkRTK = getBool(rxRecordData, index + 10, 5);
            positionTimeUTC.isLocationRTK = getBool(rxRecordData, index + 10, 6);
            positionTimeUTC.isBeaconDGPS = getBool(rxRecordData, index + 10, 7);

            positionTimeUTC.initCounter = rxRecordData.at(index + 11);
            /*
            ROS_INFO("B9: 0x%02X", rxRecordData.at(index + 9));
            ROS_INFO(" - New Position: %d", positionTimeUTC.newPosition);
            ROS_INFO(" - New Clock: %d", positionTimeUTC.newClockThisSolution);
            ROS_INFO(" - New Horizontal: %d", positionTimeUTC.newHorizontalThisSolution);
            ROS_INFO(" - New Height: %d", positionTimeUTC.newHeightThisSolution);
            ROS_INFO("B10: 0x%02X", rxRecordData.at(index + 10));
            ROS_INFO(" - isDifferential: %d", positionTimeUTC.isDifferential);
            ROS_INFO(" - isPhase: %d", positionTimeUTC.isPhase);
            ROS_INFO(" - isFixedInteger: %d", positionTimeUTC.isFixedInteger);
            ROS_INFO(" - isNetworkRTK: %d", positionTimeUTC.isNetworkRTK);
            ROS_INFO(" - isLocationRTK: %d", positionTimeUTC.isLocationRTK);
            ROS_INFO(" - isBeaconDGPS: %d", positionTimeUTC.isBeaconDGPS);*/

            rxMsgs[PositionTimeUTC] = true;
            break;

        case(PositionSigma):
            ROS_DEBUG("PositionSigma"); 
            sigma.rms = convertToFloat(rxRecordData, index + 2);
            sigma.sigmaEast = convertToFloat(rxRecordData, index + 6);
            sigma.sigmaNorth = convertToFloat(rxRecordData, index + 10);
            sigma.covarianceEastNorth = convertToFloat(rxRecordData, index + 14);
            sigma.sigmaUp = convertToFloat(rxRecordData, index + 18);
            sigma.majorAxis = convertToFloat(rxRecordData, index + 22);
            sigma.minorAxis = convertToFloat(rxRecordData, index + 26);
            sigma.orientation = convertToFloat(rxRecordData, index + 30);
            sigma.unitVariance = convertToFloat(rxRecordData, index + 34);

            sigma.numEpochs =   ((int)(rxRecordData.at(index + 38) & 0xFF) << 8) +
                                ((int)(rxRecordData.at(index + 39) & 0xFF) << 0);

            rxMsgs[PositionSigma] = true;
            break;

        case(Velocity):
            ROS_DEBUG("Velocity");
            velocity.velocityValid = getBool(rxRecordData, index + 2, 0);
            velocity.computeMethod = getBool(rxRecordData, index + 2, 1);

            velocity.horizontalVelocity = convertToFloat(rxRecordData, index + 3);
            velocity.heading = convertToFloat(rxRecordData, index + 7) * kRadianToDegrees;
            velocity.verticalVelocity = convertToFloat(rxRecordData, index + 11);

            rxMsgs[Velocity] = true;
            break;

        case(PositionVCV):
            ROS_DEBUG("PositionVCV");
            varianceCovariance.positionRMS = convertToFloat(rxRecordData, index + 2);
            varianceCovariance.xx = convertToFloat(rxRecordData, index + 6);
            varianceCovariance.xy = convertToFloat(rxRecordData, index + 10);
            varianceCovariance.xz = convertToFloat(rxRecordData, index + 14);
            varianceCovariance.yy = convertToFloat(rxRecordData, index + 18);
            varianceCovariance.yz = convertToFloat(rxRecordData, index + 22);
            varianceCovariance.zz = convertToFloat(rxRecordData, index + 26);
            varianceCovariance.unitVariance = convertToFloat(rxRecordData, index + 30);

            varianceCovariance.numEpochs = ((int)rxRecordData.at(index + 35) << 8) +
                                           ((int)rxRecordData.at(index + 34) << 0);

            rxMsgs[PositionVCV] = true;
            break;

        default:
        ROS_WARN("Unknown msg type 0x%02X.", recordType);
        }

        // Next record starts at the previous plus the type and header bytes
        index += recordLength + 2;
    }
}
