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

bool bd950::rawDataAvailable(){
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


void bd950::processPacket40(){
    int index = 0;
    gsof recordType;
    int recordLength;
    UN_DOUBLE tempDouble;
    UN_FLOAT tempFloat;

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

        switch(recordType){
        case(PositionTime):
            ROS_DEBUG("PositionTime");
            break;

        case(LLH):
            ROS_DEBUG("LLH");
            tempDouble.bDat[0] = rxRecordData.at(index + 9) & 0xFF;
            tempDouble.bDat[1] = rxRecordData.at(index + 8) & 0xFF;
            tempDouble.bDat[2] = rxRecordData.at(index + 7) & 0xFF;
            tempDouble.bDat[3] = rxRecordData.at(index + 6) & 0xFF;
            tempDouble.bDat[4] = rxRecordData.at(index + 5) & 0xFF;
            tempDouble.bDat[5] = rxRecordData.at(index + 4) & 0xFF;
            tempDouble.bDat[6] = rxRecordData.at(index + 3) & 0xFF;
            tempDouble.bDat[7] = rxRecordData.at(index + 2) & 0xFF;
            llh.latitude = tempDouble.dData * 57.2957795f;  // TODO: Change to constant

            tempDouble.bDat[0] = rxRecordData.at(index + 17) & 0xFF;
            tempDouble.bDat[1] = rxRecordData.at(index + 16) & 0xFF;
            tempDouble.bDat[2] = rxRecordData.at(index + 15) & 0xFF;
            tempDouble.bDat[3] = rxRecordData.at(index + 14) & 0xFF;
            tempDouble.bDat[4] = rxRecordData.at(index + 13) & 0xFF;
            tempDouble.bDat[5] = rxRecordData.at(index + 12) & 0xFF;
            tempDouble.bDat[6] = rxRecordData.at(index + 11) & 0xFF;
            tempDouble.bDat[7] = rxRecordData.at(index + 10) & 0xFF;
            llh.longitude = tempDouble.dData * 57.2957795f; // TODO: Change to constant

            tempDouble.bDat[0] = rxRecordData.at(index + 25) & 0xFF;
            tempDouble.bDat[1] = rxRecordData.at(index + 24) & 0xFF;
            tempDouble.bDat[2] = rxRecordData.at(index + 23) & 0xFF;
            tempDouble.bDat[3] = rxRecordData.at(index + 22) & 0xFF;
            tempDouble.bDat[4] = rxRecordData.at(index + 21) & 0xFF;
            tempDouble.bDat[5] = rxRecordData.at(index + 20) & 0xFF;
            tempDouble.bDat[6] = rxRecordData.at(index + 19) & 0xFF;
            tempDouble.bDat[7] = rxRecordData.at(index + 18) & 0xFF;
            llh.height = tempDouble.dData;
            break;

        case(SVBrief):
            ROS_DEBUG("SVBrief");
            svInfo[0].numSv = rxRecordData.at(index + 2) & 0xFF;

            for(int i = 0; i < svInfo[0].numSv; i++){
                svInfo[i].numSv = svInfo[0].numSv;
                svInfo[i].prn = rxRecordData.at( index + 3 + (i*3) ) & 0xFF;

                svInfo[i].satelliteAboveHorizon = (bool)( rxRecordData.at( index + 4 + (i*3)) & 0b00000001);
                svInfo[i].assignedToChannel = (bool)( rxRecordData.at( index + 4 + (i*3)) & 0b00000010);
                svInfo[i].trackedL1 = (bool)( rxRecordData.at( index + 4 + (i*3)) & 0b00000100);
                svInfo[i].trackedL2 = (bool)( rxRecordData.at( index + 4 + (i*3)) & 0b00001000);
                svInfo[i].trackedAtBaseL1 = (bool)( rxRecordData.at( index + 4 + (i*3)) & 0b00010000);
                svInfo[i].trackedAtBaseL2 = (bool)( rxRecordData.at( index + 4 + (i*3)) & 0b00100000);
                svInfo[i].usedInSolution = (bool)( rxRecordData.at( index + 4 + (i*3)) & 0b01000000);
                svInfo[i].usedInRTK = (bool)( rxRecordData.at( index + 4 + (i*3)) & 0b10000000);

                svInfo[i].trackedPcodeL1 = (bool)( rxRecordData.at( index + 5 + (i*3)) & 0b00000001);
                svInfo[i].trackedPcodeL2 = (bool)( rxRecordData.at( index + 5 + (i*3)) & 0b00000010);
                svInfo[i].elevation = 0;    // Not supported in this message
                svInfo[i].azimuth = 0;      // Not supported in this message
                svInfo[i].snrL1 = 0;        // Not supported in this message
                svInfo[i].snrL2 = 0;        // Not supported in this message
            }

            // Lets zero out the remaining SV info
            for(int i = svInfo[0].numSv; i < 24; i++){
                svInfo[i].numSv = 0;
                svInfo[i].prn = 0;

                svInfo[i].satelliteAboveHorizon = 0;
                svInfo[i].assignedToChannel = 0;
                svInfo[i].trackedL1 = 0;
                svInfo[i].trackedL2 = 0;
                svInfo[i].trackedAtBaseL1 = 0;
                svInfo[i].trackedAtBaseL2 = 0;
                svInfo[i].usedInSolution = 0;
                svInfo[i].usedInRTK = 0;

                svInfo[i].trackedPcodeL1 = 0;
                svInfo[i].trackedPcodeL2 = 0;

                svInfo[i].elevation = 0;
                svInfo[i].azimuth = 0;
                svInfo[i].snrL1 = 0;
                svInfo[i].snrL2 = 0;
            }

            break;

        case(SVDetail):
            ROS_DEBUG("SVDetail");
            svInfo[0].numSv = rxRecordData.at(index + 2) & 0xFF;

            for(int i = 0; i < svInfo[0].numSv; i++){
                svInfo[i].numSv = svInfo[0].numSv;
                svInfo[i].prn = rxRecordData.at( index + 3 + (i*8) ) & 0xFF;

                svInfo[i].satelliteAboveHorizon = (bool)( rxRecordData.at( index + 4 + (i*8)) & 0b00000001);
                svInfo[i].assignedToChannel = (bool)( rxRecordData.at( index + 4 + (i*8)) & 0b00000010);
                svInfo[i].trackedL1 = (bool)( rxRecordData.at( index + 4 + (i*8)) & 0b00000100);
                svInfo[i].trackedL2 = (bool)( rxRecordData.at( index + 4 + (i*8)) & 0b00001000);
                svInfo[i].trackedAtBaseL1 = (bool)( rxRecordData.at( index + 4 + (i*8)) & 0b00010000);
                svInfo[i].trackedAtBaseL2 = (bool)( rxRecordData.at( index + 4 + (i*8)) & 0b00100000);
                svInfo[i].usedInSolution = (bool)( rxRecordData.at( index + 4 + (i*8)) & 0b01000000);
                svInfo[i].usedInRTK = (bool)( rxRecordData.at( index + 4 + (i*8)) & 0b10000000);

                svInfo[i].trackedPcodeL1 = (bool)( rxRecordData.at( index + 5 + (i*8)) & 0b00000001);
                svInfo[i].trackedPcodeL2 = (bool)( rxRecordData.at( index + 5 + (i*8)) & 0b00000010);

                svInfo[i].elevation = rxRecordData.at( index + 6 + (i*8)) & 0xFF;
                svInfo[i].azimuth = ((rxRecordData.at( index + 7 + (i*8)) << 8) & 0xFF) + (rxRecordData.at( index + 8 + (i*8)) & 0xFF);
                svInfo[i].snrL1 = ((float)(rxRecordData.at( index + 9 + (i*8)) & 0xFF)) / 4.0f;
                svInfo[i].snrL2 = ((float)(rxRecordData.at( index + 10 + (i*8)) & 0xFF)) / 4.0f;
            }

            // Lets zero out the remaining SV info
            for(int i = svInfo[0].numSv; i < 24; i++){
                svInfo[i].numSv = 0;
                svInfo[i].prn = 0;

                svInfo[i].satelliteAboveHorizon = 0;
                svInfo[i].assignedToChannel = 0;
                svInfo[i].trackedL1 = 0;
                svInfo[i].trackedL2 = 0;
                svInfo[i].trackedAtBaseL1 = 0;
                svInfo[i].trackedAtBaseL2 = 0;
                svInfo[i].usedInSolution = 0;
                svInfo[i].usedInRTK = 0;

                svInfo[i].trackedPcodeL1 = 0;
                svInfo[i].trackedPcodeL2 = 0;

                svInfo[i].elevation = 0;
                svInfo[i].azimuth = 0;
                svInfo[i].snrL1 = 0;
                svInfo[i].snrL2 = 0;
            }

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

            positionTimeUTC.newPosition = (rxRecordData.at(index + 9) & 0b00000001);
            positionTimeUTC.newClockThisSolution = (rxRecordData.at(index + 9) & 0b00000010);
            positionTimeUTC.newHorizontalThisSolution = (rxRecordData.at(index + 9) & 0b00000100);
            positionTimeUTC.newHeightThisSolution = (rxRecordData.at(index + 9) & 0b00001000);
            positionTimeUTC.usesLeastSquaresPosition = (rxRecordData.at(index + 9) & 0b00100000);
            positionTimeUTC.usesFilteredL1 = (rxRecordData.at(index + 9) & 0b10000000);

            positionTimeUTC.isDifferential = (rxRecordData.at(index + 10) & 0b00000001);
            positionTimeUTC.isPhase = (rxRecordData.at(index + 10) & 0b00000010);
            positionTimeUTC.isFixedInteger = (rxRecordData.at(index + 10) & 0b00000100);
            positionTimeUTC.isOmnistar = (rxRecordData.at(index + 10) & 0b00001000);
            positionTimeUTC.isStatic = (rxRecordData.at(index + 10) & 0b00010000);
            positionTimeUTC.isNetworkRTK = (rxRecordData.at(index + 10) & 0b00100000);
            positionTimeUTC.isLocationRTK = (rxRecordData.at(index + 10) & 0b01000000);
            positionTimeUTC.isBeaconDGPS = (rxRecordData.at(index + 10) & 0b10000000);

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
            break;

        case(PositionSigma):
            ROS_DEBUG("PositionSigma");
            if(recordLength != 0x26) {
                ROS_ERROR("Invalid message length!");
                break;
            }
            tempFloat.bDat[0] = rxRecordData.at(index + 5) & 0xFF;
            tempFloat.bDat[1] = rxRecordData.at(index + 4) & 0xFF;
            tempFloat.bDat[2] = rxRecordData.at(index + 3) & 0xFF;
            tempFloat.bDat[3] = rxRecordData.at(index + 2) & 0xFF;
            sigma.rms = tempFloat.fData;

            tempFloat.bDat[0] = rxRecordData.at(index + 9) & 0xFF;
            tempFloat.bDat[1] = rxRecordData.at(index + 8) & 0xFF;
            tempFloat.bDat[2] = rxRecordData.at(index + 7) & 0xFF;
            tempFloat.bDat[3] = rxRecordData.at(index + 6) & 0xFF;
            sigma.sigmaEast = tempFloat.fData;

            tempFloat.bDat[0] = rxRecordData.at(index + 13) & 0xFF;
            tempFloat.bDat[1] = rxRecordData.at(index + 12) & 0xFF;
            tempFloat.bDat[2] = rxRecordData.at(index + 11) & 0xFF;
            tempFloat.bDat[3] = rxRecordData.at(index + 10) & 0xFF;
            sigma.sigmaNorth = tempFloat.fData;

            tempFloat.bDat[0] = rxRecordData.at(index + 17) & 0xFF;
            tempFloat.bDat[1] = rxRecordData.at(index + 16) & 0xFF;
            tempFloat.bDat[2] = rxRecordData.at(index + 15) & 0xFF;
            tempFloat.bDat[3] = rxRecordData.at(index + 14) & 0xFF;
            sigma.covarianceEastNorth = tempFloat.fData;

            tempFloat.bDat[0] = rxRecordData.at(index + 21) & 0xFF;
            tempFloat.bDat[1] = rxRecordData.at(index + 20) & 0xFF;
            tempFloat.bDat[2] = rxRecordData.at(index + 19) & 0xFF;
            tempFloat.bDat[3] = rxRecordData.at(index + 18) & 0xFF;
            sigma.sigmaUp = tempFloat.fData;

            tempFloat.bDat[0] = rxRecordData.at(index + 25) & 0xFF;
            tempFloat.bDat[1] = rxRecordData.at(index + 24) & 0xFF;
            tempFloat.bDat[2] = rxRecordData.at(index + 23) & 0xFF;
            tempFloat.bDat[3] = rxRecordData.at(index + 22) & 0xFF;
            sigma.majorAxis = tempFloat.fData;

            tempFloat.bDat[0] = rxRecordData.at(index + 29) & 0xFF;
            tempFloat.bDat[1] = rxRecordData.at(index + 28) & 0xFF;
            tempFloat.bDat[2] = rxRecordData.at(index + 27) & 0xFF;
            tempFloat.bDat[3] = rxRecordData.at(index + 26) & 0xFF;
            sigma.minorAxis = tempFloat.fData;

            tempFloat.bDat[0] = rxRecordData.at(index + 33) & 0xFF;
            tempFloat.bDat[1] = rxRecordData.at(index + 32) & 0xFF;
            tempFloat.bDat[2] = rxRecordData.at(index + 31) & 0xFF;
            tempFloat.bDat[3] = rxRecordData.at(index + 30) & 0xFF;
            sigma.orientation = tempFloat.fData;

            tempFloat.bDat[0] = rxRecordData.at(index + 37) & 0xFF;
            tempFloat.bDat[1] = rxRecordData.at(index + 36) & 0xFF;
            tempFloat.bDat[2] = rxRecordData.at(index + 35) & 0xFF;
            tempFloat.bDat[3] = rxRecordData.at(index + 34) & 0xFF;
            sigma.unitVariance = tempFloat.fData;

            sigma.numEpochs =   ((int)(rxRecordData.at(index + 38) & 0xFF) << 8) +
                                ((int)(rxRecordData.at(index + 39) & 0xFF) << 0);
            break;

        case(Velocity):
            ROS_DEBUG("Velocity");
            velocity.velocityValid = (bool)(rxRecordData.at(index + 2) & 0b00000001);
            velocity.computeMethod = (bool)(rxRecordData.at(index + 2) & 0b00000010);

            tempFloat.bDat[0] = rxRecordData.at(index + 6) & 0xFF;
            tempFloat.bDat[1] = rxRecordData.at(index + 5) & 0xFF;
            tempFloat.bDat[2] = rxRecordData.at(index + 4) & 0xFF;
            tempFloat.bDat[3] = rxRecordData.at(index + 3) & 0xFF;
            velocity.horizontalVelocity = tempFloat.fData;

            tempFloat.bDat[0] = rxRecordData.at(index + 10) & 0xFF;
            tempFloat.bDat[1] = rxRecordData.at(index +  9) & 0xFF;
            tempFloat.bDat[2] = rxRecordData.at(index +  8) & 0xFF;
            tempFloat.bDat[3] = rxRecordData.at(index +  7) & 0xFF;
            velocity.heading = tempFloat.fData * 57.2957795f; // TODO: Change to constant

            tempFloat.bDat[0] = rxRecordData.at(index + 14) & 0xFF;
            tempFloat.bDat[1] = rxRecordData.at(index + 13) & 0xFF;
            tempFloat.bDat[2] = rxRecordData.at(index + 12) & 0xFF;
            tempFloat.bDat[3] = rxRecordData.at(index + 11) & 0xFF;
            velocity.verticalVelocity = tempFloat.fData;

            break;

        case(PositionVCV):
            ROS_DEBUG("PositionVCV");
            tempFloat.bDat[0] = rxRecordData.at(index + 5) & 0xFF;
            tempFloat.bDat[1] = rxRecordData.at(index + 4) & 0xFF;
            tempFloat.bDat[2] = rxRecordData.at(index + 3) & 0xFF;
            tempFloat.bDat[3] = rxRecordData.at(index + 2) & 0xFF;
            varianceCovariance.positionRMS = tempFloat.fData;

            tempFloat.bDat[0] = rxRecordData.at(index + 9) & 0xFF;
            tempFloat.bDat[1] = rxRecordData.at(index + 8) & 0xFF;
            tempFloat.bDat[2] = rxRecordData.at(index + 7) & 0xFF;
            tempFloat.bDat[3] = rxRecordData.at(index + 6) & 0xFF;
            varianceCovariance.xx = tempFloat.fData;

            tempFloat.bDat[0] = rxRecordData.at(index + 13) & 0xFF;
            tempFloat.bDat[1] = rxRecordData.at(index + 12) & 0xFF;
            tempFloat.bDat[2] = rxRecordData.at(index + 11) & 0xFF;
            tempFloat.bDat[3] = rxRecordData.at(index + 10) & 0xFF;
            varianceCovariance.xy = tempFloat.fData;

            tempFloat.bDat[0] = rxRecordData.at(index + 17) & 0xFF;
            tempFloat.bDat[1] = rxRecordData.at(index + 16) & 0xFF;
            tempFloat.bDat[2] = rxRecordData.at(index + 15) & 0xFF;
            tempFloat.bDat[3] = rxRecordData.at(index + 14) & 0xFF;
            varianceCovariance.xz = tempFloat.fData;

            tempFloat.bDat[0] = rxRecordData.at(index + 21) & 0xFF;
            tempFloat.bDat[1] = rxRecordData.at(index + 20) & 0xFF;
            tempFloat.bDat[2] = rxRecordData.at(index + 19) & 0xFF;
            tempFloat.bDat[3] = rxRecordData.at(index + 18) & 0xFF;
            varianceCovariance.yy = tempFloat.fData;

            tempFloat.bDat[0] = rxRecordData.at(index + 25) & 0xFF;
            tempFloat.bDat[1] = rxRecordData.at(index + 24) & 0xFF;
            tempFloat.bDat[2] = rxRecordData.at(index + 23) & 0xFF;
            tempFloat.bDat[3] = rxRecordData.at(index + 22) & 0xFF;
            varianceCovariance.yz = tempFloat.fData;

            tempFloat.bDat[0] = rxRecordData.at(index + 29) & 0xFF;
            tempFloat.bDat[1] = rxRecordData.at(index + 28) & 0xFF;
            tempFloat.bDat[2] = rxRecordData.at(index + 27) & 0xFF;
            tempFloat.bDat[3] = rxRecordData.at(index + 26) & 0xFF;
            varianceCovariance.zz = tempFloat.fData;

            tempFloat.bDat[0] = rxRecordData.at(index + 33) & 0xFF;
            tempFloat.bDat[1] = rxRecordData.at(index + 32) & 0xFF;
            tempFloat.bDat[2] = rxRecordData.at(index + 31) & 0xFF;
            tempFloat.bDat[3] = rxRecordData.at(index + 30) & 0xFF;
            varianceCovariance.unitVariance = tempFloat.fData;

            varianceCovariance.numEpochs = ((int)(rxRecordData.at(index + 35) & 0xFF) << 8) +
                                           ((int)(rxRecordData.at(index + 34) & 0xFF) << 0);
            break;

        default:
        ROS_WARN("Unknown msg type 0x%02X.", recordType);
        }

        // Next record starts at the previous plus the type and header bytes
        index += recordLength + 2;
    }
}
