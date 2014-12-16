/*
Look at the header file!!
*/

#include "irobot_drive_sense.h"

namespace irobotDriveSense {


void start (Serial& device) {
    device.putc(OP_START);
    device.putc(OP_FULL);
}

void setWheelSpeed (Serial& device,
    int16_t rightWheel, 
    int16_t leftWheel) {

    device.printf("%c%c%c%c%c", OP_DRIVE_DIRECT, (rightWheel >> 8),
        (rightWheel & 0xFF), (leftWheel >> 8), (leftWheel & 0xFF));
}

void poll_netAngle (Serial& device,
    int16_t* angle) {

    int16_t highByte = 0;
    int16_t lowByte = 0;
    
    device.printf("%c%c", OP_SENSORS, SENSOR_ANGLE);
    while (!device.readable());

    highByte = (int16_t) device.getc();
    *angle |= ((highByte & 0xFF) << 8);

    while (!device.readable());

    lowByte = (int16_t) device.getc();
    *angle |= (lowByte & 0xFF);

    return;
}

void poll_netDistance(Serial& device,
    int16_t* distance) {
        
    *distance = 0x0;
        
    while (device.readable())
        device.getc();

    int16_t highByte = 0;
    int16_t lowByte = 0;

    device.printf("%c%c", OP_SENSORS, SENSOR_DISTANCE);

    while (!device.readable());

    highByte = (int16_t) device.getc();
    *distance |= ((highByte & 0xFF) << 8);

    while (!device.readable());

    lowByte = (int16_t) device.getc();
    *distance |= (lowByte & 0xFF);

    return;

}

void poll_bumpSensor(Serial& device,
    BumpDropBits* bumpBits) {

    device.printf("%c%c", OP_SENSORS, SENSOR_BUMPS_WHEELDROPS);

    while (!device.readable());
    
    char dataBytes  = device.getc();
    bumpBits->wheeldrop_castor = dataBytes & 0x10;
    bumpBits->wheeldrop_left = dataBytes & 0x08;
    bumpBits->wheeldrop_right = dataBytes & 0x04;
    bumpBits->bump_left = dataBytes & 0x02;
    bumpBits->bump_right = dataBytes & 0x01;
}


} // closing namespace iRobotDriveSense