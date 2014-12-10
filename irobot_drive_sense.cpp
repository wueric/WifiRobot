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
    
    *angle = 0;
    

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

    int16_t highByte = 0;
    int16_t lowByte = 0;
    int8_t checksum = 0;

    device.printf("%c%c", OP_SENSORS, SENSOR_DISTANCE);
    while (!device.readable());

    highByte = (int16_t) device.getc();
    *distance |= ((highByte & 0xFF) << 8);

    while (!device.readable());

    lowByte = (int16_t) device.getc();
    *distance |= (lowByte & 0xFF);

    return;

}


} // closing namespace iRobotDriveSense


