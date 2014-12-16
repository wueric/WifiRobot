#include "mbed.h"
#include "HMC5883L.h"
#include "UltrasoundAnalog.h"
#include "Localizer.h"
#include "irobot_drive_sense.h"
#include "IntensityScanner.h"

#define ULTRASOUND_AFFINE_X_SLOPE 50.1
#define ULTRASOUND_AFFINE_X_BIAS -292.0

/* UNCOMMENT THIS STUFF FOR NON STATE MACHINE STUFF
Timer timer;
Serial bluetooth(PTA2, PTA1);
HMC5883L compass(PTC9, PTC8);
Serial device(PTC4, PTC3);
UltrasoundAnalog ultrasoundDistance(PTC2,
    1.0 / ULTRASOUND_AFFINE_X_SLOPE,
    -ULTRASOUND_AFFINE_X_BIAS /  ULTRASOUND_AFFINE_X_SLOPE);
IntensityScanner rssi_scanner(D3, D5, D10, SPI(D11, D12, PTC5),
    "Joshi", "Rajashri1", mbed_cc3000::WPA, false, 200);
    */


// setup irobot
Serial device(PTC4, PTC3);
Timer timer;
// setup bluetooth
Serial bluetooth(PTA2, PTA1);
// setup compass and bias
HMC5883L compass(PTC9, PTC8);
// setup ultrasound scanner
UltrasoundAnalog ultrasoundDistance(PTC2,
    1.0 / ULTRASOUND_AFFINE_X_SLOPE,
    -ULTRASOUND_AFFINE_X_BIAS /  ULTRASOUND_AFFINE_X_SLOPE);

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);

typedef enum RobotState_t {
    FORWARD,
    TURN_CLOCKWISE,
    TURN_COUNTERCLOCKWISE,
    MEASURE,
    STOP,
    GOTO_MAX
} RobotState_t;

typedef enum PossibleHeadings_t {
    NORTH,
    EAST,
    SOUTH,
    WEST
} PossibleHeadings_t;


struct Headings {
    double north;
    double east;
    double south;
    double west;
};


void initializeHeadings (HMC5883L* magneto, Headings* heading) {
    double north = magneto->getHeadingXZDeg();

    double east = north + 90.0;
    double south = east + 90.0;
    double west = south + 90.0;

    heading->north = north;
    heading->east = (east > 360.0) ? (east - 360.0) : east;
    heading->south = (south > 360.0) ? (south - 360.0) : south;
    heading ->west = (west > 360.0) ? (west - 360.0) : west;
}

double absolute_value (double value) {
    return (value < 0) ? -value : value;
}

double abs_subtract_angle (double a1, double a2) {
    double abs_difference = absolute_value(a1 - a2);
    if (abs_difference > 180.0)
        return absolute_value(abs_difference - 360.0);
    return abs_difference;
}

void robotControl (Serial* robot,
            Serial* bluetooth,
            HMC5883L* magneto, 
            UltrasoundAnalog* ultra, 
            Timer* timer,
            IntensityScanner* scanner,
            uint8_t* rssiArray,
            uint32_t boxHeight,
            uint32_t boxWidth) {
                
    uint8_t rssi_reading;

    // Initialize state variables
    RobotState_t robotState = MEASURE;
    PossibleHeadings_t currHeading = NORTH;

    Headings heading;
    initializeHeadings(magneto, &heading);
    
    bluetooth->printf("%f N, %f E, %f S, %f W\r\n",
        heading.north,
        heading.east,
        heading.south,
        heading.west);

    double distanceAtManeuverStart = ultra->determineDistance();
    double angleAtManeuverStart = magneto->getHeadingXZDeg();

    uint32_t currentXCoordinate = 0;
    uint32_t currentYCoordinate = 0;

    // End state variables


    // Begin other variables
    double currentDistance;
    double currentAngle;
    double targetAngle;
    
    bool inSearchPattern = true;

    while (inSearchPattern) {
        switch (robotState) {

            case FORWARD:
                {
                    led1 = 1;
                    led2 = 0;

                    // stop and update state variables
                    irobotDriveSense::setWheelSpeed(*robot, 0, 0);
                    distanceAtManeuverStart = ultra->determineDistance();
                    angleAtManeuverStart = magneto->getHeadingXZDeg();

                    bluetooth->printf("In state FORWARD\r\n");
                    
                    timer->stop();
                    timer->reset();
                    timer->start();
                    irobotDriveSense::setWheelSpeed(*robot, 130, 130);
                    currentDistance = ultra->determineDistance();
                    while ( timer->read_ms() < 2700) { // hard timeout for inaccurate distance sensor
                        /*
                        We have an approximate bound on how long it takes to go 30 cm.
                        If ultrasound reading is inaccurate and reads 30 cm change outside of this
                            boundary then we ignore the ultrasound reading
                        */

                        currentDistance = ultra->determineDistance();

                        if (absolute_value(currentDistance - distanceAtManeuverStart) > 30.0
                            && timer->read_ms() > 2250) {
                            bluetooth->printf("distance sensor condition triggered\r\n");
                            break;
                        }
                        
                        //bluetooth->printf("%f\r\n", currentDistance);
                        wait_ms(100);
                    }

                    switch (currHeading) {
                        case NORTH:
                            currentYCoordinate++;
                            break;
                        case SOUTH:
                            currentYCoordinate--;
                            break;
                        case EAST:
                            currentXCoordinate++;
                            break;
                        case WEST:
                            currentXCoordinate--;
                            break;
                    }

                    // define new transitions

                    if (currHeading == NORTH || currHeading == SOUTH) {
                        robotState = MEASURE;
                    } else if (currHeading == EAST) {
                        if (currentYCoordinate == boxHeight - 1) {
                            robotState = TURN_CLOCKWISE;
                        } else if (currentYCoordinate == 0) {
                            robotState = TURN_COUNTERCLOCKWISE;
                        }
                    }
                    break;

                }

            case TURN_CLOCKWISE: // turn from NORTH to EAST, move, and then turn from EAST to SOUTH
                {
                    // stop and update state variables
                    irobotDriveSense::setWheelSpeed(*robot, 0, 0);
                    distanceAtManeuverStart = ultra->determineDistance();
                    angleAtManeuverStart = magneto->getHeadingXZDeg();

                    bluetooth->printf("In state TURN_CLOCKWISE\r\n");

                    currentAngle = angleAtManeuverStart;
    
                    switch (currHeading) {
                        case NORTH:
                            targetAngle = heading.east;
                            break;
                        case EAST:
                            targetAngle = heading.south;
                            break;
                        case SOUTH:
                            targetAngle = heading.west;
                            break;
                        case WEST:
                            targetAngle = heading.north;
                            break;
                    }

                    timer->reset();
                    timer->start();
                    irobotDriveSense::setWheelSpeed(*robot, -130, 130);
                    while (timer->read_ms() < 1800) { // hard timeout for inaccurate compass
                        /*
                        We have an approximate bound on how long it takes to turn 90 degrees.
                        If compass is inaccurate and reads 90 degree change outside of this
                            boundary then we ignore the compass
                        */
                        currentAngle = magneto->getHeadingXZDeg();
                        
                        if (abs_subtract_angle(currentAngle, targetAngle) < 5.0 
                                && timer->read_ms() > 1500) {
                            bluetooth->printf("angle sensor condition triggered\r\n");
                            break;
                        }
                        
                        //bluetooth->printf("%f angle diff\r\n", abs_subtract_angle(currentAngle, targetAngle));
                        wait_ms(100);
                    }

                    // update state variables after turn
                    if (currHeading == NORTH) currHeading = EAST;
                    else if (currHeading == EAST) currHeading = SOUTH;
                    else if (currHeading == SOUTH) currHeading = WEST;
                    else if (currHeading == WEST) currHeading = NORTH;

                    // transitions
                    if (currHeading == EAST || currHeading == WEST) {
                        robotState = FORWARD;
                    } else {
                        robotState = MEASURE;
                    }
                    break;

                }

            case TURN_COUNTERCLOCKWISE: // turn from NORTH to EAST, move, and then turn from EAST to SOUTH
                {
                    // stop and update state variables
                    irobotDriveSense::setWheelSpeed(*robot, 0, 0);
                    distanceAtManeuverStart = ultra->determineDistance();
                    angleAtManeuverStart = magneto->getHeadingXZDeg();

                    bluetooth->printf("In state TURN_COUNTERCLOCKWISE\r\n");

                    currentAngle = angleAtManeuverStart;
    
                    switch (currHeading) {
                        case NORTH:
                            targetAngle = heading.west;
                            break;
                        case EAST:
                            targetAngle = heading.north;
                            break;
                        case SOUTH:
                            targetAngle = heading.east;
                            break;
                        case WEST:
                            targetAngle = heading.south;
                            break;
                    }

                    timer->reset();
                    timer->start();
                    irobotDriveSense::setWheelSpeed(*robot, 130, -130);
                    while (timer->read_ms() < 1800) { // hard timeout for inaccurate compass
                        /*
                        We have an approximate bound on how long it takes to turn 90 degrees.
                        If compass is inaccurate and reads 90 degree change outside of this
                            boundary then we ignore the compass
                        */
                        
                        currentAngle = magneto->getHeadingXZDeg();
                        
                        if (abs_subtract_angle(currentAngle, targetAngle) < 5.0 
                                && timer->read_ms() > 1500) {
                            bluetooth->printf("angle sensor condition triggered\r\n");
                            break;
                        }
                        //bluetooth->printf("%f angle diff\r\n", abs_subtract_angle(currentAngle, targetAngle));
                        wait_ms(100);
                    }

                    // update state variables after turn
                    if (currHeading == NORTH) currHeading = WEST;
                    else if (currHeading == EAST) currHeading = NORTH;
                    else if (currHeading == SOUTH) currHeading = EAST;
                    else if (currHeading == WEST) currHeading = SOUTH;

                    // transitions
                    if (currHeading == EAST || currHeading == WEST) {
                        robotState = FORWARD;
                    } else {
                        robotState = MEASURE;
                    }
                    break;
                }

            case MEASURE:
                {            
                    led1 = 1;
                    led2 = 0;
    
                    distanceAtManeuverStart = ultra->determineDistance();
                    angleAtManeuverStart = magneto->getHeadingXZDeg();
                    irobotDriveSense::setWheelSpeed(*robot, 0, 0);
                    wait_ms(500);
                    scanner->rssi_for_ssid("Joshi", &rssi_reading);
                    bluetooth->printf("RSSI value %d, coordinates (%d, %d)\r\n", rssi_reading, currentXCoordinate, currentYCoordinate);
                    *(rssiArray + currentXCoordinate + currentYCoordinate * boxHeight) = rssi_reading;
    
                    bluetooth->printf("In state MEASURE\r\n");
    
                    wait_ms(250);
                    if (currentXCoordinate < boxWidth) {
                        if (currentYCoordinate == boxHeight - 1 && currHeading == NORTH) {
                            robotState = TURN_CLOCKWISE;   
                        } else if (currentYCoordinate == 0 && currHeading == SOUTH) {
                            robotState = TURN_COUNTERCLOCKWISE;   
                        } else {
                            robotState = FORWARD;   
                        }
                    } else {
                        robotState = STOP;
                    }
                    break;
                }
            case STOP:
                {
                    inSearchPattern = false;
                    distanceAtManeuverStart = ultra->determineDistance();
                    angleAtManeuverStart = magneto->getHeadingXZDeg();
                    irobotDriveSense::setWheelSpeed(*robot, 0, 0);
                    break;
                }
        }

    }

}



/*
void forward_one(){
    timer.reset();
    timer.start();
    irobotDriveSense::setWheelSpeed(device, 130, 130); // test drive forward
    while (timer.read_ms() < 2460);
    irobotDriveSense::setWheelSpeed(device, 0, 0);
    while( timer.read_ms() < 2760);
}

//-1 for right, 1 for left
void turn(int direction){
    //double currAngle = compass.getHeadingXZDeg();
    timer.reset();
    timer.start();
    irobotDriveSense::setWheelSpeed(device, 130 * direction, -130 * direction);
    while (timer.read_ms() < 1664);
    irobotDriveSense::setWheelSpeed(device, 0, 0);
    while( timer.read_ms() < 2164);
}

void turnSequence(int direction){
    turn(direction);
    forward_one();
    turn(direction);    
}
*/

int main(void)
{
    device.baud(57600);
    irobotDriveSense::start(device);
    
    wait_ms(1000);
    led3 = 1;

    bluetooth.baud(115200);
    
    compass.init();
    compass.setConstantOffset(-413, 0, 396);
    
    bluetooth.printf("setting up wifi shield\r\n");
    // setup wifi shield
    IntensityScanner rssi_scanner(D3, D5, D10, SPI(D11, D12, PTC5),
        "Joshi", "Rajashri1", mbed_cc3000::WPA, false, 200);
    bluetooth.printf("done setting up wifi shield\r\n");

    wait(2);
    
    /*
    int16_t data[3];
    while (1) {
        irobotDriveSense::setWheelSpeed(device, 130, -130);
        compass.getXYZ(data);
        bluetooth.printf("%d,%d\r\n", data[0], data[2]);
        wait_ms(50);
        
    }
    */
    
    int32_t xDim = 3;
    int32_t yDim = 3;
    uint8_t* destinationArray = (uint8_t*) malloc (sizeof(uint8_t) * xDim * yDim);
    robotControl (&device,
            &bluetooth,
            &compass, 
            &ultrasoundDistance, 
            &timer,
            &rssi_scanner,
            destinationArray,
            xDim,
            yDim);
            
    free(destinationArray);
    
    /*
    bool turnRight = true;
    
    wait(2);
    uint8_t rssi;
    for(int j = 0; j < 1; j++){
        for(int i = 0; i < 3; i++){
            double ultrasoundMeasureChecker;
            int16_t distanceChecker;
            forward_one();
            // rssi_scanner.rssi_for_ssid("Joshi", &rssi);
            irobotDriveSense::poll_netDistance(device, &distanceChecker);
            ultrasoundMeasureChecker = ultrasoundDistance.determineDistance();
            bluetooth.printf("%d rssi %d distance %f\r\n", rssi, distanceChecker, ultrasoundMeasureChecker);
        }
        int direction = (turnRight) ? (-1) : (1);
        turnSequence(direction);
        turnRight = !turnRight;
    }
    
    while (1) {
        bluetooth.printf("%f \r\n", ultrasoundDistance.determineDistance());
        wait_ms(100);
    }
    */
    
}
