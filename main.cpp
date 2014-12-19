#include "mbed.h"
#include "HMC5883L.h"
#include "UltrasoundAnalog.h"
#include "irobot_drive_sense.h"
#include "IntensityScanner.h"

#define ULTRASOUND_AFFINE_X_SLOPE 50.1
#define ULTRASOUND_AFFINE_X_BIAS -292.0

#define NINETY_DEGREE_TURN 1600.0
#define NINETY_DEGREE_TURN_LOWER_BOUND 1500.0
#define NINETY_DEGREE_TURN_UPPER_BOUND 1700.0
#define ULTRASOUND_UNRELIABLE_UPPER_BOUND 12000

// setup irobot
Serial device(PTC4, PTC3);

// setup timer
Timer timer;

// setup bluetooth
Serial bluetooth(PTA2, PTA1);

// setup compass and bias
HMC5883L compass(PTC9, PTC8);

// setup ultrasound scanner
UltrasoundAnalog ultrasoundDistance(PTC2,
    1.0 / ULTRASOUND_AFFINE_X_SLOPE,
    -ULTRASOUND_AFFINE_X_BIAS /  ULTRASOUND_AFFINE_X_SLOPE);

// setup LEDs for debugging help
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);


// Type for robot finite state machine states
typedef enum RobotState_t {
    FORWARD,
    TURN_CLOCKWISE,
    TURN_COUNTERCLOCKWISE,
    MEASURE,
    STOP,
    TURNAROUND
} RobotState_t;


// Type for representing the possible directions that
//  the robot can travel
typedef enum PossibleHeadings_t {
    NORTH,
    EAST,
    SOUTH,
    WEST
} PossibleHeadings_t;

// type for representing angles for the directions that
//  the robot should travel in
struct Headings {
    double north;
    double east;
    double south;
    double west;
};

/* determines angle values for north, south, east, west
    @param HMC5883L* magneto - pointer to the object that represents
        the magnetometer
    @parm Headings* heading - pointer to the Headings struct that represents
        the valid directions. The struct is modified in this function
    @param PossibleHeadings_t currHeading - the heading that the robot currently
        has
*/
void initializeHeadings (HMC5883L* magneto, 
    Headings* heading, 
    PossibleHeadings_t currHeading) {
    
    double measuredDirections[4];
    double firstValue = magneto->getHeadingXZDeg();
    
    double acc = firstValue;
    for (int i = 0; i < 4; ++i) {
        measuredDirections[i] = (acc > 360.0) ? (acc - 360.0) : acc;
        acc += 90;
    }
    
    if (currHeading == NORTH) {
        heading->north = measuredDirections[0];
        heading->east = measuredDirections[1];
        heading->south = measuredDirections[2];
        heading->west = measuredDirections[3];
    } else if (currHeading == EAST) {
        heading->north = measuredDirections[3];
        heading->east = measuredDirections[0];
        heading->south = measuredDirections[1];
        heading->west = measuredDirections[2];       
    } else if (currHeading == SOUTH) {
        heading->north = measuredDirections[2];
        heading->east = measuredDirections[3];
        heading->south = measuredDirections[0];
        heading->west = measuredDirections[1];
    } else if (currHeading == SOUTH) {
        heading->north = measuredDirections[1];
        heading->east = measuredDirections[2];
        heading->south = measuredDirections[3];
        heading->west = measuredDirections[0];
    }
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


/*
Finite state machine for searching in a X-by-Y grid pattern,
with Y as the major axis. Robot starts at (0, 0) and goes along
the y-axis first.

    @param Serial* robot - pointer to the Serial object
        representing the connection to the robot
    @param Serial* bluetooth - pointer to the Serial object
        representing the connection to the bluetooth module
    @param HMC5883L* magneto - pointer to the HMC5883L object
        representing the magnetometer
    @param Timer* timer - pointer to Timer.
    @param IntensityScanner* scanner - pointer to the IntensityScanner
        class, which wraps around the mbed_cc3000 class. This will
        perform the signal intensity scans
    @param uint8_t* rssiArray - array where the RSSI values that are
        measured will be stored. Should be at least boxHeight * boxWidth
        bytes in size, otherwise a segfault will occur
    @param uint32_t boxHeight - height (y coordinate) of the box being
        searched
    @param uint32_t boxWidth - width (x coordinate) of the box being searched
*/
PossibleHeadings_t robotControl (Serial* robot,
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
    
    
    initializeHeadings(magneto, &heading, currHeading);
  

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
                    
                    timer->stop();
                    timer->reset();
                    timer->start();
                    irobotDriveSense::setWheelSpeed(*robot, 130, 130);
                    currentDistance = ultra->determineDistance();
                    while ( timer->read_ms() < 7400) { // hard timeout for inaccurate distance sensor
                        /*
                        We have an approximate bound on how long it takes to go 30 cm.
                        If ultrasound reading is inaccurate and reads 30 cm change outside of this
                            boundary then we ignore the ultrasound reading
                        */

                        currentDistance = ultra->determineDistance();

                        if (absolute_value(currentDistance - distanceAtManeuverStart) > 90.0
                            && timer->read_ms() > 6750) {
                            break;
                        }
                    
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
                    wait_ms(200);
                    distanceAtManeuverStart = ultra->determineDistance();
                    angleAtManeuverStart = magneto->getHeadingXZDeg();

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
                    while (timer->read_ms() < NINETY_DEGREE_TURN_UPPER_BOUND) { // hard timeout for inaccurate compass
                        
                        // We have an approximate bound on how long it takes to turn 90 degrees.
                        // If compass is inaccurate and reads 90 degree change outside of this
                        //    boundary then we ignore the compass
                        
                        currentAngle = magneto->getHeadingXZDeg();
                        
                        if (abs_subtract_angle(currentAngle, targetAngle) < 5.0 
                                && timer->read_ms() > NINETY_DEGREE_TURN_LOWER_BOUND) {
                            break;
                        }
                        
                        wait_ms(50);
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
                    wait_ms(200);
                    distanceAtManeuverStart = ultra->determineDistance();
                    angleAtManeuverStart = magneto->getHeadingXZDeg();


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
                    while (timer->read_ms() < NINETY_DEGREE_TURN_UPPER_BOUND) { // hard timeout for inaccurate compass
                        
                        // We have an approximate bound on how long it takes to turn 90 degrees.
                        // If compass is inaccurate and reads 90 degree change outside of this
                        //    boundary then we ignore the compass
                        
                        currentAngle = magneto->getHeadingXZDeg();
                        
                        if (abs_subtract_angle(currentAngle, targetAngle) < 5.0 
                                && timer->read_ms() > NINETY_DEGREE_TURN_LOWER_BOUND) {
                            break;
                        }
                        wait_ms(50);
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
                    if (scanner->rssi_for_ssid("AirBears", &rssi_reading) == 0) {;
                        bluetooth->printf("RSSI value %d, coordinates (%d, %d)\r\n", 
                            rssi_reading, 
                            currentXCoordinate, 
                            currentYCoordinate);
                        *(rssiArray + currentXCoordinate + currentYCoordinate * boxHeight) = rssi_reading;
                    } else {
                        *(rssiArray + currentXCoordinate + currentYCoordinate * boxHeight) = 0x0;   
                    }
    
                    wait_ms(250);
                    if (currentXCoordinate == boxWidth - 1 && currentYCoordinate == boxHeight - 1) {
                        robotState = STOP;
                    } else if (currentXCoordinate < boxWidth) {
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
    
    return currHeading;
}

/*
Finite state machine for going to a specified (x, y) coordinate

Assumptions:
    - robot is located at either (xMax, yMax) and facing north (i.e.
        in the +y direction) or at (xMax, 0) and facing south (i.e. 
        in the -y direction)

Robot will do a 180 degree turn, and then travel the necessary y steps
    first and then travel the necessary x steps to reach the supplied
    coordinates

    @param Serial* robot - pointer to the Serial object
        representing the connection to the robot
    @param Serial* bluetooth - pointer to the Serial object
        representing the connection to the bluetooth module
    @param HMC5883L* magneto - pointer to the HMC5883L object
        representing the magnetometer
    @param Timer* timer - pointer to Timer.
    @param PossibleHeadings_t currrHeading - the current heading of the robot.
        As implmented here it should be either NORTH or SOUTH
    @param uint32_t currentXCoordinate - the x coordinate that the robot is
        initially located at when the function is invoked
    @param uint32_t currentYCoordinate - the y coordinate that the robot is
        initially located at when the funciton is invoked
    @param uint32_t xDestination - the x coordinate of the destination that
        the robot is supposed to go to
    @param uint32_t yDestination - the y coordinate of the destination that
        the robot is supposed to go to
*/

void robotFindMax (Serial* robot,
            Serial* bluetooth,
            HMC5883L* magneto, 
            UltrasoundAnalog* ultra, 
            Timer* timer,
            PossibleHeadings_t currHeading,
            uint32_t currentXCoordinate,
            uint32_t currentYCoordinate,
            uint32_t xDestination,
            uint32_t yDestination) {
                
     // Initialize state variables
    RobotState_t robotState = TURNAROUND;

    Headings heading;
    initializeHeadings(magneto, &heading, currHeading);

    double distanceAtManeuverStart = ultra->determineDistance();
    double angleAtManeuverStart = magneto->getHeadingXZDeg();
    // End state variables


    // Begin other variables
    double currentDistance;
    double currentAngle;
    double targetAngle;
    
    bool inSearchPattern = true;

    while (inSearchPattern) {
        switch (robotState) {
            
            case TURNAROUND:
                {
                    // stop and update state variables
                    irobotDriveSense::setWheelSpeed(*robot, 0, 0);
                    distanceAtManeuverStart = ultra->determineDistance();
                    angleAtManeuverStart = magneto->getHeadingXZDeg();

                    bluetooth->printf("In state TURNAROUND\r\n");

                    currentAngle = angleAtManeuverStart;
    
                    switch (currHeading) {
                        case NORTH:
                            targetAngle = heading.south;
                            break;
                        case EAST:
                            targetAngle = heading.west;
                            break;
                        case SOUTH:
                            targetAngle = heading.north;
                            break;
                        case WEST:
                            targetAngle = heading.east;
                            break;
                    }

                    timer->reset();
                    timer->start();
                    irobotDriveSense::setWheelSpeed(*robot, -130, 130);
                    while (timer->read_ms() < 2 * NINETY_DEGREE_TURN_UPPER_BOUND) { // hard timeout for inaccurate compass
                        /*
                        We have an approximate bound on how long it takes to turn 90 degrees.
                        If compass is inaccurate and reads 90 degree change outside of this
                            boundary then we ignore the compass
                        */
                        currentAngle = magneto->getHeadingXZDeg();
                        
                        if (abs_subtract_angle(currentAngle, targetAngle) < 5.0 
                                && timer->read_ms() > 2 * NINETY_DEGREE_TURN_LOWER_BOUND) {
                            bluetooth->printf("angle sensor condition triggered\r\n");
                            break;
                        }
                        
                        wait_ms(100);
                    }

                    // update state variables after turn
                    if (currHeading == NORTH) currHeading = SOUTH;
                    else if (currHeading == EAST) currHeading = WEST;
                    else if (currHeading == SOUTH) currHeading = NORTH;
                    else if (currHeading == WEST) currHeading = EAST;

                    // transitions
                    if (currHeading == NORTH && currentYCoordinate < yDestination) {
                        robotState = FORWARD;
                    } else if (currHeading == SOUTH && currentYCoordinate > yDestination) {
                        robotState = FORWARD;   
                    } else if (currentYCoordinate == yDestination) {
                        if (currHeading == NORTH) robotState = TURN_COUNTERCLOCKWISE;
                        else if (currHeading == SOUTH) robotState = TURN_CLOCKWISE;    
                    }
                    
                    break;   
                }

            case FORWARD:
                {
                    led1 = 1;
                    led2 = 0;

                    // stop and update state variables
                    irobotDriveSense::setWheelSpeed(*robot, 0, 0);
                    wait_ms(200);
                    distanceAtManeuverStart = ultra->determineDistance();
                    angleAtManeuverStart = magneto->getHeadingXZDeg();

                    bluetooth->printf("In state FORWARD\r\n");
                    
                    timer->stop();
                    timer->reset();
                    timer->start();
                    irobotDriveSense::setWheelSpeed(*robot, 130, 130);
                    currentDistance = ultra->determineDistance();
                    while ( timer->read_ms() < 7400) { // hard timeout for inaccurate distance sensor
                        /*
                        We have an approximate bound on how long it takes to go 30 cm.
                        If ultrasound reading is inaccurate and reads 30 cm change outside of this
                            boundary then we ignore the ultrasound reading
                        */

                        currentDistance = ultra->determineDistance();

                        if (absolute_value(currentDistance - distanceAtManeuverStart) > 90.0
                            && timer->read_ms() > 6750) {
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
                    if (currHeading == NORTH && currentYCoordinate < yDestination) {
                        robotState = FORWARD;
                    } else if (currHeading == NORTH && currentYCoordinate == yDestination) {
                        robotState = TURN_COUNTERCLOCKWISE;   
                    } else if (currHeading == SOUTH && currentYCoordinate > yDestination) {
                        robotState = FORWARD;
                    } else if (currHeading == SOUTH && currentYCoordinate == yDestination) {
                        robotState = TURN_CLOCKWISE;    
                    } else if (currHeading == EAST && currentXCoordinate < xDestination) {
                        robotState = FORWARD;   
                    } else if (currHeading == EAST && currentXCoordinate == xDestination) {
                        robotState = STOP;   
                    } else if (currHeading == WEST && currentXCoordinate > xDestination) {
                        robotState = FORWARD;   
                    } else if (currHeading == WEST && currentXCoordinate == xDestination) {
                        robotState = STOP;
                    } else {
                        robotState = STOP;
                    }

                    break;

                }

            case TURN_CLOCKWISE: // turn from NORTH to EAST, move, and then turn from EAST to SOUTH
                {
                    // stop and update state variables
                    irobotDriveSense::setWheelSpeed(*robot, 0, 0);
                    wait_ms(100);
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
                    while (timer->read_ms() < NINETY_DEGREE_TURN_UPPER_BOUND) { // hard timeout for inaccurate compass
                        /*
                        We have an approximate bound on how long it takes to turn 90 degrees.
                        If compass is inaccurate and reads 90 degree change outside of this
                            boundary then we ignore the compass
                        */
                        currentAngle = magneto->getHeadingXZDeg();
                        
                        if (abs_subtract_angle(currentAngle, targetAngle) < 5.0 
                                && timer->read_ms() > NINETY_DEGREE_TURN_LOWER_BOUND) {
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
                    robotState = FORWARD;
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
                    while (timer->read_ms() < NINETY_DEGREE_TURN_UPPER_BOUND) { // hard timeout for inaccurate compass
                        /*
                        We have an approximate bound on how long it takes to turn 90 degrees.
                        If compass is inaccurate and reads 90 degree change outside of this
                            boundary then we ignore the compass
                        */
                        
                        currentAngle = magneto->getHeadingXZDeg();
                        
                        if (abs_subtract_angle(currentAngle, targetAngle) < 5.0 
                                && timer->read_ms() > NINETY_DEGREE_TURN_LOWER_BOUND) {
                            bluetooth->printf("angle sensor condition triggered\r\n");
                            break;
                        }
                        //bluetooth->printf("%f angle diff\r\n", abs_subtract_angle(currentAngle, targetAngle));
                        wait_ms(50);
                    }

                    // update state variables after turn
                    if (currHeading == NORTH) currHeading = WEST;
                    else if (currHeading == EAST) currHeading = NORTH;
                    else if (currHeading == SOUTH) currHeading = EAST;
                    else if (currHeading == WEST) currHeading = SOUTH;
                    
                    robotState = FORWARD;
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



int main(void)
{
    // iRobot create requires 57600 baud
    device.baud(57600);

    // send start command to put iRobot in full mode
    irobotDriveSense::start(device);
    
    wait_ms(1000);
    led3 = 1;

    // set bluetooth baud rate
    bluetooth.baud(115200);
   
    // initialize magnetometer 
    compass.init();
    
    bluetooth.printf("setting up wifi shield\r\n");
    // setup wifi shield
    IntensityScanner rssi_scanner(D3, D5, D10, SPI(D11, D12, PTC5),
        "Joshi", "Rajashri1", mbed_cc3000::WPA, false, 200);
    bluetooth.printf("done setting up wifi shield\r\n");
    
    

    wait(2);
    
    int16_t data[3];
    
    /* calibrate the magnetometer by spinning 360 degrees
      and taking magnetometer readings

    The correct bias point will be (0.5 * (x_max + x_min),
        (0.5 * (z_max + z_min))
    */
    int16_t x_calibrate[256];
    int16_t z_calibrate[256];

    irobotDriveSense::setWheelSpeed(device, 0, 0);
    
    int array_index = 0;
    for (int j = 0; j < 4; ++j) {
        timer.stop();
        timer.reset();
        timer.start();
        irobotDriveSense::setWheelSpeed(device, 130, -130);
        
        while (timer.read_ms() < NINETY_DEGREE_TURN) {
            compass.getXYZ(data);
            *(x_calibrate + array_index) = data[0];
            *(z_calibrate + array_index) = data[2];
            ++array_index;
            wait_ms(40);
        }
        
        irobotDriveSense::setWheelSpeed(device, 0, 0);
        wait_ms(300);
    
    }
    
    
    int16_t x_min = 10000;
    int16_t x_max = -10000;
    int16_t z_min = 10000;
    int16_t z_max = -10000;
    for (int i = 0; i < array_index - 1; ++i) {
        if (x_calibrate[i] < x_min) x_min = x_calibrate[i];
        if (x_calibrate[i] > x_max) x_max = x_calibrate[i];
        if (z_calibrate[i] > z_max) z_max = z_calibrate[i];
        if (z_calibrate[i] < z_min) z_min = z_calibrate[i];
    }
    
    compass.setConstantOffset(-(x_min + x_max) / 2, 0, -(z_min + z_max) / 2);
    
    wait(2);
    
    /* Invoke the grid FSM */
    int32_t xDim = 3;
    int32_t yDim = 3;
    uint8_t* destinationArray = (uint8_t*) malloc (sizeof(uint8_t) * xDim * yDim);
    PossibleHeadings_t final_heading = robotControl (&device,
            &bluetooth,
            &compass, 
            &ultrasoundDistance, 
            &timer,
            &rssi_scanner,
            destinationArray,
            xDim,
            yDim);
            
    wait(5);

    /*
    Search for the coordinates of the point with maximal RSSI
    */
    uint8_t currMax = 0;
    int32_t xBest = -1;
    int32_t yBest = -1;
    for (int32_t i = 0; i < xDim; ++i) {
        for (int32_t j = 0; j < yDim; ++j) {
            if (*(destinationArray + i + yDim * j) > currMax) {
                currMax = *(destinationArray + i + yDim * j);
                xBest = i;
                yBest = j;
            }
        }        
    }
            
    free(destinationArray);

    /*
    Go to the point with the maximal RSSI
    */
    robotFindMax (&device,
            &bluetooth,
            &compass, 
            &ultrasoundDistance, 
            &timer,
            final_heading,
            xDim-1,
            yDim-1,
            xBest,
            yBest);

    
}

