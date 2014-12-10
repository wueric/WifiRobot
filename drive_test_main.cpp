
#include "mbed.h"
#include "irobot_drive_sense.h"

#define abs(x)  (x<0)?-x:x

Serial device(PTA2, PTA1);

DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);
DigitalOut led4(LED4);


typedef enum a {
    FORWARD,
    TURN,
    STOP
} robotState_t;

void irobotTestNavigate(
    int16_t netDistance,
    int16_t netAngle,
    int16_t* leftWheelSpeed,
    int16_t* rightWheelSpeed) {
    	
    static robotState_t robotState = FORWARD;
    static int32_t angleAtManeuverStart = 0;
    static int32_t distanceAtManeuverStart = 0;
    
    if (robotState == FORWARD) {
    	led1 = 1;
    	led2 = 0;
    	
    	if (abs(netDistance - distanceAtManeuverStart) > 1000) {
    		angleAtManeuverStart = netAngle;
    		distanceAtManeuverStart = netDistance;
    		robotState = TURN;
    	}
    } else if (robotState == TURN) {
    	led2 = 1;
    	led1 = 0;
    	
    	if (abs(netAngle - angleAtManeuverStart) > 360) {
    		angleAtManeuverStart = netAngle;
    		distanceAtManeuverStart = netDistance;
    		robotState = STOP;
    	}
    	
    }
    
    switch (robotState) {
    	case FORWARD:
    		*leftWheelSpeed = 100;
    		*rightWheelSpeed = 100;
    		break;
    	case TURN:
    		*leftWheelSpeed = 100;
    		*rightWheelSpeed = -100;
    		break;
    	case STOP:
    		*leftWheelSpeed = 0;
    		*rightWheelSpeed = 0;
    	
    }

}

int main () {
	
	wait(5);

	led1 = 0;
	led2 = 0;
	led3 = 0;
	led4 = 0;
	
	device.baud(57600);
	
	led1 = 1;
	led2 = 0;

    int16_t netAngle = 0;
    int16_t netDistance = 0;
    int16_t rightWheelSpeed = 0;
    int16_t leftWheelSpeed = 0;

    int16_t angleDelta;
    int16_t distanceDelta;

    irobotDriveSense::start(device);
    
    led1 = 0;
	led2 = 0;
	led3 = 0;
	led4 = 0;
    
    for (int i = 0; i < 6; ++i) {
		led1 = i & 0x1;
		wait_ms(500);
	}
		
	irobotDriveSense::poll_netDistance(device, &distanceDelta);
    irobotDriveSense::poll_netAngle(device, &angleDelta);
    
    distanceDelta = 0;
    angleDelta = 0;
		
	led1 = 0;
	led2 = 0;
	led3 = 0;
	led4 = 0;
    
    while (1) {
        
        irobotDriveSense::poll_netAngle(device, &angleDelta);
        netAngle += angleDelta;

        irobotDriveSense::poll_netDistance(device, &distanceDelta);
        netDistance += distanceDelta;

        // execute statechart
        irobotTestNavigate(
        	netDistance,
            netAngle,
            &leftWheelSpeed,
            &rightWheelSpeed);
            
        irobotDriveSense::setWheelSpeed(device, rightWheelSpeed, leftWheelSpeed);
    }

}