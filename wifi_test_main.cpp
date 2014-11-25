#include "mbed.h"
#include "cc3000.h"
#include "IntensityScanner.h"
#include <MMA8451Q.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include "irobot.h"
#include "irobotSensorTypes.h"
#include "accelerometer.h"
#include "TimeoutMultipleSerial.h"

#define MMA8451_I2C_ADDRESS (0x1d)

typedef enum a {
    FORWARD,
    TURN
    
    } robotState_t;
    
static robotState_t robotState = FORWARD;

void irobotTestNavigate(
    int32_t* netDistance,
    int32_t* netAngle,
    irobotSensorGroup6_t* sensors,
    AccelMeasure* acc_meas,
    int32_t* leftWheelSpeed,
    int32_t* rightWheelSpeed) {

    /*
    switch (robotState) {
        case FORWARD:
            {
                
                if (*netDistance > 100) {
                    robotState = TURN;
                    *netDistance = 0;
                    *netAngle = 0;
                    }
                else {
                    *leftWheelSpeed = 50;
                    *rightWheelSpeed = 50;
                    }
                
            }
        case TURN:
            {
                if (*netAngle > 90) {
                        *netDistance = 0;
                        *netAngle = 0;
                        robotState = FORWARD;
                    }
                else {
                    *leftWheelSpeed = 50;
                    *rightWheelSpeed = -50;
                    }
            }
        }
        */
        *leftWheelSpeed = 100;
        *rightWheelSpeed = 100;
}

Serial pc(USBTX, USBRX);
DigitalOut led_red(LED_RED);
DigitalOut led_green(LED_GREEN);
/*
int main () {
    // by default, it's red
    printf("beginning\r\n");
    led_red = 1;
    led_green = 0;

    IntensityScanner rssi_scanner = IntensityScanner(PTD7, PTD6, D10, SPI(D11, D12, PTC5), 200);
        
    uint8_t rssi;
    while (1) {
        if (rssi_scanner.rssi_for_ssid("attwifi", &rssi) == 0)
            printf("Rssi is %d\r\n", rssi);
        wait_ms(100);
    }
}
*/

int main (int argc, char** argv) {

    MMA8451Q accelerometer(PTE25, PTE24, MMA8451_I2C_ADDRESS);

    TimeoutMultipleSerial port_as_serial = TimeoutMultipleSerial(PTC4, PTC3, 
        NULL, 100);
    irobotUARTPort_t port = (irobotUARTPort_t) (&port_as_serial);

    int32_t irobotStatus;
    
    irobotSensorGroup6_t sensors;
    int32_t netDistance = 0;
    int32_t netAngle = 0;

    int32_t rightWheelSpeed = 0;
    int32_t leftWheelSpeed = 0;

    AccelMeasure accelMeasurements(0.0f, 0.0f, 0.0f);
    AccelMeasure accelPrevMeasurements(0.0f, 0.0f, 0.0f);

    int32_t status = irobotOpen(port);

    
    /*
    For debugging only
    */
    DigitalOut myled(LED2);

    irobotSensorPollSensorGroup6(port, &sensors);
    
    while (!sensors.buttons.advance || true) {
        myled = 1;

        // update from sensors
        irobotSensorPollSensorGroup6(port, &sensors);



        irobotTestNavigate(&netDistance,
            &netAngle,
            &sensors,
            &accelMeasurements,
            &leftWheelSpeed,
            &rightWheelSpeed);
       
        irobotDriveDirect(port, leftWheelSpeed, rightWheelSpeed); 
    }

    status = irobotClose(port);
    return status;
}
