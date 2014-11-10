#include <mbed.h>
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

void irobotTestNavigate(
    int32_t* netDistance,
    int32_t* netAngle,
    irobotSensorGroup6_t* sensors,
    AccelMeasure* acc_meas,
    int32_t* leftWheelSpeed,
    int32_t* rightWheelSpeed) {

    *leftWheelSpeed = 100;
    *rightWheelSpeed = 100;

}
    
                        

static const float alpha_decay = 0.5; // accelerometer low pass filter negative
        // feedback coefficient

static const uint32_t timeout = 1000;

int main (int argc, char** argv) {

    MMA8451Q accelerometer(PTE25, PTE24, MMA8451_I2C_ADDRESS);

    TimeoutMultipleSerial port_as_serial = TimeoutMultipleSerial(PTE22, PTE23, 
        NULL, timeout);
    irobotUARTPort_t port = (irobotUARTPort_t) (&port_as_serial);



    Serial port_as_serial = Serial(PTE22, PTE23);
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
    DigitalOut myled(LED1);
    myled = 1;

    irobotSensorPollSensorGroup6(port, &sensors);

    while (!sensors.buttons.advance) {

        // update from sensors
        irobotSensorPollSensorGroup6(port, &sensors);

        // take accelerometer measurement
        accelMeasurements.updateAccelerometerRead();

        // apply low pass filter y[n] = a * x[n] - (1-a) * y[n-1]
        accelMeasurements = alpha_decay * accelMeasurements + 
            (1 - alpha_decay) * accelPrevMeasurements;

        // update variables
        accelPrevMeasurements = accelMeasurements;

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
