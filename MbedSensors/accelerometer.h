#include "mbed.h"
#include "MMA8451Q.h"

#ifndef MBED_ACCELEROMETER_H
#define MBED_ACCELEROMETER_H


#define MMA8451_I2C_ADDRESS (0x1d)

class AccelMeasure {
    private:
        static MMA8451Q accelerometer;
    public:
        float x_accel;
        float y_accel;
        float z_accel;

        AccelMeasure(float, float, float);
        void updateAccelerometerRead ();
};


AccelMeasure operator+ (AccelMeasure a, AccelMeasure b);

AccelMeasure operator* (float a, AccelMeasure b);

AccelMeasure operator* (AccelMeasure b, float a);

#endif
