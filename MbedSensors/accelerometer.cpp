#include "accelerometer.h"

MMA8451Q AccelMeasure::accelerometer = MMA8451Q(PTE25, PTE24, MMA8451_I2C_ADDRESS);

AccelMeasure::AccelMeasure(float x, float y, float z) : x_accel(x), y_accel(y), z_accel(z) {
}

void AccelMeasure::updateAccelerometerRead () {
    x_accel = accelerometer.getAccX();
    y_accel = accelerometer.getAccY();
    z_accel = accelerometer.getAccZ();
}

AccelMeasure operator+ (AccelMeasure a, AccelMeasure b) {
    return AccelMeasure(a.x_accel + b.x_accel, a.y_accel + b.y_accel,
        a.z_accel + b.z_accel);
}

AccelMeasure operator* (float a, AccelMeasure b) {
    return AccelMeasure(a * b.x_accel, a * b.y_accel, a * b.z_accel);
}


AccelMeasure operator* (AccelMeasure b, float a) {
    return AccelMeasure(a * b.x_accel, a * b.y_accel, a * b.z_accel);
}
