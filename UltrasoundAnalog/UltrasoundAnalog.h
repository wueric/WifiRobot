/*
Wrapper for the AnalogIn class / pin that is used for the MaxBotix ultrasound sensor

Reads in an analog voltage, and returns both the raw reading and a value determined using
    the affine model of the sensor

Written by Eric Wu November 2014

*/

#ifndef ULTRASOUND_ANALOG
#define ULTRASOUND_ANALOG

#include "mbed.h"

class UltrasoundAnalog {

    public:
        /*
            Constructor for UltrasoundAnalog

            @param PinName analogInputPin - the name of the pin that the sensor
                is connected to
            @param double per_cm - the slope for the affine model
                Returned distance reading is calculated using equation
                bias_cm + (Sensor reading * per_cm)
            @param double bias_cm - the bias for the affine model
                Returned distance reading is calculated using equation
                bias_cm + (Sensor reading * per_cm)
        */
        UltrasoundAnalog (PinName analogInputPin,
            double per_cm, double bias_cm) : _ultrasoundIn(analogInputPin),
            _per_cm(per_cm), _bias_cm(bias_cm) { };

        /*
            Method for changing the bias in the affine model

            @param double newBias - the bias for the affine model
                Returned distance reading is calculated using equation
                bias_cm + (Sensor reading * per_cm)
        */
        void recalibrateSetBias (double newBias);


        /*
            Method for changing the slope in the affine model

            @param double slope_per_cm - the bias for the affine model
                Returned distance reading is calculated using equation
                bias_cm + (Sensor reading * per_cm)
        */
        void recalibrateSetSlope (double slope_per_cm);

        /*
            Method for reading the sensor and returning the distance
                in cm

            Return value - double, corresponding to the distance in cm

            This value is calculated using the affine model
                _bias_cm + ADCdata * _per_cm;
        */
        double determineDistance ();

       
        /*
            Method for reading the raw integer value returned by the 16 bit ADC
                on the board

            Retrun value - unsigned 16 bit integer
        */ 
        uint16_t determineDistanceInt ();

    private:
        AnalogIn _ultrasoundIn;
        double _per_cm;
        double _bias_cm;

};

#endif

