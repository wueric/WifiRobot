/*


*/

#ifndef ULTRASOUND_ANALOG
#define ULTRASOUND_ANALOG

#include "mbed.h"

class UltrasoundAnalog {

    public:
        UltrasoundAnalog (PinName analogInputPin,
            double per_cm, double bias_cm) : _ultrasoundIn(analogInputPin),
            _per_cm(per_cm), _bias_cm(bias_cm) { };

        void recalibrateSetBias (double newBias);

        void recalibrateSetSlope (double slope_per_cm);

        /* returns distance in cm */
        double determineDistance ();
        
        uint16_t determineDistanceInt ();

    private:
        AnalogIn _ultrasoundIn;
        double _per_cm;
        double _bias_cm;

};

#endif

