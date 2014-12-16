#include "UltrasoundAnalog.h"

void UltrasoundAnalog::recalibrateSetBias (double newBias) {
    _per_cm = newBias;
}

void UltrasoundAnalog::recalibrateSetSlope (double slope_per_cm) {
    _per_cm = slope_per_cm;
}

uint16_t UltrasoundAnalog::determineDistanceInt () {
    return _ultrasoundIn.read_u16();    
}

double UltrasoundAnalog::determineDistance () {
    uint16_t ADCdata = _ultrasoundIn.read_u16();
    return _bias_cm + ADCdata * _per_cm;
}

