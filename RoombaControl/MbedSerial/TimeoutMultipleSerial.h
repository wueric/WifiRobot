#ifndef TIMEOUTMULTIPLESERIAL_H
#define TIMEOUTMULTIPLESERIAL_H

#include "mbed.h"
#include <stdint.h>

class TimeoutMultipleSerial {

    public:
        TimeoutMultipleSerial (PinName tx, PinName rx,
            const char* name, uint32_t tout) : port(tx, rx, name),
            timeout(tout), timer() { };
        
        int readMultChars (uint8_t* const dest, const size_t size);
        int writeMultChars (const uint8_t* source, const size_t size);
        int clearAll ();

        void setTimeout (uint32_t tout);

        void setBaud (uint32_t baud);
        void setFormat (int bits=8, SerialBase::Parity parity=SerialBase::None, int stop_bits=1);

    private:
        Timer timer;
        uint32_t timeout;
        Serial port;
}

#endif
