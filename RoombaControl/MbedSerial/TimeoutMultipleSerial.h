/*
Implementation of a serial connection for mbed

Depends on Serial class provided by mbed sdk

Written 2014-11-10 by Eric Wu

This class writes/reads the specified number of symbols from the
    serial connection, and times out if it takes longer than
    a specified time to get the symbols
*/

#ifndef TIMEOUTMULTIPLESERIAL_H
#define TIMEOUTMULTIPLESERIAL_H

#include "mbed.h"
#include <stdint.h>

class TimeoutMultipleSerial {

    public:
        // constructor, @param tout is the timeout time in milliseconds
        TimeoutMultipleSerial (PinName tx, PinName rx,
            const char* name, uint32_t tout) : port(tx, rx, name),
            timeout(tout), timer() { };
        
        // reads @param size number of symbols from the serial connection
        //      into @param dest
        int readMultChars (uint8_t* const dest, const size_t size);

        // writes @param size number of symbols from @param source
        //      into the serial connection
        int writeMultChars (const uint8_t* source, const size_t size);

        // clears all available symbols from the serial connection
        int clearAll ();

        // sets the timeout in milliseconds to @param tout
        void setTimeout (uint32_t tout);

        // sets the baud to @param baud
        void setBaud (uint32_t baud);

        // sets the format for port
        void setFormat (int bits=8, SerialBase::Parity parity=SerialBase::None, int stop_bits=1);

    private:
        Serial port; // port
        uint32_t timeout; // timeout time in millisecondes
        Timer timer; // timer which tracks timeout time
}

#endif
