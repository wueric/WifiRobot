#include "TimeoutMultipleSerial.h"

int TimeoutMultipleSerial::readMultChars (uint8_t* const dest, 
    const size_t size) {

    timer.reset();
    timer.start();

    size_t counter = 0;
    while (counter < size 
        && timer.read_ms() < timeout) {

        while (!port.readable() 
            && timer.read_ms() < timeout) wait_ms(1);

        if (port.readable()) {
            *(dest + counter) = (uint8_t) (port.getc());
            ++counter;
        }
    }

    timer.stop();

    if (counter != size) {
        return -1;
    }

    return 1;
}

int TimeoutMultipleSerial::writeMultChars (const uint8_t* source,
    const size_t size) {

    timer.reset();
    timer.start();

    size_t counter = 0;
    while (counter < size
        && timer.read_ms() < timeout) {
        
        while (!port.writeable()
            && timer.read_ms() < timeout) wait_ms(1);

        if (port.writeable()) {
            port.putc((int) (*(source + counter)));
            ++counter;
        }
    }

    timer.stop();
    if (counter != size) {
        return -1;
    }

    return 1;
}

int TimeoutMultipleSerial::clearAll() {
    while (!port.readable()) port.getc();
    return 1;
}

void TimeoutMultipleSerial::setTimeout (uint32_t tout) {
    timeout = tout;
}

void TimeoutMultipleSerial::setBaud (uint32_t baud) {
    port.baud(baud);
}

void TimeoutMultipleSerial::setFormat (int bits, 
    SerialBase::Parity parity, 
    int stop_bits) {

    port.format(bits, parity, stop_bits);
}
