#include "irobotUART.h"

/// Convert a baud code into its actual rate
/// \return error code
static int32_t irobotUARTBaudCodeToRate(
	const irobotBaud_t	baud,		///< [in]	baud code
	int* const	rate		///< [out]	baud rate
){
	if(!rate){
		return ERROR_INVALID_PARAMETER;
	}
	else{
		switch(baud){
		case IROBOT_BAUD_300:	*rate = 300;	break;
		case IROBOT_BAUD_600:	*rate = 600;	break;
		case IROBOT_BAUD_1200:	*rate = 1200;	break;
		case IROBOT_BAUD_2400:	*rate = 2400;	break;
		case IROBOT_BAUD_4800:	*rate = 4800;	break;
		case IROBOT_BAUD_9600:	*rate = 9600;	break;
		case IROBOT_BAUD_14400:	*rate = 14400;	break;
		case IROBOT_BAUD_19200:	*rate = 19200;	break;
		case IROBOT_BAUD_28800:	*rate = 28800;	break;
		case IROBOT_BAUD_38400:	*rate = 38400;	break;
		case IROBOT_BAUD_57600:	*rate = 57600;	break;
		case IROBOT_BAUD_115200:*rate = 115200;	break;	// WARNING: UNSTABLE
		default:
			*rate = 0;
			return ERROR_INVALID_PARAMETER;
			break;
		}

		return ERROR_SUCCESS;
	}
}

int32_t irobotUARTOpen(
    const irobotUARTPort_t port,
    const irobotBaud_t baud) {

    int32_t status = ERROR_SUCCESS;
    int baudRate = 0;

    irobot_StatusMerge(&status, irobotUARTBaudCodeToRate(baud, &baudRate));

    if (!irobot_IsError(status)) {
        port->baud(baudRate);
        port->format(8, Serial::None, 1);
    }

    return status;
}

int32_t irobotUARTClose(const irobotUARTPort_t port) {
    return ERROR_SUCCESS;
}

int32_t irobotUARTRead(
    const irobotUARTPort_t port,
    xqueue_t* const queue,
    size_t nData) {

    // catch NULL pointers
    int32_t status = ERROR_SUCCESS;
    if (!queue) {
        irobot_StatusMerge(&status, ERROR_INVALID_PARAMETER);
    }

    // read
    while (!irobot_IsError(status) && nData--) {
        uint8_t rxByte = 0;
        irobot_StatusMerge(&status, irobotUARTReadRaw(port, &rxByte, 1));
        if (!irobot_IsError(status)) {
            xqueue_push8(queue, rxByte);
        }
    }

    return status;
}

int32_t irobotUARTReadRaw(
    const irobotUARTPort_t port,
    uint8_t* const data,
    const size_t nData) {

    if (!data) {
        return ERROR_INVALID_PARAMETER;
    } else {
        // will wait approximately READ_WRITE_GIVEUP_TIME seconds in total for input, and then
        //  will give up if that time is exceeded
        Timer timer;
        for (size_t i = 0; i < nData; ++i) {
            if (port->readable()) {
                *(data+i) = (uint8_t) port->getc();
            } else {
                timer.start();
                while (!port->readable() 
                    && timer.read_ms() < READ_WRITE_GIVEUP_TIME) wait_ms(1);
                timer.stop();

                if (timer.read_ms() >= READ_WRITE_GIVEUP_TIME) {
                    return ERROR_TIMEOUT;
                }
            }
        }

        return ERROR_SUCCESS;
    }
}

int32_t irobotUARTWrite(
    const irobotUARTPort_t port,
    xqueue_t* const queue) {

	int32_t status = ERROR_SUCCESS;

	// catch NULL pointers
	if(!queue){
		irobot_StatusMerge(&status, ERROR_INVALID_PARAMETER);
	}
	
	// write
	while(!irobot_IsError(status) && !xqueue_empty(queue)){
		uint8_t txByte = xqueue_front(queue);
		irobot_StatusMerge(&status, irobotUARTWriteRaw(port, &txByte, 1));
		if(!irobot_IsError(status)){
			xqueue_drop(queue);
		}
	}

	return status;

}

int32_t irobotUARTWriteRaw(
    const irobotUARTPort_t port,
    const uint8_t* const data,
    const size_t nData) {

    if (!data) {
        return ERROR_INVALID_PARAMETER;
    } else {

        // will wait approximately READ_WRITE_GIVEUP_TIME seconds in total until space is
        // available, and then will give up if that time is exceeded
        Timer timer;
        for (size_t i = 0; i < nData; ++i) {
            if (port->writeable()) {
                port->putc((int) (*(data+i)));
            } else {
                timer.start();
                while (!port->writeable() 
                    && timer.read_ms() < READ_WRITE_GIVEUP_TIME) wait_ms(1);
                timer.stop();

                if (timer.read_ms() >= READ_WRITE_GIVEUP_TIME) {
                    return ERROR_TIMEOUT;
                }
            }
        }
        return ERROR_SUCCESS;
    }
}

int32_t irobotUARTClear(
    const irobotUARTPort_t port) {
    while (port->readable()) port->getc();
    return ERROR_SUCCESS;
}
