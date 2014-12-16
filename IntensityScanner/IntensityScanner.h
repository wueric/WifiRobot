/*
Class for the RSSI 802.11 signal intensity scanner. Built on and
    heavily dependent on original cc3000 driver code provided by Texas Instruments
    and ported by Martin Kojtal from mbed
    
Important: the core functionality of this class depends on a bug fix in the cc3000 driver
    code made by the group WiFinder for this project. When the endianness of data is being
    flipped in the cc3000 drivers, a casting issue was preventing the network scan from
    being completed. This bug fix is located in cc3000_event.cpp.
    
Implemented Eric Wu November 2014
*/

#include "mbed.h"
#include "cc3000.h"

class IntensityScanner {
    

    public:
    
        /*
        Constructor for IntensityScanner
        
        All of the parameters are taken from the cc3000 object constructor
            except
            
            @param uint32_t timeout - the timeout in milliseconds for the network
                scan. This parameter is passed to ioctl_set_scan_params() in the
                driver code
        */
        IntensityScanner (PinName cc3000_irq,
            PinName cc3000_en,
            PinName cc3000_cs,
            SPI cc3000_spi, 
            const char* SSID, 
            const char* key, 
            mbed_cc3000::Security sec, 
            bool smart_config, 
            uint32_t timeout);

        /*
        Method for getting the RSSI signal intensity for a given SSID
        Return 0 if successfully found a network with the given SSID, 1 in
            all other cases of failure
            
            @param const char* ssid - the SSID for the network being probed.
                Must be a null-terminated C-string
            @param uint8_t* rssi - pointer to where the RSSI reading will be
                stored.
        */
        int rssi_for_ssid (const char* ssid, uint8_t* rssi);

    private:
        mbed_cc3000::cc3000 wifi;
        uint32_t timeout;
};