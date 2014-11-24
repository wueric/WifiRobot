#include "mbed.h"
#include "cc3000.h"

class IntensityScanner {
    public:
        IntensityScanner (PinName cc3000_irq,
            PinName cc3000_en,
            PinName cc3000_cs,
            SPI cc3000_spi, uint32_t timeout);

        int rssi_for_ssid (const char* ssid, uint8_t* rssi);

    private:
        cc3000::cc3000 wifi;
        uint32_t timeout;

};
