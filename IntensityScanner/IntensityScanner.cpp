#include "IntensityScanner.h"

IntensityScanner::IntensityScanner (PinName cc3000_irq,
    PinName cc3000_en,
    PinName cc3000_cs,
    SPI cc3000_spi, 
    uint32_t timeout) : wifi(cc3000_irq, cc3000_en,
        cc3000_cs, cc3000_spi, SSID, key, sec, smart_config), timeout(timeout) {

    wifi.init();

    const unsigned long intervalTime[16] = { 200, 200, 200, 200,  200,
    200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200 };

    wifi._wlan.ioctl_set_scan_params(timeout, 20, 30, 5,
        0x400, -80, 0, 205, (uint32_t*) intervalTime);
        
};
        

int IntensityScanner::rssi_for_ssid (const char* ssid, uint8_t* rssi) {

    uint8_t scan_buffer[50];
    uint8_t rssi_field, ssid_length_field, ssid_length;
    char* measured_ssid_name;
    
    int found_match = 0;
    while (!found_match
        && wifi._wlan.ioctl_get_scan_results(1000, scan_buffer) == 0) {
            
        rssi_field = scan_buffer[8];
        ssid_length_field = scan_buffer[9];
        ssid_length = ssid_length_field & (0x3F);

        measured_ssid_name = (char*) malloc(sizeof(char) * (ssid_length+1));
        memcpy(measured_ssid_name, scan_buffer + 12, ssid_length);
        measured_ssid_name[ssid_length] = '\0';
        
        if (strcmp(ssid, measured_ssid_name) == 0) {
            
            *rssi = (rssi_field & 0x7f);
            found_match = 1;
        }

        free(measured_ssid_name);
    }

    return found_match ? 0 : 1;
}
