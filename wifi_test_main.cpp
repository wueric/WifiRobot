/**
 *  \brief CS294-84 demo \author Ben Zhang, Antonio Iannopollo
 *
 * This sampel code illustrates how to connect the mbed KL25Z platform to internet
 * thorugh the CC3000 wifi breakout board (http://www.adafruit.com/product/1469).
 * Connections between the KL25Z and the CC3000 are made according to the
 * guide at https://learn.adafruit.com/adafruit-cc3000-wifi -- KL25Z and arduino
 * UNO are pin to pin compatible --
 *
 * This application uses the following libraries:
 * - cc3000_hostdriver_mbedsocket
 *   (http://developer.mbed.org/users/Kojto/code/cc3000_hostdriver_mbedsocket/)
 * - HTTPClient (http://developer.mbed.org/users/donatien/code/HTTPClient/)
 */

#include "mbed.h"
#include "cc3000.h"
#include "IntensityScanner.h"

const char SSID[] = "It Hertz When IP";
const char key[] = "Rajashri1";

// KL25Z wifi connection
// we need to define connection pins for:
// - IRQ      => (pin D3)
// - Enable   => (pin D5)
// - SPI CS   => (pin D10)
// - SPI MOSI => (pin D11)
// - SPI MISO => (pin D12)
// - SPI CLK  => (pin D13)
// plus wifi network SSID, password, security level and smart-configuration flag.

Serial pc(USBTX, USBRX);
DigitalOut led_red(LED_RED);
DigitalOut led_green(LED_GREEN);

int main () {
    // by default, it's red
    printf("beginning\n");
    led_red = 1;
    led_green = 0;

    IntensityScanner rssi_scanner = IntensityScanner(PTD7, PTD6, D10, SPI(D11, D12, PTC5), 500, SSID, key, WPA2, false);
    uint8_t rssi;
    while (1) {
        if (rssi_scanner.rssi_for_ssid("It Hertz When IP", &rssi) == 0)
            pc.printf("Rssi is %d\n\n", rssi);

        wait(500);
    }
}
