#include "mbed.h"
#include "cc3000.h"
#include "NVIC_set_all_priorities.h"


static const char* ssid = "It Hertz When IP";
static const char* phrase = "Rajashri1";

mbed_cc3000::cc3000 wifi(PTA12, PTA6, PTD0, SPI(PTD2, PTD3, PTD1), 
    ssid, phrase, mbed_cc3000::WPA2, false);
Serial mbed_board(PTD3, PTD2);

void init() {
    DigitalOut PWR_EN1(PTB2);
    DigitalOut PWR_EN2(PTB3);
 
    // Wi-Go set current to 500mA since we're turning on the Wi-Fi
    PWR_EN1 = 0;
    PWR_EN2 = 1;
 
    NVIC_set_all_irq_priorities(3);
    NVIC_SetPriority(SPI0_IRQn, 0x0);     // Wi-Fi SPI interrupt must be higher priority than SysTick
    NVIC_SetPriority(PORTA_IRQn, 0x1);
    NVIC_SetPriority(SysTick_IRQn, 0x2);  // SysTick set to lower priority than Wi-Fi SPI bus interrupt
    PORTA->PCR[16] |=PORT_PCR_ISF_MASK;
    PORTA->ISFR |= (1 << 16);
}
 

int main (int argc, char** argv) {
    init();
    mbd_board.baud(115200);
    printf("CC3000 ping demo. \r\n");
    wifi.init();

    if (wifi.connect() == -1) {
        printf("Failed to connect. Please verify connection details and try again. \r\n");
    } else {
        printf("IP address: %s \r\n",wifi.getIPAddress());
    }
    
    uint32_t ip;
    uint8_t *site = (uint8_t *)"google.com";
    printf("Get an IP address of %s \r\n",site);
    if (wifi._socket.gethostbyname(site,strlen((const char *)site), &ip)) {
        uint8_t add0 = (ip >> 24);
        uint8_t add1 = (ip >> 16);
        uint8_t add2 = (ip >> 8);
        uint8_t add3 = (ip >> 0);
        printf("IP address of %s: %d:%d:%d:%d \r\n", site, add0, add1, add2, add3);
    } else {
        printf("Failed. \r\n");
    }
 
    printf("Starting sending ping. \r\n");
    uint32_t reply_count = wifi.ping(ip, 5, 500, 32);
    printf("Received %d replies. \r\n", reply_count);
    printf("Ping demo completed. \r\n");
    wifi.disconnect();
    
    return 0;
}
