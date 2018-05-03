/***************
 *
 * LoRaWAN_Kiwi_ADB922S - LoRaWAN Shield simple test sample sketch
 *
 * release site: https://github.com/askn37/MultiUART
 * maintainer: askn https://twitter.com/askn37
 *
 */

// #define MULTIUART_BASEFREQ  19200
#include <Arduino.h>
#include "MultiUART.h"

#define RX_PIN		(11)		// D11
#define TX_PIN		(12)		// D12

MultiUART UART(RX_PIN, TX_PIN);

void setup (void) {
    Serial.begin(9600);
    UART.begin(9600);
    UART.setWriteBack(echoback);

    waiting();
    UART.print(F("\r"));
    waiting();
    UART.print(F("mod factory_reset\r"));
    waiting();
    UART.print(F("mod set_echo on\r"));
    waiting();
    UART.print(F("mod get_hw_deveui\r"));
    waiting();

    int cont = 0;
    while (cont < 2) {
        String result;
        UART.print(F("lorawan join otaa\r"));
        cont = 0;
        while (cont < 1) {
            while (cont < 1 && UART.available()) {
                char c = UART.read();
                result += c;
                if (result.indexOf("unsuccess") >= 0) cont = 1;
                else if (result.indexOf("accepted") >= 0) cont = 2;
            }
        }
        Serial.print(result);
    }

    UART.print(F("lorawan set_dr 5\r"));
    waiting();
}

void loop (void) {
    uint32_t now = millis();
    String data = String(now, DEC);
    if (data.length() & 1) data += 'f';

    UART.print(F("lorawan get_upcnt\r"));
    waiting();
    UART.print(F("lorawan get_downcnt\r"));
    waiting();

    UART.print(F("lorawan tx cnf 1 "));
    UART.print(data);
    UART.write('\r');
    for (int i = 0; i < 10; i++) waiting();
}

void echoback (MultiUART* UART) {
    while (UART->available()) {
        Serial.write(UART->read());
    }
}

void waiting (void) {
    delay(1000);
    echoback(&UART);
}

// end of code
