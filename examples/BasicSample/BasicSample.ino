/***************
 *
 * BasicSample - MultiUART Basic sample sketch
 *
 * release site: https://github.com/askn37/MultiUART
 * maintainer: askn https://twitter.com/askn37
 *
 */

#include <Arduino.h>
#include "MultiUART.h"

#define RX_PIN		(11)		// D11
#define TX_PIN		(12)		// D12

MultiUART UART(RX_PIN, TX_PIN);

void setup (void) {
    Serial.begin(9600);
    UART.begin(9600);
}

void loop (void) {
    while (Serial.available()) {
        UART.write(Serial.read());
    }
    while (UART.available()) {
        Serial.write(UART.read());
    }
}

// end of code
