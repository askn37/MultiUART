/***************
 *
 * MultiDevices - MultiUART multiple devices sample sketch
 *
 * Four UART devices multiple receive demo
 *
 * release site: https://github.com/askn37/MultiUART
 * maintainer: askn <askn37@users.noreply.github.com>
 *
 */

// #define MULTIUART_BASEFREQ  19200
#include <Arduino.h>
#include "MultiUART.h"

MultiUART UART1(A0, A1);
MultiUART UART2(A2, A3);
MultiUART UART3(9, 10);
MultiUART UART4(11, 12);

void setup (void) {
    Serial.begin(9600);
    UART1.begin(9600);
    UART2.begin(9600);
    UART3.begin(9600);
    UART4.begin(9600);

    Serial.println("Four UART devices multiple receive demo");
}

void loop (void) {
    // console redirect, target UART4
    if (Serial.available()) {
        while (Serial.available()) {
            UART4.write(Serial.read());
        }
    }

    if (UART1.available()) {
        Serial.print("UART1:");
        while (UART1.available()) {
            Serial.write(UART1.read());
        }
        Serial.println();
    }

    if (UART2.available()) {
        Serial.print("UART2:");
        while (UART2.available()) {
            Serial.write(UART2.read());
        }
        Serial.println();
    }

    if (UART3.available()) {
        Serial.print("UART3:");
        while (UART3.available()) {
            Serial.write(UART3.read());
        }
        Serial.println();
    }

    if (UART4.available()) {
        Serial.print("UART4:");
        while (UART4.available()) {
            Serial.write(UART4.read());
        }
        Serial.println();
    }
}

// end of code
