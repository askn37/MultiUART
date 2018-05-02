/***************
 *
 * EchoBack - MultiUART echoback console sample sketch
 *
 * [Experimental] High speed 19200bps demo
 *
 * release site: https://github.com/askn37/MultiUART
 * maintainer: askn <askn37@users.noreply.github.com>
 *
 */

#define MULTIUART_BASEFREQ 38400	// *** experimental ***
#include <Arduino.h>
#include "MultiUART.h"

MultiUART UART(0, 1);

void setup (void) {
    UART.begin(MULTIUART_BASEFREQ / 2);
}

void loop (void) {
    if (UART.available()) UART.write(UART.read());
}

// end of code
