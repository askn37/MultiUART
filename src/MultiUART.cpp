/***************
 *
 * MultiUART - multiple UART device connector, substitute "SoftwareSerial" for Arduino
 *
 * target architectures: Atmel AVR (ATmega 328P, 1284P and other)
 *
 * release site: https://github.com/askn37/MultiUART
 * maintainer: askn <askn37@users.noreply.github.com>
 *
 */

#include <Arduino.h>
#include "MultiUART.h"

//
// Interrupt handling
//
#if defined(TIMER2_COMPA_vect)
ISR(TIMER2_COMPA_vect) {
    MultiUART::interrupt_handle();
}
#endif

volatile MultiUART* MultiUART::listeners[MULTIUART_RX_LISTEN_LEN] = {NULL};
volatile uint8_t MultiUART::baseClock = 0;
inline void MultiUART::interrupt_handle (void) {
    MultiUART::baseClock++;
    for (auto&& active : MultiUART::listeners) {
        if (!(active && active->portRxMask)) continue;
        uint8_t portin = *active->portRx & active->portRxMask;
        if (active->bitCount) {
            active->bitCount--;
            if (!active->bitCount && portin) {          // stopbit
                active->buffAddr[active->buffIn++] = active->bitIn;
                active->buffIn &= active->buffMax;
                if (active->buffIn == active->buffOut) {
                    active->buffOver = true;
                    active->buffOut = (active->buffOut + 1) & active->buffMax;
                }
            }
            else if (!(active->bitCount % active->bitSkip)) {
                active->bitIn >>= 1;
                if (portin) active->bitIn |= 0x80;      // insert MSB
            }
        }
        else if (!portin) {     // find startbit edge
            active->bitCount = active->bitStart;
            active->bitIn = 0;
        }
    }
}

//
// Constructor
//
MultiUART::MultiUART (uint8_t _RX_PIN, uint8_t _TX_PIN)
    : bitCount(0)
    , buffAddr(&buff[0])
    , buffIn(0)
    , buffOut(0)
    , buffMax(MULTIUART_RX_BUFF_LEN - 1)
    , buffOver(false)
    , portRxMask(0)
    , portTxMask(0)
    , writeBack(writeBackEmpty)
{
    uint8_t _rx = digitalPinToPort(_RX_PIN);
    uint8_t _tx = digitalPinToPort(_TX_PIN);
    if (_tx != NOT_A_PIN && _RX_PIN != _TX_PIN) {
        portTx = portOutputRegister(_tx);
        portTxMask = digitalPinToBitMask(_TX_PIN);
        portTxInvt = ~portTxMask;
        digitalWrite(_TX_PIN, HIGH);
        pinMode(_TX_PIN, OUTPUT);
    }
    if (_rx != NOT_A_PIN) {
        portRx = portInputRegister(_rx);
        portRxMask = digitalPinToBitMask(_RX_PIN);
        pinMode(_RX_PIN, INPUT);
        digitalWrite(_RX_PIN, HIGH);
    }
}

//
// Methods and Functions
//
bool MultiUART::begin (long _speed) {
    // TIMER2 CTC TOP clk/8
    bitSkip = MULTIUART_BASEFREQ / _speed;
    bitStart = bitSkip * 10 - 1;

    uint8_t useListen = 0;
    for (auto&& active : MultiUART::listeners) {
        if (active != NULL) useListen++;
    }
    if (!useListen) {
        uint8_t oldSREG = SREG;
        noInterrupts();                         // := cli()
        TCCR2A = _BV(WGM21);
        TCCR2B = _BV(CS21);
        TCNT2 = MultiUART::baseClock = 0;
        OCR2A = MULTIUART_CTC_TOP;
        TIMSK2 |= _BV(OCIE2A);
        SREG = oldSREG;
    }
    return listen();
}

bool MultiUART::listen (void) {
    if (isListening()) return true;
    if (portRxMask) {
        for (auto&& active : MultiUART::listeners) {
            if (active == NULL) {
                bitCount = 0;
                buffOver = false;
                active = this;
                return true;
            }
        }
    }
    return false;
}

bool MultiUART::isListening (void) {
    for (auto&& active : MultiUART::listeners) {
        if (active == this) return true;
    }
    return false;
}

bool MultiUART::stopListening (void) {
    for (auto&& active : MultiUART::listeners) {
        if (active == this) {
            active = NULL;
            return true;
        }
    }
    return false;
}

void MultiUART::setRxBuffer (volatile char* _buffAddr, int _buffMax) {
    uint8_t oldSREG = SREG;
    noInterrupts();                             // := cli()
    if (_buffAddr) {
        buffAddr = &_buffAddr[0];
        buffMax = (uint8_t)(_buffMax - 1);
    }
    else {
        buffAddr = &buff[0];
        buffMax = MULTIUART_RX_BUFF_LEN - 1;
    }
    buffIn = buffOut = bitCount = 0;
    buffOver = false;
    SREG = oldSREG;
}

size_t MultiUART::write (const uint8_t c) {
    if (bit_is_set(SREG, SREG_I) && bit_is_set(TIMSK2, OCIE2A) && portTxMask) {
        uint16_t d = 0x200 | (c << 1);          // LSB:startbit 10b:stopbit
        uint8_t b = MultiUART::baseClock + 1;
        for (uint8_t i = 0; i < 10; i++) {
            while (b != MultiUART::baseClock);
            if (d & 1) *portTx |= portTxMask;   // send LSB
            else *portTx &= portTxInvt;
            d >>= 1;                            // buffer shift
            b += bitSkip;                       // waiting next timing
        }
        b--;
        while (b != MultiUART::baseClock);      // waiting end timing
        writeBack(this);
    }
    return 1;
}

int MultiUART::read (void) {
    if (buffIn == buffOut) return -1;           // empty rx buffer
    uint8_t c = buffAddr[buffOut++];
    buffOut &= buffMax;
    return c;
}

int MultiUART::peek (void) {
    if (buffIn == buffOut) return -1;           // empty rx buffer
    uint8_t c = buffAddr[buffOut];
    return c;
}

// end of code
