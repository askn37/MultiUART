/***************
 *
 * MultiUART - multiple UART device connector, substitute "SoftwareSerial" for Arduino
 *
 * target architectures: Atmel AVR (ATmega 328P, 1284P and other)
 *
 * release site: https://github.com/askn37/MultiUART
 * maintainer: askn https://twitter.com/askn37
 *
 */

#include <Arduino.h>
#include "MultiUART.h"

//
// Interrupt handling
//
// #pragma GCC optimize ("O3")

#if defined(MULTIUART_USED_TIMER1)
#if defined(TIMER1_COMPA_vect)
ISR(TIMER1_COMPA_vect) {
    MultiUART::interrupt_handle();
}
#endif
#elif defined(MULTIUART_USED_TIMER2)
#if defined(TIMER2_COMPA_vect)
ISR(TIMER2_COMPA_vect) {
    MultiUART::interrupt_handle();
}
#endif
#endif

volatile MultiUART* MultiUART::listeners[MULTIUART_RX_LISTEN_LEN] = {NULL};
volatile uint16_t MultiUART::bitSendBuff = 0;
volatile uint8_t* MultiUART::bitSendPort;
volatile uint8_t MultiUART::bitSendMask = 0;
volatile uint8_t MultiUART::bitSendSkip = 0;
volatile uint8_t MultiUART::bitSendWait = 0;
volatile uint8_t MultiUART::bitSendCount = 0;
volatile uint8_t MultiUART::baseClock = 0;

inline void MultiUART::interrupt_handle (void) {
    uint8_t portRxIn[4];
    uint8_t portin;
    MultiUART::baseClock++;

    // Recive Registrers
    #if defined(PINA)
    portRxIn[0] = PINA;
    #endif
    #if defined(PINB)
    portRxIn[1] = PINB;
    #endif
    #if defined(PINC)
    portRxIn[2] = PINC;
    #endif
    #if defined(PIND)
    portRxIn[3] = PIND;
    #endif

    #ifdef MULTIUART_DEBUG_PULSE
    // DEBUG PULSE A2 Pin
    PINC |= _BV(2);
    #endif

    // Transmitter
    if (MultiUART::bitSendCount) {
        if (MultiUART::bitSendCount == MultiUART::bitSendSkip) {
            if (MultiUART::bitSendBuff & 1)
                *MultiUART::bitSendPort |= MultiUART::bitSendMask;
            else
                *MultiUART::bitSendPort &= ~MultiUART::bitSendMask;
            MultiUART::bitSendBuff >>= 1;
            MultiUART::bitSendSkip -= MultiUART::bitSendWait;
        }
        MultiUART::bitSendCount--;
    }

    // Reciver
    for (auto&& active : MultiUART::listeners) {
        if (!active) continue;
        portin = portRxIn[active->portRx] & active->portRxMask;
        if (active->bitCount) {
            active->bitCount--;
            if (active->bitCount == 0) {
                active->buffAddr[active->buffIn++] = active->bitIn;
                active->buffIn &= active->buffMax;
                if (active->buffIn == active->buffOut) {
                    active->buffOver = true;
                    active->buffOut = (active->buffOut + 1) & active->buffMax;
                }
                // if (!portin) {
                //     active->bitSkip = active->bitStart;
                //     active->bitCount = active->bitStart + (active->bitWait >> 1);
                //     active->bitIn = 0;
                // }
            }
            else if (active->bitCount == active->bitSkip) {
                active->bitIn >>= 1;
                if (portin) active->bitIn |= 0x80;
                active->bitSkip -= active->bitWait;
            }
        }
        else if (!portin) {
            active->bitSkip = active->bitStart;
            active->bitCount = active->bitStart + (active->bitWait >> 1);
            active->bitIn = 0;
        }
    }
}
// #pragma GCC optimize ("Os")

void MultiUART::setThrottle (int16_t _throttle) {
    uint16_t freqClock = MULTIUART_CTC_TOP - 1 - _throttle;
    uint8_t oldSREG = SREG;
    noInterrupts();
    #ifdef MULTIUART_DEBUG_PULSE
    pinMode(A2, OUTPUT);
    #endif
    #if defined(MULTIUART_USED_TIMER1)
    // TIMER1 clk/1
    TCCR1A = 0;
    TCCR1B = _BV(WGM12) | _BV(CS10);
    OCR1AH = (uint8_t)((freqClock >> 8) & 0xFF);
    OCR1AL = (uint8_t)(freqClock & 0xFF);
    TCNT1H = 0;
    TCNT1L = 0;
    TIMSK1 |= _BV(OCIE1A);
    #elif defined(MULTIUART_USED_TIMER2)
    // TIMER2 clk/8
    TCCR2A = _BV(WGM21);
    TCCR2B = _BV(CS21);
    OCR2A = (uint8_t) MultiUART::freqClock;
    TCNT2 = 0;
    TIMSK2 |= _BV(OCIE2A);
    #endif
    MultiUART::bitSendMask = 1;
    SREG = oldSREG;
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
    , hSerial(NULL)
{
    uint8_t _rx = digitalPinToPort(_RX_PIN);
    uint8_t _tx = digitalPinToPort(_TX_PIN);
    if (_tx != NOT_A_PIN && _RX_PIN != _TX_PIN) {
        portTxReg = portOutputRegister(_tx);
        portTxMask = digitalPinToBitMask(_TX_PIN);
        digitalWrite(_TX_PIN, HIGH);
        pinMode(_TX_PIN, OUTPUT);
    }
    if (_rx != NOT_A_PIN) {
        portRx = _rx - 1;
        portRxReg = portInputRegister(_rx);
        portRxMask = digitalPinToBitMask(_RX_PIN);
        pinMode(_RX_PIN, INPUT);
        digitalWrite(_RX_PIN, HIGH);
    }
}

MultiUART::MultiUART (HardwareSerial& _SERIAL)
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
    hSerial = &_SERIAL;
}

//
// Methods and Functions
//
bool MultiUART::begin (long _speed, uint8_t _config) {
    if (hSerial) {
        hSerial->begin(_speed, _config);
        return true;
    }
    bitWait = MULTIUART_BASEFREQ / _speed;
    bitStart = bitWait * 9;
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
                if (!MultiUART::bitSendMask) MultiUART::setThrottle(0);
                #if defined(MULTIUART_USED_TIMER1)
                TIMSK1 |= _BV(OCIE1A);
                #elif defined(MULTIUART_USED_TIMER2)
                TIMSK2 |= _BV(OCIE2A);
                #endif
                return true;
            }
        }
    }
    return false;
}

bool MultiUART::isListening (void) {
    if (hSerial) return true;
    for (auto&& active : MultiUART::listeners) {
        if (active == this) return true;
    }
    return false;
}

bool MultiUART::stopListening (void) {
    if (!hSerial) {
        for (auto&& active : MultiUART::listeners) {
            if (active == this) {
                active = NULL;
                return true;
            }
        }
    }
    return false;
}

void MultiUART::stopListener (void) {
    #if defined(MULTIUART_USED_TIMER1)
    TIMSK1 &= ~_BV(OCIE1A);
    #elif defined(MULTIUART_USED_TIMER2)
    TIMSK2 &= ~_BV(OCIE2A);
    #endif
    for (auto&& active : MultiUART::listeners) active = NULL;
    MultiUART::bitSendMask = 0;
}

void MultiUART::setRxBuffer (volatile char* _buffAddr, int _buffMax) {
    uint8_t oldSREG = SREG;
    noInterrupts();
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
    if (hSerial) {
        if (hSerial->write(c)) writeBack(this);
        else return 0;
    }
    else if (portTxMask && bit_is_set(SREG, SREG_I) &&
        #if defined(MULTIUART_USED_TIMER1)
        bit_is_set(TIMSK1, OCIE1A)
        #elif defined(MULTIUART_USED_TIMER2)
        bit_is_set(TIMSK2, OCIE2A)
        #endif
    ) {
        while (MultiUART::bitSendCount);
        MultiUART::bitSendPort = portTxReg;
        MultiUART::bitSendMask = portTxMask;
        MultiUART::bitSendBuff = 0xFE00 | (c << 1);
        MultiUART::bitSendWait = bitWait;
        MultiUART::bitSendCount = bitWait * 10;
        MultiUART::bitSendSkip = MultiUART::bitSendCount;
        while (MultiUART::bitSendCount);
        writeBack(this);
    }
    return 1;
}

int MultiUART::read (void) {
    if (hSerial) hSerialReader();
    if (buffIn == buffOut) return -1;
    uint8_t c = buffAddr[buffOut++];
    buffOut &= buffMax;
    return c;
}

int MultiUART::peek (void) {
    if (hSerial) hSerialReader();
    if (buffIn == buffOut) return -1;
    uint8_t c = buffAddr[buffOut];
    return c;
}

int MultiUART::last (void) {
    if (hSerial) hSerialReader();
    if (buffIn == buffOut) return -1;
    uint8_t c = buffAddr[(buffIn - 1) & buffMax];
    return c;
}

int MultiUART::available (void) {
    if (hSerial) hSerialReader();
    return ((uint8_t)(buffIn - buffOut) & buffMax);
}

int MultiUART::availableForWrite (void) {
    if (hSerial) return hSerial->availableForWrite();
    return 1;
}

void MultiUART::hSerialReader (void) {
    while (((buffIn - buffOut) & buffMax) < buffMax && hSerial->available()) {
        buffAddr[buffIn++] = (uint8_t) hSerial->read();
        buffIn &= buffMax;
    }
}

// end of code
