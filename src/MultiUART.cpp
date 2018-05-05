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
#pragma GCC optimize ("O3")

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
volatile uint8_t MultiUART::bitSendClear = 0;
volatile uint8_t MultiUART::bitSendSkip = 0;
volatile uint8_t MultiUART::bitSendCount = 0;
volatile uint8_t MultiUART::baseClock = 0;

inline void MultiUART::interrupt_handle (void) {
    static volatile uint8_t portRxIn[4] = {0};
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

    // Transmitter
    if (MultiUART::bitSendCount) {
        if (!(MultiUART::bitSendCount % MultiUART::bitSendSkip)) {
            if (MultiUART::bitSendBuff & 1)
                *MultiUART::bitSendPort |= MultiUART::bitSendMask;
            else
                *MultiUART::bitSendPort &= MultiUART::bitSendClear;
            MultiUART::bitSendBuff >>= 1;
        }
        MultiUART::bitSendCount--;
    }

    // Reciver
    for (auto&& active : MultiUART::listeners) {
        if (!(active && active->portRxMask)) continue;
        uint8_t portin = portRxIn[active->portRx] & active->portRxMask;
        if (active->bitCount) {
            active->bitCount--;
            if (!active->bitCount) {
                active->buffAddr[active->buffIn++] = active->bitIn;
                active->buffIn &= active->buffMax;
                if (active->buffIn == active->buffOut) {
                    active->buffOver = true;
                    active->buffOut = (active->buffOut + 1) & active->buffMax;
                }
                if (!portin) {
                    active->bitCount = active->bitStart;
                    active->bitIn = 0;
                }
            }
            else if (!(active->bitCount % active->bitSkip)) {
                active->bitIn >>= 1;
                if (portin) active->bitIn |= 0x80;
            }
        }
        else if (!portin) {
            active->bitCount = active->bitStart;
            active->bitIn = 0;
        }
    }
}
#pragma GCC optimize ("Os")

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
        digitalWrite(_TX_PIN, HIGH);
        pinMode(_TX_PIN, OUTPUT);
    }
    if (_rx != NOT_A_PIN) {
        portRx = _rx - 1;
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
    bitStart = bitSkip * 9 + 1;

    uint8_t useListen = 0;
    for (auto&& active : MultiUART::listeners) {
        if (active != NULL) useListen++;
    }
    if (!useListen) {
        uint8_t oldSREG = SREG;
        noInterrupts();                         // := cli()
        #if defined(MULTIUART_USED_TIMER1)
        TCCR1A = 0;
        TCCR1B = _BV(WGM12) | _BV(CS10);
        OCR1AH = MULTIUART_CTC_TOP >> 8;
        OCR1AL = MULTIUART_CTC_TOP & 0xFF;
        TCNT1H = 0;
        TCNT1L = 0;
        TIMSK1 |= _BV(OCIE1A);
        #elif defined(MULTIUART_USED_TIMER2)
        TCCR2A = _BV(WGM21);
        TCCR2B = _BV(CS21);
        OCR2A = MULTIUART_CTC_TOP;
        TCNT2 = MultiUART::baseClock = 0;
        TIMSK2 |= _BV(OCIE2A);
        #endif
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
    if (portTxMask && bit_is_set(SREG, SREG_I)) {
        while (MultiUART::bitSendCount);
        MultiUART::bitSendPort = portTx;
        MultiUART::bitSendMask = portTxMask;
        MultiUART::bitSendClear = ~portTxMask;
        MultiUART::bitSendSkip = bitSkip;
        MultiUART::bitSendBuff = 0x200 | (c << 1);
        MultiUART::bitSendCount = bitStart + 1;
        while (MultiUART::bitSendCount);
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

int MultiUART::last (void) {
    if (buffIn == buffOut) return -1;           // empty rx buffer
    uint8_t c = buffAddr[buffIn];
    return c;
}

// end of code
