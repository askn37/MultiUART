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
#ifndef __MULTIUART_H
#define __MULTIUART_H

#ifndef MULTIUART_BASEFREQ
#define MULTIUART_BASEFREQ  19200
#endif

#ifndef MULTIUART_RX_BUFF_LEN
#define MULTIUART_RX_BUFF_LEN   32
#endif

#ifndef MULTIUART_RX_LISTEN_LEN
#define MULTIUART_RX_LISTEN_LEN   4
#endif

#define MULTIUART_CTC_TOP (F_CPU / MULTIUART_BASEFREQ / 8 - 1)
#if MULTIUART_CTC_TOP < 15
#error MULTIUART_BASEFREQ is under run, bad configuration from MultiUART
#endif

class MultiUART : public Stream {
private:
    volatile char buff[MULTIUART_RX_BUFF_LEN];
    volatile char* buffAddr;
    volatile uint8_t *portRx;
    volatile uint8_t *portTx;
    uint8_t portRxMask;
    uint8_t portTxMask;
    uint8_t portTxInvt;
    volatile uint8_t bitCount;
    uint8_t bitSkip;
    uint8_t bitStart;
    volatile uint8_t bitIn;
    uint8_t buffMax;
    volatile uint8_t buffIn;
    volatile uint8_t buffOut;
    uint8_t buffOver:1;

    typedef void (*MultiUART_CallBack)(MultiUART*);
    MultiUART_CallBack writeBack;
    static void writeBackEmpty (MultiUART*) {}

    static volatile MultiUART *listeners[MULTIUART_RX_LISTEN_LEN];
    static volatile uint8_t baseClock;

public:
    MultiUART (uint8_t, uint8_t);
    ~MultiUART (void) { stopListening(); }
    inline operator bool (void) { return true; }

    bool begin (long = 9600);
    bool listen (void);
    bool isListening (void);
    bool stopListening (void);
    void end (void) { stopListening(); }
    inline uint8_t getBaseClock (void) { return MultiUART::baseClock; }

    void setRxBuffer (volatile char* = NULL, int = 0);
    inline void setWriteBack (MultiUART_CallBack _callback = writeBackEmpty) { writeBack = _callback; }

    using Stream::read;
    virtual int read (void);
    virtual int peek (void);
    virtual inline void flush (void) {}
    virtual int available (void) { return ((uint8_t)(buffIn - buffOut) % buffMax); }
    bool overflow (void) { bool r = buffOver; if (r) buffOver = false; return r; }
    inline bool isFraming (void) { return bitCount; }

    using Print::write;
    virtual size_t write (const uint8_t);
    virtual inline int availableForWrite (void) { return 1; }

    static inline void interrupt_handle (void) __attribute__((__always_inline__));
};

#endif
