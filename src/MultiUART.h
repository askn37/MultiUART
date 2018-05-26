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
#ifndef __MULTIUART_H
#define __MULTIUART_H

#ifndef MULTIUART_BASEFREQ
#define MULTIUART_BASEFREQ 19200
#endif

#ifndef MULTIUART_BASEFREQ_ARIGN
#define MULTIUART_BASEFREQ_ARIGN 0
#endif

#ifndef MULTIUART_RX_BUFF_LEN
#define MULTIUART_RX_BUFF_LEN 64
#endif

#ifndef MULTIUART_RX_LISTEN_LEN
#define MULTIUART_RX_LISTEN_LEN 4
#endif

//// using pinOut A2
// #define MULTIUART_DEBUG_PULSE

#if defined(MULTIUART_USED_TIMER2)
#define MULTIUART_CTC_TOP (F_CPU / MULTIUART_BASEFREQ / 8 - MULTIUART_BASEFREQ_ARIGN)
#else
#ifndef MULTIUART_USED_TIMER1
#define MULTIUART_USED_TIMER1
#endif
#define MULTIUART_CTC_TOP (F_CPU / MULTIUART_BASEFREQ - MULTIUART_BASEFREQ_ARIGN)
#endif

#if MULTIUART_CTC_TOP < 250
#error MULTIUART_BASEFREQ is under run, bad configuration from MultiUART
#endif

class MultiUART : public Stream {
private:
    volatile uint8_t *portTxReg;
    volatile uint8_t *portRxReg;
    volatile uint8_t portRx;
    volatile uint8_t portRxMask;
    volatile uint8_t bitCount;
    volatile uint8_t bitSkip;
    volatile uint8_t bitWait;
    volatile uint8_t bitStart;
    volatile uint8_t bitIn;
    volatile uint8_t buffMax;
    volatile uint8_t buffIn;
    volatile uint8_t buffOut;
    volatile uint8_t buffOver:1;

    volatile char buff[MULTIUART_RX_BUFF_LEN];
    volatile char* buffAddr;

    static volatile uint16_t throttle;
    static volatile uint16_t bitSendBuff;
    static volatile uint8_t* bitSendPort;
    static volatile uint8_t bitSendMask;
    static volatile uint8_t bitSendSkip;
    static volatile uint8_t bitSendWait;
    static volatile uint8_t bitSendCount;
    static volatile uint8_t baseClock;

    static volatile MultiUART *listeners[MULTIUART_RX_LISTEN_LEN];

    uint8_t portTxMask;

    typedef void (*MultiUART_CallBack)(MultiUART*);
    MultiUART_CallBack writeBack;
    static void writeBackEmpty (MultiUART*) {}

    HardwareSerial *hSerial;
    void hSerialReader (void);

public:
    MultiUART (uint8_t, uint8_t);
    MultiUART (HardwareSerial&);
    ~MultiUART (void) { stopListening(); }
    inline operator bool (void) { return true; }

    virtual bool begin (long = 9600, uint8_t = SERIAL_8N1);
    virtual bool listen (void);
    virtual bool isListening (void);
    virtual bool stopListening (void);
    virtual void stopListener (void);
    void end (void) { if (hSerial) hSerial->end(); else stopListening(); }
    inline uint8_t getBaseClock (void) { return MultiUART::baseClock; }

    void setRxBuffer (volatile char* = NULL, int = 0);
    inline void setWriteBack (MultiUART_CallBack _callback = writeBackEmpty) { writeBack = _callback; }

    using Stream::read;
    virtual int read (void);
    virtual int peek (void);
    virtual int last (void);
    virtual inline void flush (void) { if (hSerial) hSerial->flush(); }
    virtual int available (void);
    bool overflow (void) { bool r = buffOver; if (r) buffOver = false; return r; }
    inline bool isFraming (void) { return bitCount; }

    using Print::write;
    virtual size_t write (const uint8_t);
    virtual int availableForWrite (void);

    static void setThrottle (int16_t = 0);
    static inline void interrupt_handle (void) __attribute__((__always_inline__));
};

#endif

// end of header
