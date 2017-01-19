#ifndef AFSK_H
#define AFSK_H

#include "device.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include "util/FIFO.h"
#include "util/time.h"
#include "protocol/HDLC.h"

#define SIN_LEN 512
static const uint8_t sin_table[] PROGMEM = {
    // These values have been truncated to the upper 4 bits of PORTD
    128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 128, 144, 144, 144, 144, 144,
    144, 144, 144, 144, 144, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160, 160,
    176, 176, 176, 176, 176, 176, 176, 176, 176, 176, 176, 192, 192, 192, 192, 192,
    192, 192, 192, 192, 192, 192, 192, 192, 208, 208, 208, 208, 208, 208, 208, 208,
    208, 208, 208, 208, 208, 208, 224, 224, 224, 224, 224, 224, 224, 224, 224, 224,
    224, 224, 224, 224, 224, 224, 224, 240, 240, 240, 240, 240, 240, 240, 240, 240,
    240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240,
    240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240, 240
};

inline static uint8_t sinSample(uint16_t i) {
    uint16_t newI = i % (SIN_LEN/2);
    newI = (newI >= (SIN_LEN/4)) ? (SIN_LEN/2 - newI -1) : newI;
    uint8_t sine = pgm_read_byte(&sin_table[newI]);
    return (i >= (SIN_LEN/2)) ? (256 - sine) : sine;
}


#define SWITCH_TONE(inc)  (((inc) == MARK_INC) ? SPACE_INC : MARK_INC)
#define BITS_DIFFER(bits1, bits2) (((bits1)^(bits2)) & 0x01)
#define DUAL_XOR(bits1, bits2) ((((bits1)^(bits2)) & 0x03) == 0x03)
#define SIGNAL_TRANSITIONED(bits) DUAL_XOR((bits), (bits) >> 2)
#define TRANSITION_FOUND(bits) BITS_DIFFER((bits), (bits) >> 1)

#define CPU_FREQ F_CPU

#define CONFIG_AFSK_RX_BUFLEN 64
#define CONFIG_AFSK_TX_BUFLEN 64   
#define CONFIG_AFSK_RXTIMEOUT 0
#define CONFIG_AFSK_PREAMBLE_LEN 150UL
#define CONFIG_AFSK_TRAILER_LEN 50UL
#define BIT_STUFF_LEN 5

#define SAMPLERATE 9600
#define BITRATE    1200

#define SAMPLESPERBIT (SAMPLERATE / BITRATE)
#define PHASE_INC    1                              // Nudge by an eigth of a sample each adjustment

#if BITRATE == 300
    #define FILTER_CUTOFF 600
    #define MARK_FREQ  1600
    #define SPACE_FREQ 1800
    #define PHASE_BITS   10                         // How much to increment phase counter each sample
#elif BITRATE == 960
    #define FILTER_CUTOFF 600
    #define MARK_FREQ  960
    #define SPACE_FREQ 1600
    #define PHASE_BITS   10
#elif BITRATE == 1200
    #define FILTER_CUTOFF 600
    #define MARK_FREQ  1200
    #define SPACE_FREQ 2200
    #define PHASE_BITS   8
#elif BITRATE == 1600
    #define FILTER_CUTOFF 800
    #define MARK_FREQ  1600
    #define SPACE_FREQ 2600
    #define PHASE_BITS   8
#elif BITRATE == 2400
    #define FILTER_CUTOFF 1600
    #define MARK_FREQ  2400
    #define SPACE_FREQ 4200
    #define PHASE_BITS   4
#else
    #error Unsupported bitrate!
#endif

#define PHASE_MAX    (SAMPLESPERBIT * PHASE_BITS)   // Resolution of our phase counter = 64
#define PHASE_THRESHOLD  (PHASE_MAX / 2)            // Target transition point of our phase window

typedef struct Hdlc
{
    uint8_t demodulatedBits;
    uint8_t bitIndex;
    uint8_t currentByte;
    bool receiving;
} Hdlc;

typedef struct Afsk
{
    // Stream access to modem
    FILE fd;

    // General values
    Hdlc hdlc;                              // We need a link control structure
    uint16_t preambleLength;                // Length of sync preamble
    uint16_t tailLength;                    // Length of transmission tail

    // Modulation values
    uint8_t sampleIndex;                    // Current sample index for outgoing bit 
    uint8_t currentOutputByte;              // Current byte to be modulated
    uint8_t txBit;                          // Mask of current modulated bit
    bool bitStuff;                          // Whether bitstuffing is allowed

    uint8_t bitstuffCount;                  // Counter for bit-stuffing

    uint16_t phaseAcc;                      // Phase accumulator
    uint16_t phaseInc;                      // Phase increment per sample

    FIFOBuffer txFifo;                      // FIFO for transmit data
    uint8_t txBuf[CONFIG_AFSK_TX_BUFLEN];   // Actial data storage for said FIFO

    volatile bool sending;                  // Set when modem is sending

    // Demodulation values
    FIFOBuffer delayFifo;                   // Delayed FIFO for frequency discrimination
    int8_t delayBuf[SAMPLESPERBIT / 2 + 1]; // Actual data storage for said FIFO

    FIFOBuffer rxFifo;                      // FIFO for received data
    uint8_t rxBuf[CONFIG_AFSK_RX_BUFLEN];   // Actual data storage for said FIFO

    int16_t iirX[2];                        // IIR Filter X cells
    int16_t iirY[2];                        // IIR Filter Y cells

    uint8_t sampledBits;                    // Bits sampled by the demodulator (at ADC speed)
    int8_t currentPhase;                    // Current phase of the demodulator
    uint8_t actualBits;                     // Actual found bits at correct bitrate

    volatile int status;                    // Status of the modem, 0 means OK

} Afsk;

#define DIV_ROUND(dividend, divisor)  (((dividend) + (divisor) / 2) / (divisor))
#define MARK_INC   (uint16_t)(DIV_ROUND(SIN_LEN * (uint32_t)MARK_FREQ, CONFIG_AFSK_DAC_SAMPLERATE))
#define SPACE_INC  (uint16_t)(DIV_ROUND(SIN_LEN * (uint32_t)SPACE_FREQ, CONFIG_AFSK_DAC_SAMPLERATE))

#define AFSK_DAC_IRQ_START()   do { extern bool hw_afsk_dac_isr; hw_afsk_dac_isr = true; } while (0)
#define AFSK_DAC_IRQ_STOP()    do { extern bool hw_afsk_dac_isr; hw_afsk_dac_isr = false; } while (0)
#define AFSK_DAC_INIT()        do { DAC_DDR |= 0xF0; } while (0)

// Here's some macros for controlling the RX/TX LEDs
// THE _INIT() functions writes to the DDRB register
// to configure the pins as output pins, and the _ON()
// and _OFF() functions writes to the PORT registers
// to turn the pins on or off.
#define LED_TX_INIT() do { LED_DDR |= _BV(1); } while (0)
#define LED_TX_ON()   do { LED_PORT |= _BV(1); } while (0)
#define LED_TX_OFF()  do { LED_PORT &= ~_BV(1); } while (0)

#define LED_RX_INIT() do { LED_DDR |= _BV(2); } while (0)
#define LED_RX_ON()   do { LED_PORT |= _BV(2); } while (0)
#define LED_RX_OFF()  do { LED_PORT &= ~_BV(2); } while (0)

void AFSK_init(Afsk *afsk);
void AFSK_transmit(char *buffer, size_t size);
void AFSK_poll(Afsk *afsk);

#endif