
#include "AltSoftSerial.h"

#if defined(ALTSS_HAVE_TIMER5)

// define rx and tx pins

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#define ALTSS_USE_TIMER5
#define INPUT_CAPTURE_PIN           48 // receive
#define OUTPUT_COMPARE_A_PIN        46 // transmit
#define OUTPUT_COMPARE_B_PIN        45 // unusable PWM
#define OUTPUT_COMPARE_C_PIN        44 // unusable PWM

#else
  #error "Rx and Tx pin for the board not defined!"
#endif

#define CAPTURE_INTERRUPT     TIMER5_CAPT_vect
#define COMPARE_A_INTERRUPT   TIMER5_COMPA_vect
#define COMPARE_B_INTERRUPT   TIMER5_COMPB_vect

AltSoftSerial   AltSoftSerial5(  INPUT_CAPTURE_PIN,
                                 OUTPUT_COMPARE_A_PIN, 
                     &TIMSK5,
                     &TCCR5A,
                     &TCCR5B,
                     ICNC5,
                     CS50,
                     CS51,
                     CS52,
                     COM5A1,
                     COM5A0,
                     ICES5,
                     &TIFR5,
                     ICF5,
                     OCF5A,
                     OCF5B,
                     ICIE5,
                     OCIE5A,
                     OCIE5B,
                     &TCNT5,
                     &ICR5,
                     &OCR5A,
                     &OCR5B);

ISR(COMPARE_A_INTERRUPT) {
    AltSoftSerial5.compareAInterrupt_isr();
}

ISR(CAPTURE_INTERRUPT) {
    AltSoftSerial5.captureInterrupt_isr();
}

ISR(COMPARE_B_INTERRUPT) {
    AltSoftSerial5.compareBInterrupt_isr();
}

#endif