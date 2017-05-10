
#include "AltSoftSerial.h"

#if defined(ALTSS_HAVE_TIMER4)

// define rx and tx pins

#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#define ALTSS_USE_TIMER4
#define INPUT_CAPTURE_PIN       49 // receive
#define OUTPUT_COMPARE_A_PIN    6 // transmit
#define OUTPUT_COMPARE_B_PIN    7 // unusable PWM
#define OUTPUT_COMPARE_C_PIN    8 // unusable PWM

#else
  #error "Rx and Tx pin for the board not defined!"
#endif

#define CAPTURE_INTERRUPT     TIMER4_CAPT_vect
#define COMPARE_A_INTERRUPT   TIMER4_COMPA_vect
#define COMPARE_B_INTERRUPT   TIMER4_COMPB_vect

AltSoftSerial   AltSoftSerial4(  INPUT_CAPTURE_PIN,
                 OUTPUT_COMPARE_A_PIN, 
           &TIMSK4,
                     &TCCR4A,
                     &TCCR4B,
                     ICNC4,
                     CS40,
                     CS41,
                     CS42,
                     COM4A1,
                     COM4A0,
                     ICES4,
                     &TIFR4,
                     ICF4,
                     OCF4A,
                     OCF4B,
                     ICIE4,
                     OCIE4A,
                     OCIE4B,
                     &TCNT4,
                     &ICR4,
                     &OCR4A,
                     &OCR4B);

ISR(COMPARE_A_INTERRUPT) {
  AltSoftSerial4.compareAInterrupt_isr();
}

ISR(CAPTURE_INTERRUPT) {
  AltSoftSerial4.captureInterrupt_isr();
}

ISR(COMPARE_B_INTERRUPT) {
    AltSoftSerial4.compareBInterrupt_isr();
}

#endif