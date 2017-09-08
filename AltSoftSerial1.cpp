
#include "AltSoftSerial.h"

#if defined(ALTSS_HAVE_TIMER1)

// define rx and tx pins

// ALTSS - Alt Soft Serial
// The pin number number are Arduino Pin Numbers and not Atmega Pin number

// Mega
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

#define ALTSS_USE_TIMER1
#define INPUT_CAPTURE_PIN       21 // receive (other pin with external pin used as ICP is not connected)
#define OUTPUT_COMPARE_A_PIN    11 // transmit
#define OUTPUT_COMPARE_B_PIN    12 // unusable PWM
#define OUTPUT_COMPARE_C_PIN    13 // unusable PWM

#define USE_ICP                false


// Arduino Uno and Pro Mini
#elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__)

#define ALTSS_USE_TIMER1
#define INPUT_CAPTURE_PIN       8 // receive (other pin with external pin used as ICP is not connected)
#define OUTPUT_COMPARE_A_PIN    9 // transmit
#define OUTPUT_COMPARE_B_PIN    10 // unusable PWM

#define USE_ICP                true

#else
  #error "Rx and Tx pin for the board not defined!"
#endif


#define CAPTURE_INTERRUPT     TIMER1_CAPT_vect
#define COMPARE_A_INTERRUPT   TIMER1_COMPA_vect
#define COMPARE_B_INTERRUPT   TIMER1_COMPB_vect

AltSoftSerial   AltSoftSerial1(  INPUT_CAPTURE_PIN,
                 OUTPUT_COMPARE_A_PIN, 
                     &TIMSK1,
                     &TCCR1A,
                     &TCCR1B,
                     ICNC1,
                     CS10,
                     CS11,
                     CS12,
                     COM1A1,
                     COM1A0,
                     ICES1,
                     &TIFR1,
                     ICF1,
                     OCF1A,
                     OCF1B,
                     ICIE1,
                     OCIE1A,
                     OCIE1B,
                     &TCNT1,
                     &ICR1,
                     &OCR1A,
                     &OCR1B,

                     USE_ICP);


ISR(COMPARE_A_INTERRUPT) {
  AltSoftSerial1.compareAInterrupt_isr();
}

// ISR(INT0_vect) {
//   AltSoftSerial1.captureInterrupt_isr();  
// }

ISR(CAPTURE_INTERRUPT) {
  AltSoftSerial1.captureInterrupt_isr();
}

ISR(COMPARE_B_INTERRUPT) {
    AltSoftSerial1.compareBInterrupt_isr();
}



#endif