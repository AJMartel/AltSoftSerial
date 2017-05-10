/* An Alternative Software Serial Library
 * http://www.pjrc.com/teensy/td_libs_AltSoftSerial.html
 * Copyright (c) 2014 PJRC.COM, LLC, Paul Stoffregen, paul@pjrc.com
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef AltSoftSerial_h
#define AltSoftSerial_h

#include <inttypes.h>

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#include "pins_arduino.h"
#endif

#if defined(__arm__) && defined(CORE_TEENSY)
#define ALTSS_BASE_FREQ F_BUS
#else
#define ALTSS_BASE_FREQ F_CPU
#endif

#define TX_BUFFER_SIZE 68
#define RX_BUFFER_SIZE 80

class AltSoftSerial : public Stream
{
private:
	uint16_t ticks_per_bit=0;  // number of timer counts for tranmission/reception of 1 bit

	uint8_t rx_state; 
	uint8_t rx_byte;
	uint8_t rx_bit;
	uint16_t rx_target;
	uint16_t rx_stop_ticks;
	volatile uint8_t rx_buffer_head;
	volatile uint8_t rx_buffer_tail;
	volatile uint8_t rx_buffer[RX_BUFFER_SIZE];

	volatile uint8_t tx_state; // number of bits already transmitted
	uint8_t tx_byte; // the byte currently begin tranmitted (right shift by tx_state bits)
	uint8_t tx_bit; // the value of bit to transmitted next time compareAInterrupt_isr() is called

	volatile uint8_t tx_buffer_head;
	volatile uint8_t tx_buffer_tail;

	volatile uint8_t tx_buffer[TX_BUFFER_SIZE];

    volatile uint8_t _input_capture_pin;
    volatile uint8_t _output_compare_A_pin;

    volatile uint8_t *_TIMSKn;
    volatile uint8_t *_TCCRnA;
    volatile uint8_t *_TCCRnB;
    uint8_t _ICNCn;
    uint8_t _CSn0;
    uint8_t _CSn1;
    uint8_t _CSn2;
    uint8_t _COMnA1;
    uint8_t _COMnA0;
    uint8_t _ICESn;
    volatile uint8_t *_TIFRn;
    uint8_t _ICFn;
    uint8_t _OCFnA;
    uint8_t _OCFnB;
    uint8_t _ICIEn;
    uint8_t _OCIEnA;
    uint8_t _OCIEnB;
    volatile uint16_t *_TCNTn;
    volatile uint16_t *_ICRn;
    volatile uint16_t *_OCRnA;
    volatile uint16_t *_OCRnB;

public:
	AltSoftSerial(  volatile uint8_t input_capture_pin,
                    volatile uint8_t output_compare_A_pin,

                    volatile uint8_t *TIMSKn,
                    volatile uint8_t *TCCRnA,
                    volatile uint8_t *TCCRnB,
                    uint8_t ICNCn,
                    uint8_t CSn0,
                    uint8_t CSn1,
                    uint8_t CSn2,
                    uint8_t COMnA1,
                    uint8_t COMnA0,
                    uint8_t ICESn,
                    volatile uint8_t *TIFRn,
                    uint8_t ICFn,
                    uint8_t OCFnA,
                    uint8_t OCFnB,
                    uint8_t ICIEn,
                    uint8_t OCIEnA,
                    uint8_t OCIEnB,
                    volatile uint16_t *TCNTn,
                    volatile uint16_t *ICRn,
                    volatile uint16_t *OCRnA,
                    volatile uint16_t *OCRnB) {

    _input_capture_pin = input_capture_pin;
    _output_compare_A_pin = output_compare_A_pin;

    _TIMSKn = TIMSKn;
    _TCCRnA = TCCRnA;
    _TCCRnB = TCCRnB;
    _ICNCn = ICNCn;
    _CSn0 = CSn0;
    _CSn1 = CSn1;
    _CSn2 = CSn2;
    _COMnA1 = COMnA1;
    _COMnA0 = COMnA0;
    _ICESn = ICESn;
    _TIFRn = TIFRn;
    _ICFn = ICFn;
    _OCFnA = OCFnA;
    _OCFnB = OCFnB;
    _ICIEn = ICIEn;
    _OCIEnA = OCIEnA;
    _OCIEnB = OCIEnB;
    _TCNTn = TCNTn;
    _ICRn = ICRn;
    _OCRnA = OCRnA;
    _OCRnB = OCRnB;
    }
	~AltSoftSerial() { end(); }

	/* static class refers to functions that can be called even if there is no instance
	   of class. They can access only static data members  */

	void begin(uint32_t baud) { init((ALTSS_BASE_FREQ + baud / 2) / baud); }
	void end();
	int peek();
	int read();
	int available();
#if ARDUINO >= 100
	size_t write(uint8_t byte) { writeByte(byte); return 1; }
	void flush() { flushOutput(); }
#else
	void write(uint8_t byte) { writeByte(byte); }
	void flush() { flushInput(); }
#endif
	using Print::write;
	void flushInput();
	void flushOutput();
	// for drop-in compatibility with NewSoftSerial, rxPin & txPin ignored
	AltSoftSerial(uint8_t rxPin, uint8_t txPin, bool inverse = false) { }
	bool listen() { return false; }
	bool isListening() { return true; }
	bool overflow() { bool r = timing_error; timing_error = false; return r; }
	static int library_version() { return 1; }
	static void enable_timer0(bool enable) { }
	static bool timing_error;

	void compareAInterrupt_isr();
	void captureInterrupt_isr();
	void compareBInterrupt_isr();

private:
	void init(uint32_t cycles_per_bit);
	void writeByte(uint8_t byte);


    void CONFIG_TIMER_NOPRESCALE()    { *_TIMSKn = 0, *_TCCRnA = 0, *_TCCRnB = (1<<_ICNCn) | (1<<_CSn0); }
    void CONFIG_TIMER_PRESCALE_8()    { *_TIMSKn = 0, *_TCCRnA = 0, *_TCCRnB = (1<<_ICNCn) | (1<<_CSn1); }
    void CONFIG_TIMER_PRESCALE_256()  { *_TIMSKn = 0, *_TCCRnA = 0, *_TCCRnB = (1<<_ICNCn) | (1<<_CSn2); }
    void CONFIG_MATCH_NORMAL()        { *_TCCRnA = *_TCCRnA & ~((1<<_COMnA1) | (1<<_COMnA0)); }
    void CONFIG_MATCH_TOGGLE()        { *_TCCRnA = (*_TCCRnA & ~(1<<_COMnA1)) | (1<<_COMnA0); }
    void CONFIG_MATCH_CLEAR()         { *_TCCRnA = (*_TCCRnA | (1<<_COMnA1)) & ~(1<<_COMnA0); }
    void CONFIG_MATCH_SET()           { *_TCCRnA = *_TCCRnA | ((1<<_COMnA1) | (1<<_COMnA0)); }
    void CONFIG_CAPTURE_FALLING_EDGE()   { *_TCCRnB &= ~(1<<_ICESn); }
    void CONFIG_CAPTURE_RISING_EDGE()    { *_TCCRnB |= (1<<_ICESn); }
    void ENABLE_INT_INPUT_CAPTURE()      { *_TIFRn = (1<<_ICFn), *_TIMSKn = (1<<_ICIEn); }
    void ENABLE_INT_COMPARE_A()    { *_TIFRn = (1<<_OCFnA), *_TIMSKn |= (1<<_OCIEnA); }
    void ENABLE_INT_COMPARE_B()    { *_TIFRn = (1<<_OCFnB), *_TIMSKn |= (1<<_OCIEnB); }
    void DISABLE_INT_INPUT_CAPTURE()    { *_TIMSKn &= ~(1<<_ICIEn); }
    void DISABLE_INT_COMPARE_A()        { *_TIMSKn &= ~(1<<_OCIEnA); }
    void DISABLE_INT_COMPARE_B()        { *_TIMSKn &= ~(1<<_OCIEnB); }
    uint16_t GET_TIMER_COUNT()          { return *_TCNTn; }
    uint16_t GET_INPUT_CAPTURE()        { return *_ICRn; }
    uint16_t GET_COMPARE_A()            { return *_OCRnA; }
    uint16_t GET_COMPARE_B()            { return *_OCRnB; }
    void SET_COMPARE_A(uint16_t val)	{ *_OCRnA = (val); }
    void SET_COMPARE_B(uint16_t val)	{ *_OCRnB = (val); }
};







// Teensy 2.0
//
#if defined(__AVR_ATmega32U4__) && defined(CORE_TEENSY)
  extern AltSoftSerial AltSoftSerial1;
  extern AltSoftSerial AltSoftSerial3;
  #define ALTSS_HAVE_TIMER1
  #define ALTSS_HAVE_TIMER3
 //#define ALTSS_USE_TIMER1
 //#define INPUT_CAPTURE_PIN        22 // receive
 //#define OUTPUT_COMPARE_A_PIN     14 // transmit
 //#define OUTPUT_COMPARE_B_PIN     15 // unusable PWM
 //#define OUTPUT_COMPARE_C_PIN      4 // unusable PWM

 #define ALTSS_USE_TIMER3
 #define INPUT_CAPTURE_PIN      10 // receive
 #define OUTPUT_COMPARE_A_PIN        9 // transmit



// Teensy++ 2.0
//
#elif defined(__AVR_AT90USB1286__) && defined(CORE_TEENSY)
  extern AltSoftSerial AltSoftSerial1;
  extern AltSoftSerial AltSoftSerial3;
  #define ALTSS_HAVE_TIMER1
  #define ALTSS_HAVE_TIMER3

 #define ALTSS_USE_TIMER1
 #define INPUT_CAPTURE_PIN       4 // receive
 #define OUTPUT_COMPARE_A_PIN       25 // transmit
 #define OUTPUT_COMPARE_B_PIN       26 // unusable PWM
 #define OUTPUT_COMPARE_C_PIN       27 // unusable PWM

 //#define ALTSS_USE_TIMER3
 //#define INPUT_CAPTURE_PIN        17 // receive
 //#define OUTPUT_COMPARE_A_PIN     16 // transmit
 //#define OUTPUT_COMPARE_B_PIN     15 // unusable PWM
 //#define OUTPUT_COMPARE_C_PIN     14 // unusable PWM


// Teensy 3.x
//
// #elif defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
//  #define ALTSS_USE_FTM0
//  #define INPUT_CAPTURE_PIN      20 // receive       (FTM0_CH5)
//  #define OUTPUT_COMPARE_A_PIN       21 // transmit      (FTM0_CH6)
//  #define OUTPUT_COMPARE_B_PIN       22 // unusable PWM  (FTM0_CH0)
//  #define OUTPUT_COMPARE_C_PIN       23 // PWM usable fixed freq
//  #define OUTPUT_COMPARE_D_PIN        5 // PWM usable fixed freq
//  #define OUTPUT_COMPARE_E_PIN        6 // PWM usable fixed freq
//  #define OUTPUT_COMPARE_F_PIN        9 // PWM usable fixed freq
//  #define OUTPUT_COMPARE_G_PIN       10 // PWM usable fixed freq


// Wiring-S
//
#elif defined(__AVR_ATmega644P__) && defined(WIRING)
  extern AltSoftSerial AltSoftSerial1;
  #define ALTSS_HAVE_TIMER1
 #define ALTSS_USE_TIMER1
 // #define INPUT_CAPTURE_PIN       6 // receive
 // #define OUTPUT_COMPARE_A_PIN        5 // transmit
 // #define OUTPUT_COMPARE_B_PIN        4 // unusable PWM



// Arduino Uno, Duemilanove, LilyPad, etc
//
#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  extern AltSoftSerial AltSoftSerial1;
  #define ALTSS_HAVE_TIMER1

 #define ALTSS_USE_TIMER1
 #define INPUT_CAPTURE_PIN       8 // receive
 #define OUTPUT_COMPARE_A_PIN        9 // transmit
 #define OUTPUT_COMPARE_B_PIN       10 // unusable PWM


// Arduino Leonardo & Yun (from Cristian Maglie)
//
#elif defined(ARDUINO_AVR_YUN) || defined(ARDUINO_AVR_LEONARDO) || defined(__AVR_ATmega32U4__)

  extern AltSoftSerial AltSoftSerial3;
  #define ALTSS_HAVE_TIMER3

  //#define ALTSS_USE_TIMER1
  //#define INPUT_CAPTURE_PIN       4  // receive
  //#define OUTPUT_COMPARE_A_PIN    9 // transmit
  //#define OUTPUT_COMPARE_B_PIN    10 // unusable PWM
  //#define OUTPUT_COMPARE_C_PIN    11 // unusable PWM

  #define ALTSS_USE_TIMER3
  #define INPUT_CAPTURE_PIN     13 // receive
  #define OUTPUT_COMPARE_A_PIN      5 // transmit


// Arduino Mega
//
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)

  extern AltSoftSerial AltSoftSerial4;
  extern AltSoftSerial AltSoftSerial5;
  #define ALTSS_HAVE_TIMER4
  #define ALTSS_HAVE_TIMER5

// Sanguino
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__)
  
  extern AltSoftSerial AltSoftSerial1;  
  #define ALTSS_HAVE_TIMER1

 // #define ALTSS_USE_TIMER1
 // #define INPUT_CAPTURE_PIN      14 // receive
 // #define OUTPUT_COMPARE_A_PIN       13 // transmit
 // #define OUTPUT_COMPARE_B_PIN       12 // unusable PWM

#else
  #error "Please define your board"

#endif


#endif // AltSoftSerial_h
