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






// Teensy 2.0
//
#if defined(__AVR_ATmega32U4__) && defined(CORE_TEENSY)
  #define ALTSS_HAVE_TIMER1
  #define ALTSS_HAVE_TIMER3

// Teensy++ 2.0
#elif defined(__AVR_AT90USB1286__) && defined(CORE_TEENSY)
  #define ALTSS_HAVE_TIMER1
  #define ALTSS_HAVE_TIMER3

// Wiring-S
#elif defined(__AVR_ATmega644P__) && defined(WIRING)
  #define ALTSS_HAVE_TIMER1

// Arduino Uno, Duemilanove, LilyPad, etc
#elif defined(__AVR_ATmega168__) || defined(__AVR_ATmega328P__)
  #define ALTSS_HAVE_TIMER1

// Arduino Leonardo & Yun (from Cristian Maglie)
#elif defined(ARDUINO_AVR_YUN) || defined(ARDUINO_AVR_LEONARDO) || defined(__AVR_ATmega32U4__)
  #define ALTSS_HAVE_TIMER3

// Arduino Mega
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  #define ALTSS_HAVE_TIMER1
  #define ALTSS_HAVE_TIMER4
  #define ALTSS_HAVE_TIMER5

// Sanguino
#elif defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644__)
  #define ALTSS_HAVE_TIMER1

#else
  #error "Please define your board"

#endif






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

	volatile uint8_t tx_state; // If 0: nothing to transmit
                             // If 1 to 11, number of bits that would have been transmitted before next call 
                             // to compareAInterrupt_isr() is made.
                             // any other value is not possible

	uint8_t tx_byte; // the byte currently begin tranmitted (right shift by tx_state bits)
	uint8_t tx_bit;  // the value of bit to be transmitted next time compareAInterrupt_isr() is called

	volatile uint8_t tx_buffer_head;
	volatile uint8_t tx_buffer_tail;

	volatile uint8_t tx_buffer[TX_BUFFER_SIZE];

  uint8_t _input_capture_pin;
  uint8_t _output_compare_A_pin;

  /* Variables to store register addresses. n in the following variables refers to timer number. */
  
  volatile uint8_t *_TCCRnA;  // Timer/Counter control register A
  uint8_t _COMnA1;  // Compare Output Mode in TCCRnA register
  uint8_t _COMnA0;  // Compare Output Mode in TCCRnA register

  volatile uint8_t *_TCCRnB;  // Timer/Counter control register B
  uint8_t _ICNCn;   // Input capture noise cancel in TCCRnB register
  uint8_t _CSn0;    // Clock select in TCCRnB register
  uint8_t _CSn1;    // Clock select in TCCRnB register
  uint8_t _CSn2;    // Clock select in TCCRnB register
  uint8_t _ICESn;   // Input Capture Edge select in TCCRnB register


  volatile uint8_t *_TIFRn; // Timer/Counter Interrupt Flag register
  uint8_t _ICFn;    // Input capture flag
  uint8_t _OCFnA;   // Output compare A match flag
  uint8_t _OCFnB;   // output compare B match flag

  volatile uint8_t *_TIMSKn;  // Timer interrupt mask register
  uint8_t _ICIEn;   // Input capture interrupt enable
  uint8_t _OCIEnA;  // Output Compare A Match Interrupt Enable
  uint8_t _OCIEnB;  // Output Compare B Match Interrupt Enable
  
  volatile uint16_t *_TCNTn;  // Timer/Counter Counter Value
  volatile uint16_t *_ICRn;   // Input capture register
  volatile uint16_t *_OCRnA;  // Output compare register A
  volatile uint16_t *_OCRnB;  // Output compare register B

  bool _useICP;

public:

  /* n can vary from 0 to 5. 
     All .h files are in hardware/tools/avr/avr/include/avr
     Single .h file corresponds to single processor
     Run the given command in above folder to verify
   * Bitmasks:
   *    ICNCn - For given processor, for all n, value is same. 
   *            To verify: $ for f in *.h; do grep --with-filename ICNC $f;  echo ""; done;
   *
   *    CSn0 - 0 for all processors for all n. To verify: $grep -E "(( )|($(printf '\t')))CS[0-5]0" *.h;
   *    CSn1 - 0 for all processors for all n. To verify: $grep -E "(( )|($(printf '\t')))CS[0-5]1" *.h;
        CSn2 - 0 for all processors for all n. To verify: $grep -E "(( )|($(printf '\t')))CS[0-5]2" *.h;

        COMnA1 - For given processor, value CAN BE different for different timers
   *
   */

  /**  input_rx_pin: ICP pin of the timer if useICP is true, other other INT pin
    */
	AltSoftSerial(uint8_t rx_pin,     // rx pin
                uint8_t output_compare_A_pin,  // tx pin
                bool useICP = true,         // true if rx is ICP pin
                uint8_t timerNum = 1) { // to be used if rx is not ICP pin


    _input_capture_pin = rx_pin;
    _output_compare_A_pin = output_compare_A_pin;
    
    #if defined(ALTSS_HAVE_TIMER0)
      if(timerNum == 0) {
        _TCCRnA = &TCCR0A; _COMnA1 = COM0A1; _COMnA0 = COM0A0;
        _TCCRnB = &TCCR0B; _ICNCn = ICNC0; _CSn0 = CS00; _CSn1 = CS01; _CSn2 = CS02; _ICESn = ICES0;
        _TIFRn = &TIFR0; _ICFn = ICF0; _OCFnA = OCF0A; _OCFnB = OCF0B;
        _TIMSKn = &TIMSK0; _ICIEn = ICIE0; _OCIEnA = OCIE0A; _OCIEnB = OCIE0B;
        _TCNTn = &TCNT0;
        _ICRn = &ICR0;
        _OCRnA = &OCR0A;
        _OCRnB = &OCR0B;
      }
    // #endif

    #elif defined(ALTSS_HAVE_TIMER1)
      if(timerNum == 1) {
        _TCCRnA = &TCCR1A; _COMnA1 = COM1A1; _COMnA0 = COM1A0;
        _TCCRnB = &TCCR1B; _ICNCn = ICNC1; _CSn0 = CS10; _CSn1 = CS11; _CSn2 = CS12; _ICESn = ICES1;
        _TIFRn = &TIFR1; _ICFn = ICF1; _OCFnA = OCF1A; _OCFnB = OCF1B;
        _TIMSKn = &TIMSK1; _ICIEn = ICIE1; _OCIEnA = OCIE1A; _OCIEnB = OCIE1B;
        _TCNTn = &TCNT1;
        _ICRn = &ICR1;
        _OCRnA = &OCR1A;
        _OCRnB = &OCR1B;
      }
    // #endif

    #elif defined(ALTSS_HAVE_TIMER2)
      if(timerNum == 2) {
        _TCCRnA = &TCCR2A; _COMnA1 = COM2A1; _COMnA0 = COM2A0;
        _TCCRnB = &TCCR2B; _ICNCn = ICNC2; _CSn0 = CS20; _CSn1 = CS21; _CSn2 = CS22; _ICESn = ICES2;
        _TIFRn = &TIFR2; _ICFn = ICF2; _OCFnA = OCF2A; _OCFnB = OCF2B;
        _TIMSKn = &TIMSK2; _ICIEn = ICIE2; _OCIEnA = OCIE2A; _OCIEnB = OCIE2B;
        _TCNTn = &TCNT2;
        _ICRn = &ICR2;
        _OCRnA = &OCR2A;
        _OCRnB = &OCR2B;
      }
    #else
      #error "Ha!"
    #endif


    _useICP = useICP;
    }

	~AltSoftSerial() { end(); }

	/* static class refers to functions that can be called even if there is no instance
	   of class. They can accessed only from static method members  */

  /* Function similar to those that are present in Serial library */
	void begin(uint32_t baud) { init((ALTSS_BASE_FREQ + baud / 2) / baud); }
	void end();
	int peek();
	int read();
	int available();

  /* In older Arduino, flush() function flushed the input stream instead of output. */
#if ARDUINO >= 100
	size_t write(uint8_t byte) { writeByte(byte); return 1; }
	void flush() { flushOutput(); }
#else
	void write(uint8_t byte) { writeByte(byte); }
	void flush() { flushInput(); }
#endif

	using Print::write;

  /* Empties the receive buffer */
	void flushInput();

  /* Waits for all the data in the transmit buffer to transmit. It is blocking function. */
	void flushOutput();

	/* Following fxns are for drop-in compatibility with NewSoftSerial (now SoftwareSerial), rxPin & 
  txPin ignored. They do nothing. */
	AltSoftSerial(uint8_t rxPin, uint8_t txPin, bool inverse = false) { }
	bool listen() { return false; }
	bool isListening() { return true; }
	bool overflow() { bool r = timing_error; timing_error = false; return r; }
	static int library_version() { return 1; }
	static void enable_timer0(bool enable) { }
	static bool timing_error;

  /* Rx is the input capture pin. Whenever edge is detected on the input capture pin, the interrupt
  is generated. */

  /* The following 3 functions are public but they should not be called by the user directly. They
  are public so that they can be called from ISR (see file AltSoftSerial1.cpp) */

  /* Invoked when Output Compare A interrupt is generated. Used for transmission. It is called  */
	void compareAInterrupt_isr();

  /* Used for reception */
	void captureInterrupt_isr();

  /* Invoked when Output Compare B interrupt is generated. Used for reception. */
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

  void CONFIG_CAPTURE_FALLING_EDGE()   { 
      if(_useICP) *_TCCRnB &= ~(1<<_ICESn); 
      else {
          DISABLE_INT_INPUT_CAPTURE();
          EICRA = (EICRA | (1<<ISC01)) & ~(1<<ISC00);
          ENABLE_INT_INPUT_CAPTURE();
      }
  }

  void CONFIG_CAPTURE_RISING_EDGE()    { 
      if(_useICP) *_TCCRnB |= (1<<_ICESn); 
      else {
          DISABLE_INT_INPUT_CAPTURE();
          EICRA = EICRA | ((1<<ISC01) | (1<<ISC00));
          ENABLE_INT_INPUT_CAPTURE();
      }
  }

  void ENABLE_INT_INPUT_CAPTURE()      { if(_useICP) *_TIFRn = (1<<_ICFn), *_TIMSKn = (1<<_ICIEn); 
                                         else EIMSK = (1<<INT0);}

  void ENABLE_INT_COMPARE_A()    { *_TIFRn = (1<<_OCFnA), *_TIMSKn |= (1<<_OCIEnA); }
  void ENABLE_INT_COMPARE_B()    { *_TIFRn = (1<<_OCFnB), *_TIMSKn |= (1<<_OCIEnB); }

  void DISABLE_INT_INPUT_CAPTURE()    { if(_useICP) *_TIMSKn &= ~(1<<_ICIEn); 
                                        else EIMSK &= ~(1<<INT0); }

  void DISABLE_INT_COMPARE_A()        { *_TIMSKn &= ~(1<<_OCIEnA); }
  void DISABLE_INT_COMPARE_B()        { *_TIMSKn &= ~(1<<_OCIEnB); }
  uint16_t GET_TIMER_COUNT()          { return *_TCNTn; }

  uint16_t GET_INPUT_CAPTURE()        { if(_useICP) return *_ICRn; else return *_TCNTn;}

  uint16_t GET_COMPARE_A()            { return *_OCRnA; }
  uint16_t GET_COMPARE_B()            { return *_OCRnB; }
  void SET_COMPARE_A(uint16_t val)	{ *_OCRnA = (val); }
    void SET_COMPARE_B(uint16_t val)	{ *_OCRnB = (val); }
};

#endif // AltSoftSerial_h
