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
	uint16_t ticks_per_bit=0;

	uint8_t rx_state;
	uint8_t rx_byte;
	uint8_t rx_bit;
	uint16_t rx_target;
	uint16_t rx_stop_ticks;
	volatile uint8_t rx_buffer_head;
	volatile uint8_t rx_buffer_tail;
	volatile uint8_t rx_buffer[RX_BUFFER_SIZE];

	volatile uint8_t tx_state;
	uint8_t tx_byte;
	uint8_t tx_bit;
	volatile uint8_t tx_buffer_head;
	volatile uint8_t tx_buffer_tail;

	volatile uint8_t tx_buffer[TX_BUFFER_SIZE];

public:
	AltSoftSerial() { }
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
	void compareInterrupt_isr();
	void compareBInterrupt_isr();

private:
	void init(uint32_t cycles_per_bit);
	void writeByte(uint8_t byte);

  void CONFIG_TIMER_NOPRESCALE()    { TIMSK5 = 0, TCCR5A = 0, TCCR5B = (1<<ICNC5) | (1<<CS50); }
  void CONFIG_TIMER_PRESCALE_8()    { TIMSK5 = 0, TCCR5A = 0, TCCR5B = (1<<ICNC5) | (1<<CS51); }
  void CONFIG_TIMER_PRESCALE_256()  { TIMSK5 = 0, TCCR5A = 0, TCCR5B = (1<<ICNC5) | (1<<CS52); }
  void CONFIG_MATCH_NORMAL()        { TCCR5A = TCCR5A & ~((1<<COM5A1) | (1<<COM5A0)); }
  void CONFIG_MATCH_TOGGLE()        { TCCR5A = (TCCR5A & ~(1<<COM5A1)) | (1<<COM5A0); }
  void CONFIG_MATCH_CLEAR()         { TCCR5A = (TCCR5A | (1<<COM5A1)) & ~(1<<COM5A0); }
  void CONFIG_MATCH_SET()           { TCCR5A = TCCR5A | ((1<<COM5A1) | (1<<COM5A0)); }
  void CONFIG_CAPTURE_FALLING_EDGE()   { TCCR5B &= ~(1<<ICES5); }
  void CONFIG_CAPTURE_RISING_EDGE()    { TCCR5B |= (1<<ICES5); }
  void ENABLE_INT_INPUT_CAPTURE()      { TIFR5 = (1<<ICF5), TIMSK5 = (1<<ICIE5); }
  void ENABLE_INT_COMPARE_A()    { TIFR5 = (1<<OCF5A), TIMSK5 |= (1<<OCIE5A); }
  void ENABLE_INT_COMPARE_B()    { TIFR5 = (1<<OCF5B), TIMSK5 |= (1<<OCIE5B); }
  void DISABLE_INT_INPUT_CAPTURE()    { TIMSK5 &= ~(1<<ICIE5); }
  void DISABLE_INT_COMPARE_A()        { TIMSK5 &= ~(1<<OCIE5A); }
  void DISABLE_INT_COMPARE_B()        { TIMSK5 &= ~(1<<OCIE5B); }
  uint16_t GET_TIMER_COUNT()      { return TCNT5; }
  uint16_t GET_INPUT_CAPTURE()    { return ICR5; }
  uint16_t GET_COMPARE_A()        { return OCR5A; }
  uint16_t GET_COMPARE_B()        { return OCR5B; }
  void SET_COMPARE_A(uint16_t val)	  { OCR5A = (val); }
  void SET_COMPARE_B(uint16_t val)	  { OCR5B = (val); }
};

extern AltSoftSerial AltSoftSerial5;

#endif
