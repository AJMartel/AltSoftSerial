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

    volatile uint8_t *TIMSKn;
    volatile uint8_t *TCCRnA;
    volatile uint8_t *TCCRnB;
    uint8_t ICNCn;
    uint8_t CSn0;
    uint8_t CSn1;
    uint8_t CSn2;
    uint8_t COMnA1;
    uint8_t COMnA0;
    uint8_t ICESn;
    volatile uint8_t *TIFRn;
    uint8_t ICFn;
    uint8_t OCFnA;
    uint8_t OCFnB;
    uint8_t ICIEn;
    uint8_t OCIEnA;
    uint8_t OCIEnB;
    volatile uint16_t *TCNTn;
    volatile uint16_t *ICRn;
    volatile uint16_t *OCRnA;
    volatile uint16_t *OCRnB;

public:
	AltSoftSerial(  volatile uint8_t *TIMSKn,
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

    this->TIMSKn = TIMSKn;
    this->TCCRnA = TCCRnA;
    this->TCCRnB = TCCRnB;
    this->ICNCn = ICNCn;
    this->CSn0 = CSn0;
    this->CSn1 = CSn1;
    this->CSn2 = CSn2;
    this->COMnA1 = COMnA1;
    this->COMnA0 = COMnA0;
    this->ICESn = ICESn;
    this->TIFRn = TIFRn;
    this->ICFn = ICFn;
    this->OCFnA = OCFnA;
    this->OCFnB = OCFnB;
    this->ICIEn = ICIEn;
    this->OCIEnA = OCIEnA;
    this->OCIEnB = OCIEnB;
    this->TCNTn = TCNTn;
    this->ICRn = ICRn;
    this->OCRnA = OCRnA;
    this->OCRnB = OCRnB;
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
	void compareInterrupt_isr();
	void compareBInterrupt_isr();

private:
	void init(uint32_t cycles_per_bit);
	void writeByte(uint8_t byte);


    void CONFIG_TIMER_NOPRESCALE()    { *TIMSKn = 0, *TCCRnA = 0, *TCCRnB = (1<<ICNCn) | (1<<CSn0); }
    void CONFIG_TIMER_PRESCALE_8()    { *TIMSKn = 0, *TCCRnA = 0, *TCCRnB = (1<<ICNCn) | (1<<CSn1); }
    void CONFIG_TIMER_PRESCALE_256()  { *TIMSKn = 0, *TCCRnA = 0, *TCCRnB = (1<<ICNCn) | (1<<CSn2); }
    void CONFIG_MATCH_NORMAL()        { *TCCRnA = *TCCRnA & ~((1<<COMnA1) | (1<<COMnA0)); }
    void CONFIG_MATCH_TOGGLE()        { *TCCRnA = (*TCCRnA & ~(1<<COMnA1)) | (1<<COMnA0); }
    void CONFIG_MATCH_CLEAR()         { *TCCRnA = (*TCCRnA | (1<<COMnA1)) & ~(1<<COMnA0); }
    void CONFIG_MATCH_SET()           { *TCCRnA = *TCCRnA | ((1<<COMnA1) | (1<<COMnA0)); }
    void CONFIG_CAPTURE_FALLING_EDGE()   { *TCCRnB &= ~(1<<ICESn); }
    void CONFIG_CAPTURE_RISING_EDGE()    { *TCCRnB |= (1<<ICESn); }
    void ENABLE_INT_INPUT_CAPTURE()      { *TIFRn = (1<<ICFn), *TIMSKn = (1<<ICIEn); }
    void ENABLE_INT_COMPARE_A()    { *TIFRn = (1<<OCFnA), *TIMSKn |= (1<<OCIEnA); }
    void ENABLE_INT_COMPARE_B()    { *TIFRn = (1<<OCFnB), *TIMSKn |= (1<<OCIEnB); }
    void DISABLE_INT_INPUT_CAPTURE()    { *TIMSKn &= ~(1<<ICIEn); }
    void DISABLE_INT_COMPARE_A()        { *TIMSKn &= ~(1<<OCIEnA); }
    void DISABLE_INT_COMPARE_B()        { *TIMSKn &= ~(1<<OCIEnB); }
    uint16_t GET_TIMER_COUNT()          { return *TCNTn; }
    uint16_t GET_INPUT_CAPTURE()        { return *ICRn; }
    uint16_t GET_COMPARE_A()            { return *OCRnA; }
    uint16_t GET_COMPARE_B()            { return *OCRnB; }
    void SET_COMPARE_A(uint16_t val)	{ *OCRnA = (val); }
    void SET_COMPARE_B(uint16_t val)	{ *OCRnB = (val); }
};

extern AltSoftSerial AltSoftSerial5;

#endif
