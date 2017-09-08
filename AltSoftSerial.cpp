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

// Revisions are now tracked on GitHub
// https://github.com/PaulStoffregen/AltSoftSerial
//
// Version 1.2: Support Teensy 3.x
//
// Version 1.1: Improve performance in receiver code
//
// Version 1.0: Initial Release


#include "AltSoftSerial.h"
#include "config/AltSoftSerial_Boards.h"
#include "config/AltSoftSerial_Timers.h"

/****************************************/
/**          Initialization            **/
/****************************************/


bool AltSoftSerial::timing_error=false;


#ifndef INPUT_PULLUP
#define INPUT_PULLUP INPUT
#endif

#define MAX_COUNTS_PER_BIT  6241  // 65536 / 10.5

void AltSoftSerial::init(uint32_t cycles_per_bit)
{
	rx_bit = 0;
	rx_stop_ticks = 0;
	tx_state = 0;

	//Serial.printf("cycles_per_bit = %d\n", cycles_per_bit);
	if (cycles_per_bit < MAX_COUNTS_PER_BIT) {
		CONFIG_TIMER_NOPRESCALE();
	} else {
		cycles_per_bit /= 8;
		//Serial.printf("cycles_per_bit/8 = %d\n", cycles_per_bit);
		if (cycles_per_bit < MAX_COUNTS_PER_BIT) {
			CONFIG_TIMER_PRESCALE_8();
		} else {
#if defined(CONFIG_TIMER_PRESCALE_256)
			cycles_per_bit /= 32;
			//Serial.printf("cycles_per_bit/256 = %d\n", cycles_per_bit);
			if (cycles_per_bit < MAX_COUNTS_PER_BIT) {
				CONFIG_TIMER_PRESCALE_256();
			} else {
				return; // baud rate too low for AltSoftSerial
			}
#elif defined(CONFIG_TIMER_PRESCALE_128)
			cycles_per_bit /= 16;
			//Serial.printf("cycles_per_bit/128 = %d\n", cycles_per_bit);
			if (cycles_per_bit < MAX_COUNTS_PER_BIT) {
				CONFIG_TIMER_PRESCALE_128();
			} else {
				return; // baud rate too low for AltSoftSerial
			}
#else
			return; // baud rate too low for AltSoftSerial
#endif
		}
	}
	ticks_per_bit = cycles_per_bit;
	rx_stop_ticks = cycles_per_bit * 37 / 4; // 9 bit time (time from start of start bit to start of stop bit)

	pinMode(_input_capture_pin, INPUT_PULLUP);
	digitalWrite(_output_compare_A_pin, HIGH);
	pinMode(_output_compare_A_pin, OUTPUT);
	rx_state = 0;
	rx_buffer_head = 0;
	rx_buffer_tail = 0;
	tx_state = 0;
	tx_buffer_head = 0;
	tx_buffer_tail = 0;
	ENABLE_INT_INPUT_CAPTURE();
}

void AltSoftSerial::end(void)
{
	DISABLE_INT_COMPARE_B();
	DISABLE_INT_INPUT_CAPTURE();
	flushInput();
	flushOutput();
	DISABLE_INT_COMPARE_A();
	// TODO: restore timer to original settings?
}


/****************************************/
/**           Transmission             **/
/****************************************/

void AltSoftSerial::writeByte(uint8_t b)
{
	/* New data is entered at head, old data is transmitted from tail side 
	 * tx_buffer_head points to position, where there is valid data */
	uint8_t intr_state, head;

	head = tx_buffer_head + 1;
	if (head >= TX_BUFFER_SIZE) head = 0;

	/* Let all the previous data get transmitted */
	while (tx_buffer_tail == head) ; // wait until space in buffer

	/* Critical section
	 * tx_state, tx_byte, tx_bit are modified in compareAInterrupt_isr() */
	intr_state = SREG;
	cli();
	if (tx_state) {  // some byte is already being transmitted
		tx_buffer[head] = b;
		tx_buffer_head = head;
	} else {
		tx_state = 1;
		tx_byte = b;
		tx_bit = 0;		// Set next bit to be transmitted as low (start bit)
		ENABLE_INT_COMPARE_A();
		CONFIG_MATCH_CLEAR();
		SET_COMPARE_A(GET_TIMER_COUNT() + 16);
	}
	SREG = intr_state;
}

/*  The interrupts have priority in accordance with their Interrupt Vector position. 
The lower the Interrupt Vector address, the higher the priority.  

The priority of interrupts is: capture interrupt (highest priority), compare A interrupt, 
capture B interrupt (lowest priority)  */

/* Called when timer matches the value in register OCRA. This happens at 3 occassions:
	1. Start of first data bit
	2. Toggling of tx pin (except for start of first data bit)
	3. End of 2nd stop bit */
void AltSoftSerial::compareAInterrupt_isr()
{
	uint8_t state, byte, bit, head, tail;
	uint16_t target; // value of timer when tx pin has to be toggled

	/* Byte format is 1 start bit, 8 data bit, no parity bit, 2 stop bits */
	state = tx_state;  // the number of bits already transmitted
	byte = tx_byte;    // the byte to be transmitted

	target = GET_COMPARE_A();  // note down the timer value when this isr was invoked

	/* 1. Find the next bit (data bit or stop bit) that is different from current bit.
	   2. Get the timer value when the tx pin has to be toggled.
	   3. Set the timer to generate compare A interrupt when toggle has to take place.
	   4. Set the timer to toggle tx pin at approriate timer value */

	while (state < 10) {  // if byte has not been fully transmitted yet

		target += ticks_per_bit; // value for setting OCRA register to cause this interrupt 
		                         // to occur after 1 bit time
		
		if (state < 9) // if not all data-bits are transmitted
			bit = byte & 1;  // get the bit to be transmitted after 1 bit time
		else
			bit = 1; // stopbit
		byte >>= 1;
		state++;

		// if next bit is same as current bit, then there is no need to this interrupt
		// after 1 bit time. Find which next bit is different from current one, and set the isr
		// to fire at that time.
		if (bit != tx_bit) {
			if (bit) {
				CONFIG_MATCH_SET();   // set tx pin to high when timer value matches OCRA value
			} else {
				CONFIG_MATCH_CLEAR(); // set tx pin to low when timer value matches OCRA value
			}
			SET_COMPARE_A(target); 
			tx_bit = bit;
			tx_byte = byte;
			tx_state = state;
			// TODO: how to detect timing_error?
			return;
		}
	}

	// Control reaches this point if 
	//	- tx pin is not to be toggled till end of first stop bit (state = 10). This also means 
	//    currently tx pin is High and Timer output compare pin would be set to MATCH_SET.
	//  - we are at the end of 2nd stop bit (state = 11)


	head = tx_buffer_head;
	tail = tx_buffer_tail;  // the byte currently being transmiited or already transmitted
	if (head == tail) {
		if (state == 10) {
			// Wait for final stop bit to finish
			tx_state = 11;
			SET_COMPARE_A(target + ticks_per_bit);
		} else {
			tx_state = 0;
			CONFIG_MATCH_NORMAL();   // do nothing on output pin on compare match
			DISABLE_INT_COMPARE_A();
		}
	} else {
		if (++tail >= TX_BUFFER_SIZE) tail = 0;
		tx_buffer_tail = tail;
		tx_byte = tx_buffer[tail];
		tx_bit = 0; // start bit
		CONFIG_MATCH_CLEAR();
		if (state == 10)
			SET_COMPARE_A(target + ticks_per_bit);  // wait for stop bit com finish
		else
			SET_COMPARE_A(GET_TIMER_COUNT() + 16);
		tx_state = 1;
		// TODO: how to detect timing_error?
	}
}

void AltSoftSerial::flushOutput(void)
{
	while (tx_state) /* wait */ ;
}


/****************************************/
/**            Reception               **/
/****************************************/

/* Reception uses 2 ISRs: captureInterrupt_isr() and compareBInterrupt_isr().
 * These 2 ISRs don't preempt each other as they are designed to called at 
 * different times. */

/* This isr is called at start of start bit and at time when level change happens
 * during reception of 10 bits. */
void AltSoftSerial::captureInterrupt_isr()
{
	uint8_t state, bit, head;
	uint16_t capture, target;
	uint16_t offset, offset_overflow;

	/* When the interrupt occurs, the value of timer count TCNT register is written to
	 * ICR register. Read the contents of that register */
	capture = GET_INPUT_CAPTURE();
	bit = rx_bit;
	if (bit) {
		CONFIG_CAPTURE_FALLING_EDGE();
		rx_bit = 0;
	} else {
		CONFIG_CAPTURE_RISING_EDGE();
		rx_bit = 0x80;
	}
	state = rx_state;
	if (state == 0) { // start of start bit
		if (!bit) {
			uint16_t end = capture + rx_stop_ticks;
			SET_COMPARE_B(end);
			ENABLE_INT_COMPARE_B();
			rx_target = capture + ticks_per_bit + ticks_per_bit/2; // middle of 1st data bit
			rx_state = 1;
		}
	} else {
		target = rx_target;
		offset_overflow = 65535 - ticks_per_bit;
		while (1) {
			offset = capture - target;
			if (offset > offset_overflow) break;
			rx_byte = (rx_byte >> 1) | rx_bit;
			target += ticks_per_bit;
			state++;
			if (state >= 9) {
				DISABLE_INT_COMPARE_B();
				head = rx_buffer_head + 1;
				if (head >= RX_BUFFER_SIZE) head = 0;
				if (head != rx_buffer_tail) {
					rx_buffer[head] = rx_byte;
					rx_buffer_head = head;
				}
				CONFIG_CAPTURE_FALLING_EDGE();
				rx_bit = 0;
				rx_state = 0;
				return;
			}
		}
		rx_target = target;
		rx_state = state;
	}
	//if (GET_TIMER_COUNT() - capture > ticks_per_bit) AltSoftSerial::timing_error = true;
}

/* This function is called at start of stop bit if 8th data bit is not 1 */
void AltSoftSerial::compareBInterrupt_isr()
{
	uint8_t head, state, bit;

	DISABLE_INT_COMPARE_B();
	CONFIG_CAPTURE_FALLING_EDGE();
	state = rx_state;
	bit = rx_bit ^ 0x80; // invert the rx_bit
	while (state < 9) {
		rx_byte = (rx_byte >> 1) | bit;
		state++;
	}
	head = rx_buffer_head + 1;
	if (head >= RX_BUFFER_SIZE) head = 0;
	if (head != rx_buffer_tail) {
		rx_buffer[head] = rx_byte;
		rx_buffer_head = head;
	}
	rx_state = 0;
	CONFIG_CAPTURE_FALLING_EDGE();
	rx_bit = 0;
}



int AltSoftSerial::read(void)
{
	uint8_t head, tail, out;

	/* Store value of rx_buffer_head in temporary variable instead of using 
	   rx_buffer_head at multiple places inside the fxn because its value might
	   be changed by one of the ISRs */

	head = rx_buffer_head;  // can be changed by ISRs
	tail = rx_buffer_tail;  // read by ISRs but not written by them

	if (head == tail) return -1;
	if (++tail >= RX_BUFFER_SIZE) tail = 0;
	out = rx_buffer[tail];
	rx_buffer_tail = tail;
	return out;
}

int AltSoftSerial::peek(void)
{
	uint8_t head, tail;

	head = rx_buffer_head;
	tail = rx_buffer_tail;
	if (head == tail) return -1;
	if (++tail >= RX_BUFFER_SIZE) tail = 0;
	return rx_buffer[tail];
}

int AltSoftSerial::available(void)
{
	uint8_t head, tail;

	head = rx_buffer_head;
	tail = rx_buffer_tail;
	if (head >= tail) return head - tail;
	return RX_BUFFER_SIZE + head - tail;
}

void AltSoftSerial::flushInput(void)
{
	rx_buffer_head = rx_buffer_tail;
}


#ifdef ALTSS_USE_FTM0
void ftm0_isr(void)
{
	uint32_t flags = FTM0_STATUS;
	FTM0_STATUS = 0;
	if (flags & (1<<0) && (FTM0_C0SC & 0x40)) altss_compare_b_interrupt();
	if (flags & (1<<5)) altss_capture_interrupt();
	if (flags & (1<<6) && (FTM0_C6SC & 0x40)) altss_compare_a_interrupt();
}
#endif
