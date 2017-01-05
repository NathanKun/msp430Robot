/*
 * simpleMSP.h
 *
 *  Created on: 6 dec. 2016
 *      Author: Junyang HE
 */
#ifndef SIMPLEMSP_H_   /* Include guard */
#define SIMPLEMSP_H_

#include <msp430G2553.h>
#include <stdint.h>

#define HIGH 1
#define LOW 0

#define OUTPUT 1
#define INPUT 0

#define ENABLE 1
#define DISABLE 0

#define RISING_EDGE 1
#define FALLING_EDGE 0

#define P10 10
#define P11 11
#define P12 12
#define P13 13
#define P14 14
#define P15 15
#define P16 16
#define P17 17

#define P20 20
#define P21 21
#define P22 22
#define P23 23
#define P24 24
#define P25 25
#define P26 26
#define P27 27

//typedef enum { P10, P11, P12, P13, P14, P15, P16, P17 } port1Pins;
//typedef enum { P20, P21, P22, P23, P24, P25, P26, P27 } port2Pins;

typedef enum {
false = 0, true = 1}bool;

/*
 * 0000 A0
 * 0001 A1
 * 0010 A2
 * 0011 A3
 * 0100 A4
 * 0101 A5
 * 0110 A6
 * 0111 A7
 * 1000 VeREF+
 * 1001 VREF--/VeREF--
 * 1010 Temperature sensor
 * 1011 (VCC – VSS) / 2
 * */
typedef enum {
	A0 = 0, /* P10 */
	A1 = 1, /* P11 */
	A2 = 2, /* P12 */
	A3 = 3, /* P13 */
	A4 = 4, /* P14 */
	A5 = 5, /* P15 */
	A6 = 6, /* P16 */
	A7 = 7, /* P17 */
	VeREFp = 8,
	VREFnByVeREFn = 9,
	Temperature_sensor = 10,
	HalfOfVCCMinusVSS = 11
} inch;

/**
 * timer emum
 */
typedef enum {
TIMER0 = 0, TIMER1 = 1
} timer;

/*
 * Mode control for timerA
 * 00 Stop mode: the timer is halted.
 * 01 Up mode: the timer counts up to TACCR0.
 * 10 Continuous mode: the timer counts up to 0FFFFh.
 * 11 Up/down mode: the timer counts up to TACCR0 then down to 0000h.
 */
typedef enum {
STOP_MODE = MC_0, UP_MODE = MC_1, CONTINUOUS_MODE = MC_2, UP_DOWN_MODE = MC_3
} timer_mode_control;

/**
 * Output mode for timerA
 *
 * OUTMODx bits 7-5
 *
 * Output modes 2, 3, 6 and 7 are not useful for TACCR0 beacause EQUx=EQU0
 *
 * 000 OUT bit value
 * 001 Set
 * 010 Toggle/reset
 * 011 Set/reset
 * 100 Toggle
 * 101 Reset
 * 110 Toggle/set
 * 111 Reset/set
 */
typedef enum {
SET = OUTMOD_1,
TOGGLE_RESET = OUTMOD_2,
SET_RESET = OUTMOD_3,
TOGGLE = OUTMOD_4,
RESET = OUTMOD_5,
TOGGLE_SET = OUTMOD_6,
RESET_SET = OUTMOD_7
} timer_output_mode;

extern const uint8_t port1Pins[];
extern const uint8_t port2Pins[];

/**
 * set direction of a pin
 *
 * \param pin the pin's name
 *
 * \param dir INPUT or OUTPUT
 */
void directionPin(const uint8_t pin, const uint8_t dir);

/**
 * write a digital signal to a pin
 *
 * \param pin the pin's name
 *
 * \value HIGH or LOW
 */
void digitalWrite(const uint8_t pin, const uint8_t value);

/**
 * read a digital signal from a pin
 *
 * \param pin the pin's name
 *
 * \return HIGH or LOW
 */
int digitalRead(const uint8_t pin);

/**
 * select function mode for a pin
 *
 * \param pin the pin's name
 *
 * \param mode 0 to 3
 */
void selectPin(const uint8_t pin, const uint8_t mode);

/**
 * set pull-up for a pin
 *
 * \param pin the pin's name
 *
 * \param enable ENABLE or DISABLE
 */
void pullupPin(const uint8_t pin, const uint8_t enable);

/**
 * set interrupt for a pin
 *
 * \param pin the pin's name
 *
 * \param enable ENABLE or DISABLE
 *
 * \param edge RISING_EDGE or FALLING_EDGE, FALLING_EDGE by default
 */
void interruptPin(const uint8_t pin, const uint8_t enable, ...);

/**
 * enable button S2
 */
void enableS2();

/**
 * Initialize all ports to output mode and turn off output.
 */
void initPorts();

/**
 * initialization for analog read
 */
//void analogReadInit();
/**
 * read analog signal
 *
 * parameters:
 * reference Vcc GND, 4 x ADC10CLKs
 * reference-generator on, voltage 2,5 Volts
 * ADC10 clock divider = 1, clock source select MCLK, 1MHz
 * Sample-and-hold source select = ADC10SC bit
 * Conversion sequence mode = Single-channel-single-conversion
 *
 * \param input_channel input channel to read
 * 		A0, A1, A2, A3, A4, A5, A6, A7,
 * 		VeREFp, VREFnByVeREFn, Temperature_sensor, HalfOfVCCMinusVSS
 */
uint16_t analogRead(const inch input_channel);

/**
 * write pwm
 */
void analogWrite(timer timer, uint16_t ccr0_microsecond,
	uint16_t ccr1_microsecond);

/**
 * initialization timer, set 1MHZ
 */
void timer_init();

/**
 * create a SMCLK timer and start counting
 *
 * \param timer TIMER0 or TIMER1
 *
 * \param microsecond ccr in microsecond,from 0 to 524280
 *
 * \param mode_control STOP_MODE, UP_MODE, CONTINUOUS_MODE, UP_DOWN_MODE
 */
void timer_create(timer timer, uint32_t microsecond,
	timer_mode_control mode_control, uint8_t interrupt_enable);

/**
 * stop and delete a timer
 *
 * \param timer TIMER0 or TIMER1
 */
void timer_delete(timer timer);

/**
 * reverse the output of a pin, 1 to 0 or 0 to 1
 *
 * \param pin the pin's name
 */
void reverseOutput(const uint8_t pin);

/**
 * clear flag of interruption for a pin or a timer
 *
 * \param pin the pin's name, or TIMER0, or TIMER1
 */
void clearFlag(const uint8_t pin);

#endif // SIMPLEMSP_H_

