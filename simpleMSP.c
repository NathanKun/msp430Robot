/*
 * simpleMSP.c
 *
 *  Created on: 6 dec. 2016
 *      Author: Junyang HE
 */
#include "simpleMSP.h"
#include <stdarg.h>

const uint8_t port1Pins[] = { P10, P11, P12, P13, P14, P15, P16, P17 };
const uint8_t port2Pins[] = { P20, P21, P22, P23, P24, P25, P26, P27 };
const uint8_t pwm0Pins[] = { P11, P12, P15, P16, P26 };
const uint8_t pwm1Pins[] = { P20, P21, P22, P23, P24, P25 };

bool isPort1Pin(const uint8_t pin) {
	// in c 0 is false, any not 0 is true, so if(isPort1Pin) works
	if (pin >= P10 && pin <= P17)
		return true;
	return false;
}

bool isPort2Pin(const uint8_t pin) {
	if (pin >= P20 && pin <= P27)
		return true;
	return false;
}

bool isPwm0Pin(const uint8_t pin) {
	int i = 0;
	for (i = 0; i < 5; i++) {
		if (pin == pwm0Pins[i])
			return true;
	}
	return false;
}

bool isPwm1Pin(const uint8_t pin) {
	int i = 0;
	for (i = 0; i < 6; i++) {
		if (pin == pwm1Pins[i])
			return true;
	}
	return false;
}

uint16_t pinToBit(const uint8_t pin) {
	uint16_t bit = 0;
	switch (pin) {
	case P10:
	case P20:
		bit = BIT0;
		break;
	case P11:
	case P21:
		bit = BIT1;
		break;
	case P12:
	case P22:
		bit = BIT2;
		break;
	case P13:
	case P23:
		bit = BIT3;
		break;
	case P14:
	case P24:
		bit = BIT4;
		break;
	case P15:
	case P25:
		bit = BIT5;
		break;
	case P16:
	case P26:
		bit = BIT6;
		break;
	case P17:
	case P27:
		bit = BIT7;
		break;
	}
	return bit;
}

void directionPin(const uint8_t pin, const uint8_t direction) {
	uint16_t bit = pinToBit(pin);
	if (isPort1Pin(pin)) {
		if (direction == OUTPUT)
			P1DIR |= bit;
		else if (direction == INPUT)
			P1DIR &= ~bit;
	} else if (isPort2Pin(pin)) {
		if (direction == OUTPUT)
			P2DIR |= bit;
		else if (direction == INPUT)
			P2DIR &= ~bit;
	}
}

void digitalWrite(const uint8_t pin, const uint8_t value) {
	uint16_t bit = pinToBit(pin);
	if (isPort1Pin(pin)) {
		if (value == OUTPUT)
			P1OUT |= bit;
		else if (value == INPUT)
			P1OUT &= ~bit;
	} else if (isPort2Pin(pin)) {
		if (value == OUTPUT)
			P2OUT |= bit;
		else if (value == INPUT)
			P2OUT &= ~bit;
	}
}

int digitalRead(const uint8_t pin) {
	uint16_t bit = pinToBit(pin);
	if (isPort1Pin(pin)) {
		if ((P1IN & bit) == bit)
			return HIGH;
		else
			return LOW;
	} else if (isPort2Pin(pin)) {
		if ((P2IN & bit) == bit)
			return HIGH;
		else
			return LOW;
	}
	return LOW;
}

void selectPin(const uint8_t pin, const uint8_t mode) {
	uint16_t bit = pinToBit(pin);
	if (isPort1Pin(pin)) {
		switch (mode) {
		case 0:
			P1SEL &= ~bit;
			P1SEL2 &= ~bit;
			break;
		case 1:
			P1SEL |= bit;
			P1SEL2 &= ~bit;
			break;
		case 2:
			P1SEL &= ~bit;
			P1SEL2 |= bit;
			break;
		case 3:
			P1SEL |= bit;
			P1SEL2 |= bit;
			break;
		}
	} else if (isPort2Pin(pin)) {
		switch (mode) {
		case 0:
			P2SEL &= ~bit;
			P2SEL2 &= ~bit;
			break;
		case 1:
			P2SEL |= bit;
			P2SEL2 &= ~bit;
			break;
		case 2:
			P2SEL &= ~bit;
			P2SEL2 |= bit;
			break;
		case 3:
			P2SEL |= bit;
			P2SEL2 |= bit;
			break;
		}
	}
}

void pullupPin(const uint8_t pin, const uint8_t enable) {
	uint16_t bit = pinToBit(pin);
	if (isPort1Pin(pin)) {
		if (enable == ENABLE)
			P1REN |= bit;
		else if (enable == DISABLE)
			P1REN &= ~bit;
	} else if (isPort2Pin(pin)) {
		if (enable == ENABLE)
			P2REN |= bit;
		else if (enable == DISABLE)
			P2REN &= ~bit;
	}
}

void interruptPin(const uint8_t pin, const uint8_t enable, ...) {
	va_list valist;

	// initialize valist for num number of arguments
	va_start(valist, enable);

	bool is_enable = false;
	uint8_t edge = FALLING_EDGE;
	if (enable == ENABLE) {
		if (sizeof(valist) == 2 * sizeof(uint8_t))
			edge = va_arg(valist, uint8_t);
		else
			edge = FALLING_EDGE;
		is_enable = true;
	}

	va_end(valist);

	uint16_t bit = pinToBit(pin);
	if (isPort1Pin(pin)) {
		if (is_enable == true) {
			if (edge == RISING_EDGE)
				P1IES |= bit;
			else if (edge == FALLING_EDGE)
				P1IES &= ~bit;
			P1IE |= bit;
		} else
			P1IE &= ~bit;
		P1IFG &= ~bit;
	} else if (isPort2Pin(pin)) {
		if (is_enable == true) {
			if (edge == RISING_EDGE)
				P2IES |= bit;
			else if (edge == FALLING_EDGE)
				P2IES &= ~bit;
			P2IE |= bit;
		} else
			P2IE &= ~bit;
		P2IFG &= ~bit;
	}
}

void enableS2() {
	selectPin(P13, 0);
	directionPin(P13, INPUT);
	pullupPin(P13, ENABLE);
	digitalWrite(P13, HIGH);
}

void initPorts() {
	P1DIR = 0xFF;
	P1OUT = 0x00;
	P2DIR = 0xFF;
	P2OUT = 0x00;
	P3DIR = 0xFF;
	P3OUT = 0x00;
}

/*void analogReadInit() {
 // init to 0
 ADC10CTL0 = ADC10CTL1 = 0;

 // SREF_0: reference Vcc GND
 // ADC10SHT_0: 4 x ADC10CLKs
 // REF2_5V: reference-generator voltage 2,5 Volts
 // REFON: Reference generator on
 // ADC10ON: ADC10 on
 ADC10CTL0 = SREF_0 + ADC10SHT_0 + REF2_5V + REFON + ADC10ON;

 // ADC10DIV_0: ADC10 clock divider = 1
 // ADC10SSEL_2: ADC10 clock source select MCLK, 1MHz
 // SHS_0: Sample-and-hold source select = ADC10SC bit
 // CONSEQ_0: Conversion sequence mode select = Single-channel-single-conversion
 ADC10CTL1 = ADC10DIV_0 + ADC10SSEL_2 + SHS_0 + CONSEQ_0;
 }*/

uint16_t analogRead(const inch input_channel) {

	// input channel select
	ADC10CTL1 = (input_channel * 0x1000);
	;
	ADC10CTL0 |= ENC + ADC10SC;     // Sampling and conversion start

	while (ADC10CTL1 & ADC10BUSY)
		;	// Tant que ADC occupé on attend
	ADC10CTL0 &= ~ENC;		// Conversion finie alors Disable ADC conversion

	return ADC10MEM;	        // Return Conversion value
}

void analogWrite(timer timer, uint16_t ccr0_microsecond,
		uint16_t ccr1_microsecond) {

	uint16_t ccr0 = 0;
	uint16_t ccr1 = 0;
	uint8_t divise_by = 1;

	if (ccr0_microsecond > 0 && ccr0_microsecond <= 65535) {
		ccr0 = ccr0_microsecond;
		ccr1 = ccr1_microsecond;
		divise_by = ID_0;
	} else if (ccr0_microsecond > 65535 && ccr0_microsecond <= 131070) {
		ccr0 = ccr0_microsecond / 2;
		ccr1 = ccr1_microsecond / 2;
		divise_by = ID_1;
	} else if (ccr0_microsecond > 131070 && ccr0_microsecond <= 262140) {
		ccr0 = ccr0_microsecond / 4;
		ccr1 = ccr1_microsecond / 4;
		divise_by = ID_2;
	} else if (ccr0_microsecond > 262140 && ccr0_microsecond <= 524280) {
		ccr0 = ccr0_microsecond / 8;
		ccr1 = ccr1_microsecond / 8;
		divise_by = ID_3;
	}
	if (timer == TIMER0) {
		// for sure
		TA0CTL = 0;
		TA0CCR0 = 0;
		TA0R = 0;
		TA0CTL &= ~TAIFG; // reset flag
		TA0CTL = UP_MODE | TASSEL_2 | divise_by; // use SMCLK
		TA0CCTL1 |= OUTMOD_7;
		TA0CCR0 = ccr0;
		TA0CCR1 = ccr1;
	} else {
		TA1CTL = 0;
		TA1CCR0 = 0;
		TA1R = 0;
		TA1CTL &= ~TAIFG;
		TA1CTL = UP_MODE | TASSEL_2 | divise_by; // use SMCLK
		TA1CCTL1 |= OUTMOD_7;
		TA1CCR0 = ccr0;
		TA1CCR1 = ccr1;
	}

}

void timer_init() {
	BCSCTL1 = CALBC1_1MHZ;
	DCOCTL = CALDCO_1MHZ;
}

void timer_create(timer timer, uint32_t microsecond,
		timer_mode_control mode_control, uint8_t interrupt_enable) {
	uint16_t ccr = 0;
	uint8_t divise_by = 1;

	if (microsecond > 0 && microsecond <= 65535) {
		ccr = microsecond;
		divise_by = ID_0;
	} else if (microsecond > 65535 && microsecond <= 131070) {
		ccr = microsecond / 2;
		divise_by = ID_1;
	} else if (microsecond > 131070 && microsecond <= 262140) {
		ccr = microsecond / 4;
		divise_by = ID_2;
	} else if (microsecond > 262140 && microsecond <= 524280) {
		ccr = microsecond / 8;
		divise_by = ID_3;
	}

	if (timer == TIMER0) {
		// for sure
		TA0CTL = 0;
		TA0CCR0 = 0;
		TA0R = 0;
		TA0CTL &= ~TAIFG; // reset flag
		TA0CCR0 = ccr;
		TA0CTL = mode_control | TASSEL_2 | divise_by; // use SMCLK

		if (interrupt_enable == ENABLE)
			TA0CTL |= TAIE; // enable timer interruption
	} else if (timer == TIMER1) {
		TA1CTL = 0;
		TA1CCR0 = 0;
		TA1R = 0;
		TA1CTL &= ~TAIFG;
		TA1CCR0 = ccr;
		TA1CTL = mode_control | TASSEL_2 | divise_by; // use SMCLK

		if (interrupt_enable == ENABLE)
			TA1CTL |= TAIE; // enable timer interruption
	}

}

void timer_delete(timer timer) {
	if (timer == TIMER0) {
		TA0CTL &= ~TAIE; // disable timer interruption
		TA0CTL &= ~MC_3; // halt timer
		TA0CTL |= TACLR; // Timer_A clear
		// for sure
		TA0R = 0;
		TA0CCR0 = 0;
		TA0CCR1 = 0;
		TA0CCR2 = 0;
		TA0CTL = 0;
		TA0CCTL0 = 0;
		TA0CCTL1 = 0;
		TA0CCTL2 = 0;
	} else if (timer == TIMER1) {
		TA1CTL &= ~TAIE; // disable timer interruption
		TA1CTL &= ~MC_3; // halt timer
		TA1CTL |= TACLR; // Timer_A clear
		// for sure
		TA1R = 0;
		TA1CCR0 = 0;
		TA1CCR1 = 0;
		TA1CCR2 = 0;
		TA1CTL = 0;
		TA1CCTL0 = 0;
		TA1CCTL1 = 0;
		TA1CCTL2 = 0;
	}

}

void reverseOutput(const uint8_t pin) {
	uint16_t bit = pinToBit(pin);
	if (isPort1Pin(pin)) {
		P1OUT ^= bit;
	} else if (isPort2Pin(pin)) {
		P2OUT ^= bit;
	}
}

void clearFlag(const uint8_t pin) {
	uint16_t bit = pinToBit(pin);
	if (isPort1Pin(pin)) {
		P1IFG &= ~bit;
	} else if (isPort2Pin(pin)) {
		P2IFG &= ~bit;
	} else if (pin == TIMER0) {
		TA0CTL &= ~TAIFG;
	} else if (pin == TIMER1) {
		TA1CTL &= ~TAIFG;
	}
}

uint8_t checkFlag(const uint8_t pin) {
	uint16_t bit = pinToBit(pin);
	if (isPort1Pin(pin)) {
		if ((P1IFG & bit) == bit)
			return 1;
		else
			return 0;
	} else if (isPort2Pin(pin)) {
		if ((P2IFG & bit) == bit)
			return 1;
		else
			return 0;
	} else if (pin == TIMER0) {
		if ((TA0CTL & TAIFG) == TAIFG)
			return 1;
		else
			return 0;
	} else if (pin == TIMER1) {
		if ((TA1CTL & TAIFG) == TAIFG)
			return 1;
		else
			return 0;
	}
	return 0;
}
