/*
 * robot.c
 *
 *  Created on: 5 janv. 2017
 *      Author: Junyang HE
 */
#include "robot.h"

void initADC() {
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
}

void initRobotPort1() {
	// TODO wait for design
	P1DIR &= ~(BIT1 | BIT2 | BIT3 | BIT4 | BIT5 | BIT7);
	P1DIR |= (BIT0 | BIT6);
	P1REN |= (BIT1 | BIT2 | BIT3 | BIT4 | BIT5);
}

void initRobotPort2() {
	P2DIR = 0x36; // P2.1 2 4 5 out, p2.0 3 in
	P2SEL = 0x14; // PWM: P2.2 P2.4
	P2IE |= (BIT0); //opto: P2.0 p2.3, p2.3 dammaged
	// TODO P26, P27 unknown
}

void initMotor() {
	TA1CTL = 0;
	TA1CCR0 = 0;
	TA1R = 0;
	TA1CTL &= ~TAIFG;

	TA1CTL = UP_MODE | TASSEL_2 | ID_0; // use SMCLK
	TA1CCTL1 |= OUTMOD_7;
	TA1CCTL2 |= OUTMOD_7;

	TA1CCR0 = 10000;
	TA1CCR1 = 0;
	TA1CCR2 = 0;
}

void initTimer0() {
	TA0CTL = 0;
	TA0CCR0 = 0;
	TA0R = 0;
	TA0CTL &= ~TAIFG;

	TA0CTL = UP_DOWN_MODE | TASSEL_2 | ID_3 | TAIE; // use SMCLK
	TA0CCR0 = 62500;
}

void initRobot() {
	initPorts();
	initRobotPort1();
	initRobotPort2();
	timer_init();
	initTimer0();
	initMotor();
	initADC();
}

void setMotorSpeed(motor motor, int speed) {
	if (motor == motorA) {
		TA1CCR1 = 100 * speed * 0.98;
	} else if (motor == motorB) {
		TA1CCR2 = 100 * speed;
	}
}

void setMotorDirection(motor motor, direction d) {
	if (motor == motorA) {
		if (d == FORWARD) {
			digitalWrite(MOTORADIRPIN, LOW);
		} else if (d == BACK) {
			digitalWrite(MOTORADIRPIN, HIGH);
		}
	} else if (motor == motorB) {
		if (d == FORWARD) {
			digitalWrite(MOTORBDIRPIN, LOW);
		} else if (d == BACK) {
			digitalWrite(MOTORBDIRPIN, HIGH);
		}
	}
}

int readOpto(opto opto) {
	if (opto == optoA)
		return digitalRead(MOTORAOPTOPIN);
	else if (opto == optoB)
		return digitalRead(MOTORBOPTOPIN);
	return 0;
}

void goForward(int speed) {
	setMotorDirection(motorA, FORWARD);
	setMotorDirection(motorB, FORWARD);
	setMotorSpeed(motorA, speed);
	setMotorSpeed(motorB, speed);
}

void goBack(int speed) {
	setMotorDirection(motorA, BACK);
	setMotorDirection(motorB, BACK);
	setMotorSpeed(motorA, speed);
	setMotorSpeed(motorB, speed);
}

void turnLeft(int speed) {
	setMotorDirection(motorA, BACK);
	setMotorDirection(motorB, FORWARD);
	setMotorSpeed(motorA, speed);
	setMotorSpeed(motorB, speed);
}

void turnRight(int speed) {
	setMotorDirection(motorA, FORWARD);
	setMotorDirection(motorB, BACK);
	setMotorSpeed(motorA, speed);
	setMotorSpeed(motorB, speed);
}

void goForwardTurn(int speedA, int speedB) {
	setMotorDirection(motorA, FORWARD);
	setMotorDirection(motorB, FORWARD);
	setMotorSpeed(motorA, speedA);
	setMotorSpeed(motorB, speedB);
}

void goBackTurn(int speedA, int speedB) {
	setMotorDirection(motorA, BACK);
	setMotorDirection(motorB, BACK);
	setMotorSpeed(motorA, speedA);
	setMotorSpeed(motorB, speedB);
}

void stopRobot() {
	int i = 100;
	for (i = TA1CCR1 / 100; i >= 0; i = i - 2){
		setMotorSpeed(motorA, i);
		setMotorSpeed(motorB, i);
	}
	setMotorSpeed(motorA, 0);
	setMotorSpeed(motorB, 0);
}

void adjustGoFoward(uint16_t a, uint16_t b, uint8_t speed) {
	uint16_t ccr = 100 * speed;
	if (a - b >= 2) {
		if (TA1CCR2 <= ccr * 0.95)
			TA1CCR2 = ccr;
		else if (TA1CCR1 == ccr)
			TA1CCR1 = ccr * 0.9;
		//P1OUT |= BIT0;
		//P1OUT &= ~BIT6;
	} else if (b - a >= 2) {
		if (TA1CCR1 <= ccr * 0.95)
			TA1CCR1 = ccr;
		else if (TA1CCR2 == ccr)
			TA1CCR2 = ccr * 0.9;
		//P1OUT |= BIT6;
		//P1OUT &= ~BIT0;
	}
}
