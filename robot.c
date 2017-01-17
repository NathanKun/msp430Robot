/*
 * robot.c
 *
 *  Created on: 5 janv. 2017
 *      Author: j.he.12
 */
#include "robot.h"

void initRobotPort1() {
	// TODO wait for design
	P1DIR &= ~(BIT0 | BIT1 | BIT2 | BIT3 | BIT4);
}

void initRobotPort2() {
	P2DIR = 0x36;
	P2SEL = 0x14; // PWM: P2.2 P2.4
	//P2IE |= 0x09; // opto: P2.0 p2.3
	P2IE |= (BIT0 | BIT3);
	P2IES &= ~(BIT0 | BIT3); // opto: P2.0 p2.3
	//P2REN |= 0x09;
	// TODO P26, P27 unknown
}

void initMotor() {
	// for sure
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
	BCSCTL1 = CALBC1_1MHZ;
	DCOCTL = CALDCO_1MHZ;
	TA0CTL = 0;
	TA0CCR0 = 0;
	TA0R = 0;
	TA0CTL &= ~TAIFG;

	TA0CTL = UP_MODE | TASSEL_2 | ID_1 | TAIE; // use SMCLK
	TA0CCR0 = 50000;
}

void initRobot() {
	initPorts();
	initRobotPort1();
	initRobotPort2();
	initMotor();
}

void setMortorSpeed(motor motor, int speed) {
	if (motor == motorA) {
		TA1CCR1 = 100 * speed * 0.98;
	} else if (motor == motorB) {
		TA1CCR2 = 100 * speed;
	}
}

void setMortorDirection(motor motor, direction d) {
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
	setMortorDirection(motorA, FORWARD);
	setMortorDirection(motorB, FORWARD);
	setMortorSpeed(motorA, speed);
	setMortorSpeed(motorB, speed);
}

void goBack(int speed) {
	setMortorDirection(motorA, BACK);
	setMortorDirection(motorB, BACK);
	setMortorSpeed(motorA, speed);
	setMortorSpeed(motorB, speed);
}

void turnLeft(int speed) {
	setMortorDirection(motorA, BACK);
	setMortorDirection(motorB, FORWARD);
	setMortorSpeed(motorA, speed);
	setMortorSpeed(motorB, speed);
}

void turnRight(int speed) {
	setMortorDirection(motorA, FORWARD);
	setMortorDirection(motorB, BACK);
	setMortorSpeed(motorA, speed);
	setMortorSpeed(motorB, speed);
}

void goForwardTurn(int speedA, int speedB) {
	setMortorDirection(motorA, FORWARD);
	setMortorDirection(motorB, FORWARD);
	setMortorSpeed(motorA, speedA);
	setMortorSpeed(motorB, speedB);
}

void goBackTurn(int speedA, int speedB) {
	setMortorDirection(motorA, BACK);
	setMortorDirection(motorB, BACK);
	setMortorSpeed(motorA, speedA);
	setMortorSpeed(motorB, speedB);
}

void stopRobot() {
	setMortorSpeed(motorA, 0);
	setMortorSpeed(motorB, 0);
}

void ajustGoFoward(uint16_t a, uint16_t b, uint8_t speed) {
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
