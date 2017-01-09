/*
 * robot.c
 *
 *  Created on: 5 janv. 2017
 *      Author: j.he.12
 */
#include "robot.h"

void initRobotPort1() {
	// TODO wait for design
}

void initRobotPort2() {
	P2DIR = 0x36;
	// TODO P26, P27 unknown
}

void initMotor() {
	// for sure
	TA0CTL = 0;
	TA0CCR0 = 0;
	TA0R = 0;
	TA0CTL &= ~TAIFG; // reset flag
	TA0CTL = UP_MODE | TASSEL_2 | ID_0; // use SMCLK
	TA0CCTL1 |= OUTMOD_7;
	TA0CCR0 = 1000;
	TA0CCR1 = 0; // speed = 0

	TA1CTL = 0;
	TA1CCR0 = 0;
	TA1R = 0;
	TA1CTL &= ~TAIFG;
	TA1CTL = UP_MODE | TASSEL_2 | ID_0; // use SMCLK
	TA1CCTL1 |= OUTMOD_7;
	TA1CCR0 = 1000;
	TA1CCR1 = 0;

	selectPin(MOTORAPWMPIN, 1);
	selectPin(MOTORBPWMPIN, 1);
}

void initRobot() {
	initPorts();
	initRobotPort1();
	initRobotPort2();
	initMotor();
}

void setMortorSpeed(motor motor, int speed) {
	if (motor == motorA) {
		TA0CCR1 = 10 * speed;
	} else if (motor == motorB) {
		TA1CCR1 = 10 * speed;
	}
}

void setMortorDirection(motor motor, direction d) {
	if (motor == motorA) {
		if(d == FORWARD) {
			digitalWrite(MOTORADIRPIN, HIGH);
		} else if (d == BACK) {
			digitalWrite(MOTORADIRPIN, LOW);
		}
	} else if (motor == motorB) {
		if(d == FORWARD) {
			digitalWrite(MOTORBDIRPIN, HIGH);
		} else if (d == BACK) {
			digitalWrite(MOTORBDIRPIN, LOW);
		}
	}
}

int readOpto(opto opto) {
	if (opto == optoA)
		return digitalRead(P20);
	else if (opto == optoB)
		return digitalRead(P23);
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
