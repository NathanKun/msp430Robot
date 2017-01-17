#include <msp430G2553.h>
#include "simpleMSP.h"
#include "robot.h"

volatile uint8_t start = 0;
volatile uint16_t optoA_count = 0;
volatile uint16_t optoB_count = 0;
volatile uint16_t opto_need = 0;
volatile uint16_t optoA_count_set = 0;

const uint8_t perimeter_mm = 135;
const uint8_t opto_count_per_round = 12;
//const float turn90_opto_need = 3.14 * 22.5 * 12 / perimeter_mm;

/*
 * main.c
 */

void main(void) {
	WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

	initRobot();
	enableS2();
	interruptPin(P13, ENABLE, FALLING_EDGE);
	__enable_interrupt();

	/*
	 turnLeft(100);
	 delay_ms(250);
	 goForward(100);
	 delay_ms(2000);
	 turnRight(100);
	 delay_ms(250);
	 goForward(100);
	 delay_ms(3000);
	 stopRobot();*/

	/*
	 setMortorDirection(motorB, FORWARD);
	 setMortorSpeed(motorB, 100);*/
	//goForward(100);
	int a, b;
	uint16_t c;
	while (1) {
		while (!start)
			;
		start = 0;
		goForwardDistance(100, 500);
		turnRight90();
		goForwardDistance(100, 400);
		turnLeft90();
		goForwardDistance(100, 1000);
		//a = optoA_count;
		//b = optoB_count;
		//c = analogRead(A0);
		//ajustGoFoward(optoA_count, optoB_count, 100);
	}
}

#pragma vector=PORT1_VECTOR
__interrupt void port1_interrupt(void) {
	if ((P1IFG & BIT3) == BIT3) {
		start = 1;
		P1IFG &= ~BIT3;
	}
	//P1OUT |= BIT0;
	//P1OUT &= ~BIT6;
}

#pragma vector=PORT2_VECTOR
__interrupt void motor_opto_count(void) {
	if ((P2IFG & BIT0) == BIT0) {
		P2IFG &= ~BIT0;
		optoA_count++;
	}
	if ((P2IFG & BIT3) == BIT3) {
		P2IFG &= ~BIT3;
		optoB_count++;
	}
	//P1OUT |= BIT0;
	//P1OUT &= ~BIT6;
}
// ta0ccr0 interrupt
#pragma vector=TIMER0_A0_VECTOR
__interrupt void my_timer() {

	//clearFlag(TIMER0);
}

void setDistance(int distance_mm) {
	opto_need = distance_mm * 12 / perimeter_mm;
	optoA_count_set = optoA_count;
}

uint8_t waitDistance() {
	while (optoA_count - optoA_count_set < opto_need)
		;
}

void goForwardDistance(int speed, int distance_mm) {
	goForward(speed);
	setDistance(distance_mm);
	waitDistance();
	stopRobot();
}

void turnLeft90() {
	turnLeft(50);
	delay_ms(520);
	stopRobot();
}

void turnRight90() {
	turnRight(50);
	delay_ms(520);
	stopRobot();
}
