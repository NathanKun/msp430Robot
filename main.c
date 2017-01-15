#include <msp430G2553.h>
#include "simpleMSP.h"
#include "robot.h"

volatile uint16_t distance_total_cm = 0;
volatile uint16_t distance_last_cm = 0;
volatile uint16_t optoA_count = 0;
volatile uint16_t optoB_count = 0;
volatile uint16_t opto_count_target = 65535;
const uint8_t perimeter_mm = 35;
const uint8_t opto_count_per_round = 12;

/*
 * main.c
 */

void main(void) {
	WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer


	initRobot();
	__enable_interrupt();
/*
	goForward(100);
	delay_ms(3000);
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
	//goForward(50);
	int a,b;
	while (1) {
		a = optoA_count;
		b = optoB_count;
		//ajustGoFoward(optoA_count, optoB_count, 50);
	}
}

#pragma vector=PORT2_VECTOR
__interrupt void motor_opto_count(void) {
	if ((P2IFG & BIT0) == BIT0) {
		P2IFG &= ~BIT0;
		optoA_count++;
		if(optoA_count>=12){
			optoA_count = 0;
			setMortorSpeed(motorB, 100);
			delay_ms(250);
			setMortorSpeed(motorB, 0);
		}
	}
	if ((P2IFG & BIT3) == BIT3) {
		P2IFG &= ~BIT3;
		optoB_count++;
		if(optoB_count>=12){
			optoB_count = 0;
			setMortorSpeed(motorA, 100);
			delay_ms(250);
			setMortorSpeed(motorA, 0);
		}
	}
}

// ta0ccr0 interrupt
#pragma vector=TIMER0_A0_VECTOR
__interrupt void my_timer() {

	//clearFlag(TIMER0);
}
