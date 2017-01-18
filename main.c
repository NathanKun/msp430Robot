/**
 * MircoProcesseur Projet
 *
 * Sensors connection:
 * Ir-sensor left P1.1
 * Ir-sensor right P1.2
 * Ir-sensor center P1.3
 * Hold-sensor left P1.4
 * Hold-sensor right P1.5
 */

#include <msp430G2553.h>
#include "simpleMSP.h"
#include "robot.h"

#define perimeter_mm 135				/* perimeter of wheel */
#define ir_sill 540						/* sill of ir-sensor output when obstacle detected (after ADC) */
#define turn90DelayMs 610				/* time needed for turning 90 degrees, roughly because it depends on battery's voltage, floor's friction, etc..*/
#define time_out 20						/* 20 seconds time out */

volatile uint16_t optoA_count = 0;		/* optoA interrupt count */
volatile uint16_t optoB_count = 0;		/* optoB interrupt count */
volatile uint16_t opto_need = 0;		/* optoA interrupt needed for going a giving distance */
volatile uint16_t optoA_count_set = 0;	/* copy optoA_count when distance set */

volatile uint16_t timer0_count = 0;		/* timer0 interrupt count */

volatile bool paused = false;			/* robot pause if 1 */
volatile bool start = false;			/* robot start moving if 1 */

//static const uint8_t opto_count_per_round = 12;	/* opto interrupt time per round */
//static const float turn90_opto_need = 3.14 * 22.5 * 12 / perimeter_mm;		/* opto count needed for turning 90 degrees */

/**
 * set going distance
 * \distance_mm going distance in millimeter
 */
void setDistance(const int distance_mm) {
	opto_need = distance_mm * 12 / perimeter_mm;
	optoA_count_set = optoA_count;
}

/**
 * wait until distance set reach
 * stop automatically when an obstacle detected
 * stop automatically when a hole detected
 * \param speed from 0 to 100
 */
void waitDistance(const int speed) {
	while ((optoA_count - optoA_count_set < opto_need) && (timer0_count < time_out)) {  /* if distance not reached and not time outed */
		uint16_t irA, irB;
		irA = analogRead(A1);	/* read 2 ir sensors */
		irB = analogRead(A2);
		if ((irA >= ir_sill || irB >= ir_sill) && !paused) {	/* if obstacle detected and not paused*/
			if(irA >= ir_sill)	digitalWrite(P10, HIGH);		/* turn on led(s) to show obstacle(s) detected */
			else digitalWrite(P10, LOW);
			if(irB >= ir_sill)	digitalWrite(P16, HIGH);
			else digitalWrite(P16, LOW);
			stopRobot();										/* stop robot */
			delay_ms(500);										/* wait a little before reread sensors */
			paused = true;
		} else if ((irA < ir_sill && irB < ir_sill) && paused) {/* if no obstacles but paused */
			digitalWrite(P10, LOW);								/* turn off led(s) */
			digitalWrite(P16, LOW);
			delay_ms(200);
			goForward(speed);									/* go ahead */
			paused = false;
		}
	}
	// if time outed
	if(timer0_count >= time_out){
		stopRobot();
	}
}
/**
 * go forward for a given distance with a given speed
 * \param speed from 0 to 100
 * \param distance_mm distance in millimeter for going
 */
void goForwardDistance(const int speed, const int distance_mm) {
	if(timer0_count < time_out){
		setDistance(distance_mm);	/* set opto count needed */
		goForward(speed);			/* go */
		waitDistance(speed);		/* wait until distance reached, detect obstacle and timeout */
		stopRobot();				/* stop */
	}
}

/**
 * turn left 90 degrees
 */
void turnLeft90() {
	if(timer0_count < time_out) {
		turnLeft(50);
		delay_ms(turn90DelayMs);
		stopRobot();
	}
}

/**
 * turn right 90 degrees
 */
void turnRight90() {
	if(timer0_count < time_out) {
		turnRight(50);
		delay_ms(turn90DelayMs);
		stopRobot();
	}
}

/*
 * main.c
 */

void main(void) {
	WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

	initRobot();	/* initiate robot (ports and timers) */
	enableS2();		/* initialize button S2 */
	interruptPin(P13, ENABLE, FALLING_EDGE);	/* enable interrupt for P13 (button S2) */
	__enable_interrupt();						/* enable interruot macro */

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
	//int a, b;
	//uint16_t c;
	while (1) {
		if (start) {
			timer0_count = 0;
			goForwardDistance(100, 500);	/* go forward 50cm with speed 100 */
			turnRight90();					/* turn right 90 degrees */
			goForwardDistance(100, 400);	/* go forward 40cm with speed 100 */
			turnLeft90();					/* turn left 90 degrees */
			goForwardDistance(100, 1000);	/* go forward 100cm with speed 100 */
			/*turnLeft90();
			delay_ms(1000);
			turnLeft90();
			delay_ms(1000);
			turnLeft90();
			delay_ms(1000);
			turnLeft90();
			delay_ms(1000);
			turnRight90();
			delay_ms(1000);
			turnRight90();
			delay_ms(1000);
			turnRight90();
			delay_ms(1000);
			turnRight90();*/
			start = false;
		}
		//a = optoA_count;
		//b = optoB_count;
		//a = analogRead(A1);
		//b = analogRead(A2);
		//ajustGoFoward(optoA_count, optoB_count, 100);
	}
}

#pragma vector=PORT1_VECTOR
__interrupt void port1_interrupt(void) {
	if ((P1IFG & BIT3) == BIT3) {	/* press S2 to start robot */
		start = true;
		P1IFG &= ~BIT3;
	}
	//P1OUT |= BIT0;
	//P1OUT &= ~BIT6;
}

#pragma vector=PORT2_VECTOR
__interrupt void motor_opto_count(void) {
	if ((P2IFG & BIT0) == BIT0) {	/* optoA count */
		P2IFG &= ~BIT0;
		optoA_count++;
	}
	/* optoB broken */
	/*if ((P2IFG & BIT3) == BIT3) {
		P2IFG &= ~BIT3;
		optoB_count++;
	}*/
	//P1OUT |= BIT0;
	//P1OUT &= ~BIT6;
}
// ta0ccr0 interrupt
#pragma vector=TIMER0_A1_VECTOR
__interrupt void timer0_counter() {
	timer0_count++;
	TA0CTL &= ~TAIFG;
}

