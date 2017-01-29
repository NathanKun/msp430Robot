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

#define PERIMETER_MM 135				/* perimeter of wheel */
#define IR_SILL 550						/* sill of ir-sensor output when obstacle detected (after ADC) */
#define TURN_90_DELAY_MS 610				/* time needed for turning 90 degrees, roughly because it depends on battery's voltage, floor's friction, etc..*/
#define TIME_OUT 20						/* 20 seconds time out */

#define IRAPIN A4
#define IRBPIN A5
#define IRCPIN P17
#define HOLDSENSORPIN P12

volatile uint16_t optoA_count = 0; /* optoA interrupt count */
volatile uint16_t optoB_count = 0; /* optoB interrupt count */
volatile uint16_t opto_need = 0; /* optoA interrupt needed for going a giving distance */
volatile uint16_t optoA_count_set = 0; /* copy optoA_count when distance set */

volatile uint16_t timer0_count = 0; /* timer0 interrupt count */

volatile bool paused = false; /* robot pause if 1 */
volatile bool start = false; /* robot start moving if 1 */

//static const uint8_t opto_count_per_round = 12;	/* opto interrupt time per round */
//static const float turn90_opto_need = 3.14 * 22.5 * 12 / perimeter_mm;		/* opto count needed for turning 90 degrees */

/**
 * set going distance
 * \distance_mm going distance in millimeter
 */
void setDistance(const int distance_mm) {
	opto_need = distance_mm * 12 / PERIMETER_MM;
	optoA_count_set = optoA_count;
}

/**
 * wait until distance set reach
 * stop automatically when an obstacle detected
 * stop automatically when a hole detected
 * \param speed from 0 to 100
 */
void waitDistance(const int speed) {
	uint16_t irA = 0, irB = 0;
	while ((optoA_count - optoA_count_set < opto_need)
			&& (timer0_count < TIME_OUT)) { /* if distance not reached and not time outed */
		irA = analogRead(IRAPIN); /* read 2 ir sensors */
		irB = analogRead(IRBPIN);
		if ((irA >= IR_SILL || irB >= IR_SILL
				|| digitalRead(HOLDSENSORPIN) == HIGH) && !paused) { /* if obstacle detected and not paused*/
			if (irA >= IR_SILL)
				digitalWrite(P10, HIGH); /* turn on led(s) to show obstacle(s) detected */
			else
				digitalWrite(P10, LOW);
			if (irB >= IR_SILL)
				digitalWrite(P16, HIGH);
			else
				digitalWrite(P16, LOW);
			stopRobot(); /* stop robot */
			delay_ms(500); /* wait a little before reread sensors */
			paused = true;
		} else if ((irA < IR_SILL && irB < IR_SILL)
				&& (digitalRead(HOLDSENSORPIN) == LOW) && paused) {/* if no obstacles but paused */
			digitalWrite(P10, LOW); /* turn off led(s) */
			digitalWrite(P16, LOW);
			delay_ms(150);
			goForward(speed); /* go ahead */
			paused = false;
		}
	}
	// if time outed
	if (timer0_count >= TIME_OUT) {
		stopRobot();
	}
}


void waitDistanceBreak(const int speed) {
	uint16_t irA = 0, irB = 0;
	while ((optoA_count - optoA_count_set < opto_need)
			&& (timer0_count < TIME_OUT)) { /* if distance not reached and not time outed */
		irA = analogRead(IRAPIN); /* read 2 ir sensors */
		irB = analogRead(IRBPIN);
		if ((irA >= IR_SILL || irB >= IR_SILL
				|| digitalRead(HOLDSENSORPIN) == HIGH) && !paused) { /* if obstacle detected and not paused*/
			if (irA >= IR_SILL)
				digitalWrite(P10, HIGH); /* turn on led(s) to show obstacle(s) detected */
			else
				digitalWrite(P10, LOW);
			if (irB >= IR_SILL)
				digitalWrite(P16, HIGH);
			else
				digitalWrite(P16, LOW);
			stopRobot(); /* stop robot */
			delay_ms(500); /* wait a little before reread sensors */
			paused = true;
			break;
		}
	}
	// if time outed
	if (timer0_count >= TIME_OUT) {
		stopRobot();
	}
}

/**
 * go forward for a given distance with a given speed
 * \param speed from 0 to 100
 * \param distance_mm distance in millimeter for going
 */
void goDistance(const int speed, const int distance_mm, const direction d) {
	if (timer0_count < TIME_OUT) {
		setDistance(distance_mm); /* set opto count needed */
		if (d == FORWARD)
			goForward(speed); /* go */
		else
			goBack(speed);
		waitDistance(speed); /* wait until distance reached, detect obstacle and timeout */
		stopRobot(); /* stop */
	}
}

void goDistanceBreak(const int speed, const int distance_mm, const direction d) {
	if (timer0_count < TIME_OUT) {
		setDistance(distance_mm); /* set opto count needed */
		if (d == FORWARD)
			goForward(speed); /* go */
		else
			goBack(speed);
		waitDistanceBreak(speed); /* wait until distance reached, detect obstacle and timeout */
		stopRobot(); /* stop */
	}
}

/**
 * turn left 90 degrees
 */
void turnLeft90() {
	if (timer0_count < TIME_OUT) {
		turnLeft(50);
		delay_ms(TURN_90_DELAY_MS);
		stopRobot();
	}
}

/**
 * turn right 90 degrees
 */
void turnRight90() {
	if (timer0_count < TIME_OUT) {
		turnRight(50);
		delay_ms(TURN_90_DELAY_MS);
		stopRobot();
	}
}

/*
 void turnByFacingDirection(turnDirection td, direction d) {
 if(d == FORWARD){
 if(td == left)
 turnLeft90();
 else
 turnRight90();
 } else {
 if(td == left)
 turnRight90();
 else
 turnLeft90();
 }
 }*/

void goUntilObstacle(const int speed, const direction d);
uint8_t goUntilLeftNoObstacleOr30cm(const int speed, const direction d);
void avoidObstacle(const int speed, direction d);

void goUntilObstacle(const int speed, const direction d) {
	digitalWrite(P10, LOW);
	digitalWrite(P16, HIGH);

	uint16_t irA = 0, irB = 0;
	goForward(speed);

	while (timer0_count < TIME_OUT) { /* if distance not reached and not time outed */
		irA = analogRead(IRAPIN); /* read 2 ir sensors */
		irB = analogRead(IRBPIN);
		if (irA >= IR_SILL || irB >= IR_SILL) { /* if obstacle*/
			stopRobot(); // stop robot
			/*delay_ms(500); // wait a little before reread sensors
			 paused = true;*/
			break;
		}
	}
	// if time outed
	if (timer0_count >= TIME_OUT) {
		stopRobot();
	}
	avoidObstacle(speed, d);
}

uint8_t goUntilLeftNoObstacleOr30cm(const int speed, const direction d) {
	uint16_t irA, irB;
	digitalWrite(P10, HIGH);
	digitalWrite(P16, HIGH);
	delay_ms(300);
	setDistance(300);
	if (d == FORWARD)
		goForward(speed);
	else
		goBack(speed);
	while ((timer0_count < TIME_OUT)
			&& (optoA_count - optoA_count_set < opto_need)) { /* if distance not reached and not time outed */

		irA = analogRead(IRAPIN);  // read 2 ir sensors
		irB = analogRead(IRBPIN);
		if (irA >= IR_SILL || irB >= IR_SILL) { // if obstacle detected
			break;
		}
		//irC = analogRead(IRCPIN);
		//if (irC < IR_SILL) {
		if (digitalRead(IRCPIN) == HIGH) { // LOW for obstacle detected, HIGH for no obstacle detected
			if (d == FORWARD){ // IRC at the front of the robot, so go 15cm more
				goDistanceBreak(speed, 150, FORWARD);
				optoA_count_set -= 14; //150 * 12 / PERIMETER_MM
			}
			return optoA_count - optoA_count_set; // obstacle disappeared, means it is really an obstacle, return opto_counted
		}
	}
	// if time outed
	if (timer0_count >= TIME_OUT) {
		stopRobot();
	}

	return 0; // obstacle still there for 30cm, means it is probably a wall
}

void avoidObstacle(const int speed, direction d) {
	if (timer0_count < TIME_OUT) {
		digitalWrite(P10, HIGH);
		digitalWrite(P16, LOW);
		turnRight90();
		uint8_t returnVal = goUntilLeftNoObstacleOr30cm(speed, d);
		if (returnVal == 0) { // it was a wall
			if (d == FORWARD)
				d = BACK;
			else
				d = FORWARD;
			goDistanceBreak(speed, 150, d);
			turnRight90();
		} else { // it was an obstacle, try go around and avoid it
			turnLeft90();
			uint16_t irA, irB;

			/*		go and detect for 40 cm
			 if obstacle
			 aboard and treat like a wall
			 if no obstacle
			 turn  left
			 go counted opto-count and detect
			 if obstacle
			 aboard
			 turn right and go ahead normal
			 if no obstacle
			 turn right and go ahead normal
			 */

			// go forward and detect for 40 cm
			setDistance(400);
			bool obstacleDetected = false;
			goForward(speed);
			while ((timer0_count < TIME_OUT)
					&& (optoA_count - optoA_count_set < opto_need)) {

				irA = analogRead(IRAPIN);
				irB = analogRead(IRBPIN);
				if (irA >= IR_SILL || irB >= IR_SILL
						|| digitalRead(HOLDSENSORPIN) == HIGH) { // if obstacle met, aboard and treat like a wall
					if (returnVal == 0) {
						if (d == FORWARD)
							d = BACK;
						else
							d = FORWARD;
					}
					obstacleDetected = true;
					break;
				}
			}
			stopRobot();
			if (!obstacleDetected) { // if no obstacle
				turnLeft90();
				setDistance(returnVal / 12 * PERIMETER_MM); // go counted opto-count's distance

				if (d == FORWARD)
					goForward(speed);
				else
					goBack(speed);

				while ((timer0_count < TIME_OUT)
						&& (optoA_count - optoA_count_set < opto_need)) {
					irA = analogRead(IRAPIN);  // read 2 ir sensors
					irB = analogRead(IRBPIN);
					if (irA >= IR_SILL || irB >= IR_SILL
							|| digitalRead(HOLDSENSORPIN) == HIGH) { // if obstacle detected, stop going back
						break;
					}
				}
				stopRobot();
				turnRight90();
			} else {
				turnLeft90();
				turnLeft90();
			}
		}
		goUntilObstacle(speed, d);
	}
}

/*
 * main.c
 */

void main(void) {
	WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

	initRobot(); /* initiate robot (ports and timers) */
	enableS2(); /* initialize button S2 */
	interruptPin(P13, ENABLE, FALLING_EDGE); /* enable interrupt for P13 (button S2) */
	__enable_interrupt(); /* enable interruot macro */

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
//			goDistance(100, 500);	/* go forward 50cm with speed 100 */
//			turnRight90();					/* turn right 90 degrees */
//			goDistance(100, 400);	/* go forward 40cm with speed 100 */
//			turnLeft90();					/* turn left 90 degrees */
//			goDistance(100, 1000);	/* go forward 100cm with speed 100 */
			goUntilObstacle(100, FORWARD);
			start = false;
		}
		//a = optoA_count;
		//b = optoB_count;
		//a = analogRead(A1);
		//b = analogRead(A2);
		//ajustGoFoward(optoA_count, optoB_count, 100);
		if (digitalRead(HOLDSENSORPIN) == HIGH)
			digitalWrite(P16, HIGH);
		else
			digitalWrite(P16, LOW);
	}
}

#pragma vector=PORT1_VECTOR
__interrupt void port1_interrupt(void) {
	if ((P1IFG & BIT3) == BIT3) { /* press S2 to start robot */
		start = true;
		P1IFG &= ~BIT3;
	}
	//P1OUT |= BIT0;
	//P1OUT &= ~BIT6;
}

#pragma vector=PORT2_VECTOR
__interrupt void motor_opto_count(void) {
	if ((P2IFG & BIT0) == BIT0) { /* optoA count */
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

