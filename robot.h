/*
 * robot.h
 *
 *  Created on: 5 janv. 2017
 *      Author: j.he.12
 */

/**
 * P1.0-P1.7 Connectés au bornier vert (cf nomenclature sur CI) Entrées / Sorties
 * P 2.0 Opto-coupleur roue A 									Entrée
 * P 2.1 Sens pour le moteur A 									Sortie
 * P 2.2 PWM pour le moteur A, 250 KHz maximum 					Sortie
 * P 2.3 Opto-coupleur roue B 									Entrée
 * P 2.4 PWM pour le moteur B, 250 KHz maximum 					Sortie
 * P 2.5 Sens pour le moteur B 									Sortie
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include "simpleMSP.h"

#define CPU_F ((double)1000000)
#define delay_us(x) __delay_cycles((long)(CPU_F*(double)x/1000000.0))
#define delay_ms(x) __delay_cycles((long)(CPU_F*(double)x/1000.0))

#define MOTORAPWMPIN P22
#define MOTORBPWMPIN P24
#define MOTORADIRPIN P11
#define MOTORBDIRPIN P12

// direction enum
typedef enum {
FORWARD = HIGH, BACK = LOW
} direction;

// motor enum
typedef enum {
	motorA = 0, motorB = 1
} motor;

// opto-isoler enum
typedef enum {
	optoA = 0, optoB = 1
} opto;

void initRobot();
void setMortorSpeed(motor motor, int speed);
void setMortorDirection(motor motor, direction d);
int readOpto(opto opto);
void goForward(int speed);
void goBack(int speed);
void turnLeft(int speed);
void turnRight(int speed);







#endif /* ROBOT_H_ */
