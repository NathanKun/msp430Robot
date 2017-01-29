/*
 * robot.h
 *
 *  Created on: 5 janv. 2017
 *      Author: Junyang HE
 */


#ifndef ROBOT_H_
#define ROBOT_H_

#include "simpleMSP.h"

#define CPU_F ((double)1000000) /* CPU clock frequency */
#define delay_us(x) __delay_cycles((long)(CPU_F*(double)x/1000000.0)) /* Delay microsecond */
#define delay_ms(x) __delay_cycles((long)(CPU_F*(double)x/1000.0)) /* Delay millisecond */

/**
 * P1.0-P1.7 Connect�s au bornier vert (cf nomenclature sur CI) Entrees / Sorties
 * P 2.0 Opto-coupleur roue A 									Entree
 * P 2.1 Sens pour le moteur A 									Sortie
 * P 2.2 PWM pour le moteur A, 250 KHz maximum 					Sortie
 * P 2.3 Opto-coupleur roue B 									Entree
 * P 2.4 PWM pour le moteur B, 250 KHz maximum 					Sortie
 * P 2.5 Sens pour le moteur B 									Sortie
 */
#define MOTORAPWMPIN P22
#define MOTORBPWMPIN P24
#define MOTORADIRPIN P21
#define MOTORBDIRPIN P25
#define MOTORAOPTOPIN P20
#define MOTORBOPTOPIN P23

// direction enum
typedef enum {
FORWARD = HIGH, BACK = LOW
} direction;

// direction enum
typedef enum {
left = 0, right = 1
} turnDirection;

// motor enum
typedef enum {
	motorA = 0, motorB = 1
} motor;

// opto-isoler enum
typedef enum {
	optoA = 0, optoB = 1
} opto;

/**
 * initialize robot
 */
void initRobot();

/**
 * set motor speed
 * \param motor motorA or motorB
 * \param speed motor's turning speed(duty cycle for PWM), from 0 to 100
 */
void setMotorSpeed(motor motor, int speed);

/**
 * set motor's turning direction
 * \param motor motorA or motorB
 * \param direction turning direction, FORWARD or BACK
 */
void setMotorDirection(motor motor, direction d);

/**
 * robot goes forward
 * \param speed motor's turning speed(duty cycle for PWM), from 0 to 100
 */
void goForward(int speed);

/**
 * robot goes back
 * \param speed motor's turning speed(duty cycle for PWM), from 0 to 100
 */
void goBack(int speed);

/**
 * robot turns left around the center of the robot
 * \param speed motor's turning speed(duty cycle for PWM), from 0 to 100
 */
void turnLeft(int speed);

/**
 * robot turns right around the center of the robot
 * \param speed motor's turning speed(duty cycle for PWM), from 0 to 100
 */
void turnRight(int speed);

/**
 * robot goes forward and turns by varying the relative rate of rotation of its wheels
 * \param speedA motorA's turning speed(duty cycle for PWM), from 0 to 100
 * \param speedB motorB's turning speed(duty cycle for PWM), from 0 to 100
 */
void goForwardTurn(int speedA, int speedB);

/**
 * robot goes back and turns by varying the relative rate of rotation of its wheels
 * \param speedA motorA's turning speed(duty cycle for PWM), from 0 to 100
 * \param speedB motorB's turning speed(duty cycle for PWM), from 0 to 100
 */
void goBackTurn(int speedA, int speedB);

/**
 * robot stops
 */
void stopRobot();

/**
 * robot go forward and adjust it direction for going straightly
 * \param a optoA count
 * \param a optoB count
 * \param speed motor's turning speed(duty cycle for PWM), from 0 to 100
 */
void adjustGoFoward(uint16_t a, uint16_t b, uint8_t speed);

#endif /* ROBOT_H_ */
