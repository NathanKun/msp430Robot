#include <msp430G2553.h>
#include "simpleMSP.h"
#include "robot.h"

/*
 * main.c
 */
void main(void) {
    WDTCTL = WDTPW | WDTHOLD;	// Stop watchdog timer

    initRobot();

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
    stopRobot();

   /*
    setMortorDirection(motorB, FORWARD);
    setMortorSpeed(motorB, 100);*/

    while(1) {
    }
}
