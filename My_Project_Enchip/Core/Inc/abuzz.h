/*
 * abuzz.h
 *
 *  Created on: Apr 12, 2022
 *      Author: Alexander Göransson
 */

#ifndef INC_ABUZZ_H_
#define INC_ABUZZ_H_

#include "main.h"


// Set up the values in TIM2 to give a nice beep. Very long period gets set.
void abuzz_start();

// Sets the signal to be of zero length.
void abuzz_stop();


// Sets the period to be long. 2.0 seconds between pulses.
void abuzz_p_long();

// Sets the period to be short. 0.3 seconds between pulses.
void abuzz_p_short();



void abuzz_start()
{
	TIM15->PSC   = 40000 - 1;
	TIM15->ARR   = 0xFFFF;
	TIM15->CCR1  = 0x0064;
	TIM15->CNT   = 0x0000;
}



void abuzz_stop()
{
	TIM15->PSC  = 0x0000;
	TIM15->ARR  = 0xFFFF;
	TIM15->CCR1 = 0x0000;
}

void abuzz_p_long()
{
	TIM15->ARR = 0x0F90;
	TIM15->CNT = 0x0000;
}


void abuzz_p_short()
{
	TIM15->ARR = 0x0100;
	TIM15->CNT = 0x0000;
}


#endif /* INC_ABUZZ_H_ */
