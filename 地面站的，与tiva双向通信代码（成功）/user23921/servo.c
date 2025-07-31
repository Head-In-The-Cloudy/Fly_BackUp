#include "servo.h"

#include "stdio.h"


void baffle_respond(void)
{
	printf("{#255P1500T0010!}");
	HAL_Delay(500);
	printf("{#255P2500T0010!}");
	HAL_Delay(500);	
}








