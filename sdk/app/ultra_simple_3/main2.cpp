#include "motor_hat.h"
#include <wiringPiSPI.h>
#include <mcp3004.h>
#include <iostream>

#define BASE 100
#define SPI_CHAN 0

using namespace motor_hat;

int main(int argc, char *argv[])
{
	//int count = 0;
	//int x = 0;
	//wiringPiSetup() ;
  	//mcp3004Setup (BASE, SPI_CHAN) ; // 3004 and 3008 are the same 4/8 channels

	motor_hat::motor_hat mh;
	mh.set_speed(2, 100, 1);
	mh.set_speed(0,130, 1);
	delay(1500);
  	//while(count < 1000)
	//{
    	//	x = analogRead (BASE);
	//	std::cout << x << std::endl;
	//	//delay(500);
	//	count++;
	//}	
	mh.set_speed(2, 100, 1);
	mh.set_speed(0, 130, -1);
	delay(800);
	mh.set_speed(2, 100, 1);
	mh.set_speed(0,130, 1);
	delay(500);
	mh.set_speed(2, 0, 1);
	mh.set_speed(0, 0, 1);
	
	
}