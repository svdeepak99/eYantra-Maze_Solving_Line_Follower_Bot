/*
*
* Team Id: 1508
* Author List: Gudapati Nitish, Deeepak S V, Jinesh R, Himadri Poddar
* Filename: NS_Task_1_Sandbox.cpp
* Theme: Nutty Squirrel -- Specific to eYRC
* Functions: forward_wls(unsigned char), left_turn_wls(void), right_turn_wls(void)
*			Square(void), Task_1_1(void), Task_1_2(void)
* Global Variables: left_sensor, centre_sensor, right_sensor
*
*/

#include "NS_Task_1_Sandbox.h"

//variables that store the adc values of the white line sensors
unsigned char left_sensor, centre_sensor, right_sensor;

/*
*
* Function Name: ir_array
* Input: left_sensor, centre_sensor, right_sensor 
* Output: unsigned char -> returns a combined value corresponding to the white line sensor
* Logic: Sets a threshold value to distinguish white and black and sets 1 for black and 0 for white
*		
* Example Call: forward_wls(2); //Goes forward by two nodes
*
*/
/*
unsigned char ir_array (unsigned char left_sensor, unsigned char centre_sensor, unsigned char right_sensor)
{
	left_sensor = ADC_Conversion(1);
	centre_sensor = ADC_Conversion(2);
	right_sensor = ADC_Conversion(3);
	return 1;
}*/

/*
*
* Function Name: forward_wls
* Input: node
* Output: void
* Logic: Uses white line sensors to go forward by the number of nodes specified
* Example Call: forward_wls(2); //Goes forward by two nodes
*
*/
void forward_wls(unsigned char node)
{

	while (1)
	{
		left_sensor = ADC_Conversion(1);
		centre_sensor = ADC_Conversion(2);
		right_sensor = ADC_Conversion(3);
		printf("\n %d, %d, %d", left_sensor, centre_sensor, right_sensor);
	}
}
/*
*
* Function Name: left_turn_wls
* Input: void
* Output: void
* Logic: Uses white line sensors to turn left until black line is encountered
* Example Call: left_turn_wls(); //Turns right until black line is encountered
*
*/
void left_turn_wls(void)
{

}

/*
*
* Function Name: right_turn_wls
* Input: void
* Output: void
* Logic: Uses white line sensors to turn right until black line is encountered
* Example Call: right_turn_wls(); //Turns right until black line is encountered
*/
void right_turn_wls(void)
{

}

/*
*
* Function Name: Square
* Input: void
* Output: void
* Logic: Use this function to make the robot trace a square path on the arena
* Example Call: Square();
*/
void Square(void)
{
	forward();	velocity(230, 230);		//moves forward for 1 sec with ~90% dutycycle
	_delay_ms(1000);
	right();	velocity(230, 230);		//turns 90 deg right
	_delay_ms(500);
	forward();	velocity(230, 230);		//moves forward for 1 sec with ~90% dutycycle
	_delay_ms(1000);
	right();	velocity(230, 230);		//turns 90 deg right
	_delay_ms(500);
	forward();	velocity(230, 230);		//moves forward for 1 sec with ~90% dutycycle
	_delay_ms(1000);
	right();	velocity(230, 230);		//turns 90 deg right
	_delay_ms(500);
	forward();	velocity(230, 230);		//moves forward for 1 sec with ~90% dutycycle
	_delay_ms(1000);
	right();	velocity(230, 230);		//turns 90 deg right
	_delay_ms(500);
}


/*
*
* Function Name: Task_1_1
* Input: void
* Output: void
* Logic: Use this function to encapsulate your Task 1.1 logic
* Example Call: Task_1_1();
*/
void Task_1_1(void)
{

}

/*
*
* Function Name: Task_1_2
* Input: void
* Output: void
* Logic: Use this function to encapsulate your Task 1.2 logic
* Example Call: Task_1_2();
*/
void Task_1_2(void)
{

}
