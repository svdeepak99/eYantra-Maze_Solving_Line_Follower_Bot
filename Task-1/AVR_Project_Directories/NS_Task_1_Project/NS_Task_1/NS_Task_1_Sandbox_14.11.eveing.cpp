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

//variable to store the digital value of the whiteline sensor
unsigned char line_sensor, line_memory;

#define confidence_max 20   //max value of confidence in soft debouncing
#define confidence_thresh 14  //threshold for dececion in soft debouncing
//int x = 0;

/*
*
* Function Name: ir_array
* Input: NONE 
* Output: unsigned char -> returns a combined value corresponding to the white line sensor
* Logic: Sets a threshold value to distinguish white and black and sets 1 for black and 0 for white
*		Threshold = 180, if adc_value > 180 => 1 (black), and vice versa
* Example Call: ir_array(); //Returns one of 100, 010, 001, 110, etc.
*
*/
unsigned char ir_array (void)
{
	left_sensor = ADC_Conversion(1);
	centre_sensor = ADC_Conversion(2);
	right_sensor = ADC_Conversion(3);
	line_sensor=(left_sensor>180)*4+(centre_sensor>180)*2+(right_sensor>180);
	//printf("\n %03d", line_sensor);
	return line_sensor;
}

/*
*
* Function Name: line_track
* Input: NONE
* Output: Follows the black line
* Logic: Sets a threshold value to distinguish white and black and sets 1 for black and 0 for white
*		Threshold = 180, if adc_value > 180 => 1 (black), and vice versa
* Example Call: ir_array(); //Returns one of 100, 010, 001, 110, etc.
*
*/
void line_track(void)
{
	int confidence=0;
//	if (x == 1)
//		printf("\nline_track: %d", ir_array());
	while (1)
	{
	//	velocity(100, 100);
		if (ir_array() == 0b010)
		{
			forward();
		}
		else if (ir_array() == 0b001)
		{
			soft_right();
		//	line_memory = 1;
		}
		else if (ir_array() == 0b100)
		{
			soft_left();
		//	line_memory = 2;
		}
		else if (ir_array() == 0b011)
		{
			soft_right();
		//	line_memory = 1;
		}
		else if (ir_array() == 0b110)
		{
			soft_left();
		//	line_memory = 2;
		}
		else if (ir_array() == 0b101)
		{
			if (line_memory == 1)
				soft_right();
			else
				soft_left();
		}
		else
		{
			for (int i = 0;i < 100;i++) {
				if (ir_array() == 0 || (ADC_Conversion(1) + ADC_Conversion(2) + ADC_Conversion(3)) > 600 && ADC_Conversion(2) > 249)
					confidence++;
				_delay_ms(1);
			}
			if (confidence > 80)
				break;
			confidence = 0;
		}
		velocity(150, 150);
	}
}

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
		/*left_sensor = ADC_Conversion(1);
		centre_sensor = ADC_Conversion(2);
		right_sensor = ADC_Conversion(3);
		printf("\n %d, %d, %d", left_sensor, centre_sensor, right_sensor);*/
		//ir_array();
		//left_turn_wls();
		line_track();
	}
}
/*
*
* Function Name: left_turn_wls
* Input: void
* Output: void
* Logic: Uses white line sensors to turn left until black line is encountered
* Example Call: left_turn_wls(); //Turns left until black line is encountered
*
*/
void left_turn_wls(void)
{
	int confidence = 0;
	for (int i = 0;i < confidence_max;i++) {
		if ((ir_array() & 4) == 4)
			confidence++;
	//	_delay_ms(1);
	}
	if (confidence > confidence_thresh) {
		left();		velocity(150, 150);		//Turn left if left ir sensor is on a black line till it is out
		while (1) {
			confidence = 0;
			for (int i = 0;i < confidence_max;i++) {
				if ((ir_array() & 4) == 0)
					confidence++;
			//	_delay_ms(1);
			}
			if (confidence > confidence_thresh)
				break;
		}
	}
	while (1)
	{								//Turn left until left ir sensor detects a black line

		left();		velocity(150, 150);
		confidence = 0;
		for (int i = 0;i < confidence_max;i++) {
			if ((ir_array() & 4) == 4)
				confidence++;
		//	_delay_ms(1);
		}
		if (confidence > confidence_thresh)
			break;
	}
	_delay_ms(20);
	stop();
	_delay_ms(100);
	line_memory = 2;			// stores in memory that last turn was left
}

/*
*
* Function Name: right_turn_wls
* Input: void
* Output: void
* Logic: Uses white line sensors to turn right until black line is encountered
* Example Call: right_turn_wls(); //Turns right until black line is encountered
*/
/*void right_turn_wls(void)
{
	while (ir_array() & 4 == 4)
		right();	velocity(100, 100);	//Turn right if right ir sensor is on a black line till it is out
	while (!(ir_array()&1))
	{									//Turn right until right ir sensor detects a black line
		right();	velocity(100, 100);
		_delay_ms(10);
	}
	stop();
	line_memory = 1;
}*/
void right_turn_wls(void)
{
	int confidence = 0;
	for (int i = 0;i < confidence_max;i++) {
		if ((ir_array() & 1) == 1)
			confidence++;
	//	_delay_ms(1);
	}
	if (confidence > confidence_thresh) {
		right();		velocity(100, 100);		//Turn right if right ir sensor is on a black line till it is out
		while (1) {
			confidence = 0;
			for (int i = 0;i < confidence_max;i++) {
				if ((ir_array() & 1) == 0)
					confidence++;
			//	_delay_ms(1);
			}
			if (confidence > confidence_thresh)
				break;
		}
	}
	while (1)
	{								//Turn right until right ir sensor detects a black line

		right();		velocity(100, 100);
		confidence = 0;
		for (int i = 0;i < confidence_max;i++) {
			if ((ir_array() & 1) == 1)
				confidence++;
		//	_delay_ms(1);
		}
		if (confidence > confidence_thresh)
			break;
	}
	_delay_ms(20);
	stop();
	_delay_ms(100);
	line_memory = 1;			// stores in memory that last turn was right
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

void node_cross(void)
{
	int confidence = 0;
	forward();	velocity(100, 100);
	_delay_ms(200);
	forward(); velocity(50, 50);
	while (1)
	{
		for (int i = 0;i < confidence_max;i++) {
			if ((ADC_Conversion(1)+ ADC_Conversion(2)+ ADC_Conversion(3))< 500 && ir_array()!=5 ) {
	//			stop();
				confidence++;
			}
		//	_delay_ms(1);
		}
		if (confidence > confidence_thresh) {
			break;
	//	forward(); velocity(50, 50);
		}
		confidence = 0;
	}
	//forward(); velocity(100, 100);
	_delay_ms(200);
	stop();
}

void F() {
	node_cross();
	line_track();
}
void L() {
	node_cross();
	printf("\n%d", ir_array());
	left_turn_wls();
	printf("\n%d", ir_array());
	line_track();
	printf("\n%d\n", ir_array());
}
void R() {
	node_cross();
	printf("\n%d", ir_array());
	right_turn_wls();
	printf("\n%d", ir_array());
	line_track();
	printf("\n%d\n", ir_array());
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
		line_track();
		R();
		L();
/*		printf("\nNode cross:%d", ir_array());
		node_cross();
		printf("\nLeft_turn: %d", ir_array());
		left_turn_wls();
		printf("\nLine_Track_main: %d", ir_array());
		x = 1; 
//		left_turn_wls();
//		line_track();
		left();
		_delay_ms(400);
		line_track(); */
		L();
		L();
		R();
		F();
		F();
		R();
		L();
		L();
		L();
		R();
		stop();
		_delay_ms(30000);
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
