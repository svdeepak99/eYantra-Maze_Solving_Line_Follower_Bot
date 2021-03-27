/*
*
* Team Id: 1508
* Author List: Gudapati Nitish, Deepak S V, Jinesh R, Himadri Poddar
* Filename: NS_Task_1_Sandbox.cpp
* Theme: Nutty Squirrel -- Specific to eYRC
* Functions: ir_array(void), line_track(void), F(void), L(void), R(void), forward_wls(unsigned char), left_turn_wls(void), right_turn_wls(void)
*			Square(void), Task_1_1(void), Task_1_2(void)
* Global Variables: line_memory, line_memory_rw, line_track_memory, obstracle_front
*
*/

#include "NS_Task_1_Sandbox.h"

/*
*	Variable name: line_memory
*	To specify preferred turn direction for 'line_track()' in '0b101' case and also set if line tracking is to be done with with 2 or 3 sensors.
*	Possible values:
*		0 - Turn left during '0b101' case in 'line_track()'
*		1 - Turn right during '0b101' case in 'line_track()'
*		2 - Specifies 'line_track()' to track line only with center-right sensors (2 sensors)
*		3 - Specifies 'line_track()' to track line only with center-left sensors (2 sensors)
*/
unsigned char line_memory;

/*
*	Variable name: line_memory_rw
*	Switch variable to allow modification of 'line_memory'
*	Possible values:
*		0 - 'line_memory' set to read-only
*		1 - 'line_memory' set to read-write
*/
unsigned char line_memory_rw = 1;

/*
*	Variable name: line_track_memory
*	Stores the previous turn value, and used to guide the bot when out of the track in 'line_track()'
*	Possible values:
*		0 - stored after left turn
*		1 - stored after right turn
*/
unsigned char line_track_memory = 0;

/*
*	Variable name: obstracle_front
*	Reduces forward distance travelled to align the bot after crossing a node if an independant line (obstracle) is in vicinity (eg. 1st node)
*	Possible values:
*		0 - no independant line in vicinity
*		1 - independant line present in vicinity
*/
unsigned char obstracle_front = 0;

#define confidence_max 20			//max value of confidence in software debouncing
#define confidence_thresh 14		//threshold for dececion in software debouncing

/*
*
* Function Name: ir_array
* Input: NONE
* Output: unsigned char -> returns a combined value corresponding to the white line sensor data (only three LSBs or least significant bits of the char is used)
* Logic: Sets a threshold value to distinguish white and black and sets 1 for black and 0 for white
*		Threshold = 180, if adc_value > 180 => 1 (black), and vice versa
* Example Call: ir_array(); //Returns one of 0(0b000), 1(0b001), 2(0b010), ....., 7(0b111).
*
*/
unsigned char ir_array(void)
{
	unsigned char left_sensor, centre_sensor, right_sensor;									//variables that store the adc values of the white line sensors
	left_sensor = ADC_Conversion(1);
	centre_sensor = ADC_Conversion(2);
	right_sensor = ADC_Conversion(3);
	return ((left_sensor > 180) * 4 + (centre_sensor > 180) * 2 + (right_sensor > 180));	//converting adc values into a single decimal number based on the binary value
}

/*
*
* Function Name: line_track
* Input: NONE
* Output: Follows the black line with either two or three sensors based on 'line_memory' value
* Logic:
*	When tracking with three sensors:
*		If, centre sensor is on line => it goes straight, left sensor on line => slight right, right sensor on line => slight left
*		If two consecutive sensors are on the line, correspondingly it goes straight
*	When tracking with two sensors: (This is done to avoid conflict at certain nodes where the line is thick on one side)
*		Based on the 'line_memory' value, it tracks using left-centre or right-centre sensors
*	When it is out of line, then it moves based on the previous movement stored in 'line_track_memory'
*	When a node is detected (all three sensors are on black) , the control exits the line track function (software debouncing is done here)
*Example Call: line_track();
*
*/
void line_track(void)
{
	int confidence = 0;								// counter variable for software debounce 
	while (1)
	{
		velocity(250, 250);
		if (ir_array() == 0b010)					// only centre sensor on black line
		{
			if (line_memory == 2)					// line track with right-centre sensors 
			{
				forward(); velocity(125, 250);		// curve left
			}
			else if (line_memory == 3)				// line track with left-centre sensors
			{
				forward(); velocity(250, 125);		// curve right
			}
			else				
				forward();							// when tracking with all sensors
		}
		else if (ir_array() == 0b001)				// only right sensor on black	
		{
			if (line_memory == 2)					// line track with right-centre sensors 
			{
				forward(); velocity(250, 125);		// curve right
			}
			else
				soft_right();
			line_track_memory = 1;					// store the movement 
		}
		else if (ir_array() == 0b100)				// only left sensor on black	
		{
			if (line_memory == 3)					// line track with left-centre sensors
			{
				forward(); velocity(125, 250);		// curve left
			}
			else
				soft_left();
			line_track_memory = 0;					// store the movement 
		}
		else if (ir_array() == 0b011)				// right-centre sensor on black	
		{
			if (line_memory == 3)					// line track with left-centre sensors
				soft_right();
			else
				forward();
			line_track_memory = 1;					// store the movement 
		}
		else if (ir_array() == 0b110)				// left-centre sensor on black	
		{
			if (line_memory == 2)					// line track with right-centre sensors 
				soft_left();
			else
				forward();
			line_track_memory = 0;					// store the movement 
		}
		else if (ir_array() == 0b101)				// rare case of left-right sensors
		{
			if (line_memory == 0)	
				soft_left();						//line_memory = 0 => if previous was movement right_turn_wls, right is given priority
			else if (line_memory == 1)
				soft_right();						//line_memory = 1 => if previous was movement left_turn_wls, left is given priority
			else if (line_memory == 2)				// line track with right-centre sensors 
			{
				left();
			}
			else if (line_memory == 3)				// line track with left-centre sensors
			{
				right();
			}
		}
		else if (ir_array() == 0)					// out of line
		{
			if (line_track_memory == 0)
				soft_left();
			else if (line_track_memory == 1)
				soft_right();
		}
		else										// node detected
		{
			stop();
			for (int i = 0; i < 10; i++)			// Confirms node detection via software debouncing
			{
				if (ir_array() == 7)				 
				{
					confidence++;
				}
				_delay_ms(1);
			}
			if (confidence > 5)
				break;								// breaks from loop (eventually function) after confirmation
			confidence = 0;
		}
		_delay_ms(1);
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
	while (node-- > 0)									// runs for 'node' number of times (number of nodes)
	{
		int confidence = 0;								// counter variable for software debounce 
		forward(); velocity(250, 250);
		while (1)
		{
			for (int i = 0; i < confidence_max; i++)	// To confirm when out of a node via software debouncing
			{
				if (ir_array() != 0b111)
					confidence++;						// Increment 'confidence' when past node 
				if (ir_array() == 0b011)				// left sensor out of line
				{
					forward(); velocity(250, 125);		// curve right
				}
				if (ir_array() == 0b110)				// right sensor out of line
				{
					forward(); velocity(125, 250);		// curve left
				}
			}
			if (confidence > confidence_thresh)
				break;									// confirms and breaks out of the loop
			_delay_ms(1);
			confidence = 0;								// resets confidence value
		}
		forward();	velocity(150, 150);
		if (obstracle_front == 1)
			_delay_ms(63);								// to align the centre of the bot close to the node and prevent false detection when there is an independant black line (obstracle) in the front (eg. first node)
		else
			_delay_ms(100);								// to align the centre of the bot on the node to ease turning
		stop();
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
	int confidence = 0;									// counter variable for software debounce 				
	for (int i = 0; i < confidence_max; i++)			// confirms if left sensor is on black line
	{
		if ((ir_array() & 0b100) == 0b100)
			confidence++;								
	}
	if (confidence > confidence_thresh) 
	{
		left();		velocity(150, 150);
		while (1)										// if left sensor is on black, turn left until it is out of the black line
		{
			confidence = 0;								// resets 'confidence' variable
			for (int i = 0; i < confidence_max; i++)	// confirms if out of line via software debouncing
			{
				if ((ir_array() & 0b100) == 0)
					confidence++;
			}
			if (confidence > confidence_thresh)
				break;									// confirms and breaks out of the loop
		}
	}
	while (1)
	{
		left();											// Turn left until left ir sensor detects a black line
		if (line_memory == 3)
			velocity(0, 100);							// soft left at critical left turns (eg. last before node)
		else 
			velocity(100, 100);	
		confidence = 0;
		for (int i = 0; i < confidence_max; i++)		// confirms if black line is detected by left sensor
		{
			if ((ir_array() & 0b100) == 0b100)
				confidence++;
		}
		if (confidence > confidence_thresh)				// confirms and breaks out of the loop
			break;
	}
	_delay_ms(20);
	stop();
	_delay_ms(100);										// to stabilize after a turn
	if(line_memory_rw)
		line_memory = 0;								//	updates 'line_memory' to set line track preference to left turn at 0b101 case
}

/*
*
* Function Name: right_turn_wls
* Input: void
* Output: void
* Logic: Uses white line sensors to turn right until black line is encountered
* Example Call: right_turn_wls(); //Turns right until black line is encountered
*
*/
void right_turn_wls(void)
{
	int confidence = 0;									// counter variable for software debounce
	for (int i = 0; i < confidence_max; i++)			// confirms if right sensor is on black line
	{
		if ((ir_array() & 0b001) == 0b001)
			confidence++;
	}
	if (confidence > confidence_thresh) 
	{
		right();	
		while (1)										// if right sensor is on black, turn right until it is out of the black line
		{
			confidence = 0;								// resets 'confidence' variable
			for (int i = 0; i < confidence_max; i++)	// confirms if out of line via software debouncing
			{
				if ((ir_array() & 1) == 0)
					confidence++;
			}
			if (confidence > confidence_thresh)
				break;									// confirms and breaks out of the loop
		}
	}
	while (1)
	{
		right();		velocity(100, 100);				// Turn right until right ir sensor detects a black line
		if (line_memory == 2)
			velocity(0, 100);							// soft right at critical right turns
		else
			velocity(100, 100);
		confidence = 0;
		for (int i = 0; i < confidence_max; i++)		// confirms if black line is detected by right sensor
		{
			if ((ir_array() & 1) == 1)
				confidence++;
		}
		if (confidence > confidence_thresh)				// confirms and breaks out of the loop
			break;
	}
	_delay_ms(20);
	stop();
	_delay_ms(100);										// to stabilize after a turn
	if(line_memory_rw)
		line_memory = 1;								//	updates 'line_memory' to set line track preference to right turn at 0b101 case
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
* Function Name: F
* Input: void
* Output: void
* Logic: Use this function to cross a node, track the line and stop at the next node
* Example Call: F();
*
*/
void F() 
{
	forward_wls(1);
	line_track();
}

/*
*
* Function Name: L
* Input: void
* Output: void
* Logic: Use this function to cross a node, then turn left and finally stop at the next node 
* Example Call: L();
*
*/
void L() 
{
	forward_wls(1);
	left_turn_wls();
	line_track();
}

/*
*
* Function Name: R
* Input: void
* Output: void
* Logic: Use this function to cross a node, then turn right and finally stop at the next node 
* Example Call: R();
*/
void R() 
{
	forward_wls(1);
	right_turn_wls();
	line_track();
}

/*
*
* Function Name: Task_1_1
* Input: void
* Output: void
* Logic: Use this function to encapsulate the Task 1.1 logic
* Example Call: Task_1_1();
*/
void Task_1_1(void)
{
	_delay_ms(10);								// Wait for the microcontroller to start running this code
	line_memory = 3;							// Set left-centre line tracking
	line_memory_rw = 0;							// Set read-only mode for 'line_memory'
	line_track();	
	obstracle_front = 1;						// Reduces distance travelled to align the bot after crossing a node to avoid false tracking of independant line present in vicinity
	R();										
	obstracle_front = 0;						// Sets distance travelled to align the bot after crossing a node to default value
	line_memory_rw = 1;							// Set read-write mode for 'line_memory'
	L();	L();	L();	R();	F();		
	line_memory = 3;							// Set left-centre line tracking
	line_memory_rw = 0;							// Set read-only mode for 'line_memory'
	F();										
	line_memory_rw = 1;							// Set read-write mode for 'line_memory'
	R();	L();	L();						
	line_memory = 3;							// Set left-centre line tracking
	line_memory_rw = 0;							// Set read-only mode for 'line_memory'
	L();										
	line_memory_rw = 1;							// Set read-write mode for 'line_memory'
	R();										
	stop();										
	_delay_ms(3000);							// Display for 3 secs after completion
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