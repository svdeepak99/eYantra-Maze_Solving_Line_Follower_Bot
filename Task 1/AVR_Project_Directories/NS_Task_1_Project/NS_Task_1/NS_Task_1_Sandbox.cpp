/*
*
* Team Id: 1508
* Author List: Gudapati Nitish, Deepak S V, Jinesh R, Himadri Poddar
* Filename: NS_Task_1_Sandbox.cpp
* Theme: Nutty Squirrel -- Specific to eYRC
* Functions: ir_array(void), obstracle(int), line_track(void), forward_wls(unsigned char), left_turn_wls(int), right_turn_wls(int), Square(void), F(void), L(int), R(int), filter_color(), pick_nut(), place_nut(), generate_path(char, char), orient(int), Task_1_1(void), Task_1_2(void);
* Global Variables: line_memory, line_memory_rw, line_track_memory, obstracle_front, iteration, maze[][][], path[], prev_node, curr_node, next_node, target_node, obstruction, obs_new_dir, nut_color, nuts_picked, occupied[]
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

/*	
*
*	Counts the number of iterations in line_track();
*		+1 for foward movement
*		+0.75 for curve turn
*		+0.5 for soft turn
*		+0 for spot turn
*
*/
float iteration = 0;

//	Initializing default values for North, East, South, West
enum direction { N, E, S, W };
/*
*
*	Variable name: maze
*	Array that stores complete node details of Task 1.2 track and can be used by Dijikstra`s algorithm for path planning
*
*/
//							0		1		2		3		4		5		6		7		8		9		10		11		12		13		14		15		16		17		18		19		20		21		22		23
int maze[24][24][2] = { {	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{46,N },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},	//0
						{	{-1,-1},{-1,-1},{-1,-1},{88,N },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},	//1
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{88,N },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},	//2
						{	{-1,-1},{88,S },{-1,-1},{-1,-1},{91,E },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{240,N},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},	//3
						{	{-1,-1},{-1,-1},{-1,-1},{91,W },{-1,-1},{53,E },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},	//4
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{53,W },{-1,-1},{420,S},{-1,-1},{-1,-1},{-1,-1},{103,N},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},	//5
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{420,S},{-1,-1},{53,E },{-1,-1},{-1,-1},{-1,-1},{103,N},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},	//6
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{53,W },{-1,-1},{91,E },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},	//7
						{	{-1,-1},{-1,-1},{88,S },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{91,W },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{240,N},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},	//8
						{	{46,S },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{67,N },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},	//9
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{103,W},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{107,E},{-1,-1},{-1,-1},{223,N},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},	//10
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{103,E},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{107,W},{-1,-1},{-1,-1},{-1,-1},{223,N},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},	//11
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{67,S },{107,W},{107,E},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{290,N},{-1,-1},{-1,-1},{-1,-1}},	//12
						{	{-1,-1},{-1,-1},{-1,-1},{240,S},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{126,E},{-1,-1},{248,N},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},	//13
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{240,S},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{126,W},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{248,N}},	//14
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{223,S},{-1,-1},{-1,-1},{126,W},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{132,N},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},	//15
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{223,S},{-1,-1},{-1,-1},{126,E},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{132,N},{-1,-1}},	//16
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{248,S},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{92,E },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},	//17
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{132,S},{-1,-1},{92,W },{-1,-1},{92,E },{-1,-1},{-1,-1},{-1,-1},{-1,-1}},	//18
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{92,W },{-1,-1},{92,E },{-1,-1},{-1,-1},{-1,-1}},	//19
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{290,S},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{92,W },{-1,-1},{92,E },{-1,-1},{-1,-1}},	//20
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{92,W },{-1,-1},{92,E },{-1,-1}},	//21
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{132,S},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{92,W },{-1,-1},{92,E }},	//22
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{248,S},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{92,W },{-1,-1}},	//23
					  };

/*
*	Variable name: path
*	Array to store node no.s of final path
*/
char path[24];

/*
*	Variable name: prev_node
*	Stores the previous travelled node number
*/
char prev_node = 0;

/*
*	Variable name: curr_node
*	Stores the current node number
*/
char curr_node = 0;

/*
*	Variable name: next_node
*	Stores the next node number to be travelled
*/
char next_node = 0;

// Stores the destination node of current path
char target_node = 0;

// Variables of communication between obstracle() and orient function() to signify node detection and bot facing direction (0 - no change , 1 - 180 deg turn)
char obstruction = 0 , obs_new_dir = 0;

// colour of the nut picked
int nut_color;

// number of nuts picked
int nuts_picked = 0;

// tell the occupancy status of the drop locations
unsigned char occupied[4] = { 0,0,0,0 };

#define confidence_max 20			//max value of confidence in software debouncing
#define confidence_thresh 14		//threshold for dececion in software debouncing

// The colours are given specific numerical codes
#define clear 0
#define red 1
#define green 2
#define brown 3

// Function definitions
unsigned char ir_array(void);
void obstracle(int);
void line_track(void);
void forward_wls(unsigned char);
void left_turn_wls(int);
void right_turn_wls(int);
void Square(void);
void F();
void L(int);
void R(int);
int filter_color();
int pick_nut();
void place_nut();
void generate_path(char, char);
void orient(int);
void Task_1_1(void);
void Task_1_2(void);

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
* Function Name: obstrale
* Input: int condition
* Output: void
* Logic:
*	Reroutes to previous node when obstracle is detected in path
*	Cases:
*		0 - Clearance available, take 180 degree turn
*		1 - No proper clearance, go back till safe for spot turn
*		2 - No clearance, spot turn
*Example Call: obstracle(1);
*
*/
void obstracle(int condition)
{
	stop();
	obstruction = 1;
	if (!condition)
	{
		L(1);									// Take 180 degree turn and line track till previous node
		_delay_ms(100);
		obs_new_dir = 1;						//  Made 180 degree turn
	}
	else if (condition == 2)
	{
		_delay_ms(100);
		obs_new_dir = 0;						// Made no turn
	}
	else
	{	
		back();		velocity(250, 250);			// go back
		_delay_ms((iteration*15)/2);			// delay until the node is reached
		stop();
		_delay_ms(100);
		obs_new_dir = 0;						// MAde no turn
	}
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
	int confidence = 0, obs_confidence = 0;								// counter variable for software debounce 
	iteration = 0;														// to estimate approximate distance travelled since start of function
	int ir_local = -1;													// local variable for ir_array()
	while (1)
	{
		if (ADC_Conversion(4) < 60)										// Obstracle detected
		{
			stop();
			obs_confidence = 0;
			for (int i = 0; i < 10; i++)								// Confirms obstracle detection via software debouncing
			{
				if (ADC_Conversion(4) < 90)
				{
					obs_confidence++;
				}
				_delay_ms(1);
			}
			if (obs_confidence > 5)										// If obstracle present, reroute the bot to previous node
			{
				if (iteration > 120)									// Good clearance for 180 degree turn
				{
					obstracle(0);
					break;
				}
				else if (iteration < 10)								// No clearance, safe for on spot turn
				{
					obstracle(2);
					break;
				}
				else
				{														// No proper clearance, travel back till safe for on spot turn
					obstracle(1);
					break;
				}
			}
		}
		_delay_ms(1);
		ir_local = ir_array();											// local variable for ir_array storage
		velocity(250, 250);
		if (ir_local == 0b010)											// only centre sensor on black line
		{
			if (line_memory == 2)										// line track with right-centre sensors 
			{
				forward(); velocity(125, 250);							// curve left
				iteration += 0.75;
			}
			else if (line_memory == 3)									// line track with left-centre sensors
			{
				forward(); velocity(250, 125);							// curve right
				iteration += 0.75;
			}
			else
			{
				forward();												// when tracking with all sensors
				iteration += 1;
			}
		}
		else if (ir_local == 0b001)										// only right sensor on black	
		{
			if (line_memory == 2)										// line track with right-centre sensors 
			{
				forward(); velocity(250, 125);							// curve right
				iteration += 0.75;
			}
			else
			{
				soft_right();
				iteration += 0.5;
			}
			line_track_memory = 1;										// store the movement 
		}
		else if (ir_array() == 0b100)									// only left sensor on black	
		{
			if (line_memory == 3)										// line track with left-centre sensors
			{
				forward(); velocity(125, 250);							// curve left
				iteration += 0.75;
			}
			else
			{
				soft_left();
				iteration += 0.5;
			}
			line_track_memory = 0;										// store the movement 
		}
		else if (ir_local == 0b011)										// right-centre sensor on black	
		{
			if (line_memory == 3)										// line track with left-centre sensors
			{
				soft_right();
				iteration += 0.5;
			}
			else
			{
				forward();
				iteration += 1;
			}
			line_track_memory = 1;										// store the movement 
		}
		else if (ir_local == 0b110)										// left-centre sensor on black	
		{
			if (line_memory == 2)										// line track with right-centre sensors 
			{
				soft_left();
				iteration += 0.5;
			}
			else
			{
				forward();
				iteration += 1;
			}
			line_track_memory = 0;										// store the movement 
		}
		else if (ir_local == 0b101)										// rare case of left-right sensors
		{
			if (line_memory == 0)
			{
				soft_left();											//line_memory = 0 => if previous was movement right_turn_wls, right is given priority
				iteration += 0.5;
			}
			else if (line_memory == 1)
			{
				soft_right();											//line_memory = 1 => if previous was movement left_turn_wls, left is given priority
				iteration += 0.5;
			}
			else if (line_memory == 2)									// line track with right-centre sensors 
			{
				left();
			}
			else if (line_memory == 3)									// line track with left-centre sensors
			{
				right();
			}
		}
		else if (ir_local == 0)											// out of line
		{
			if (line_track_memory == 0)
			{
				soft_left();
				iteration += 0.5;
			}
			else if (line_track_memory == 1)
			{
				soft_right();
				iteration += 0.5;
			}
		}
		else															
		{																// node detected
			stop();
			for (int i = 0; i < 10; i++)								// Confirms node detection via software debouncing
			{
				if (ir_array() == 7)
				{
					confidence++;
				}
				_delay_ms(1);
			}
			if (confidence > 5)
				break;													// breaks from loop (eventually function) after confirmation
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
		if (ir_array() == 0b111)
		{
			forward();	velocity(250, 250);
			if (obstracle_front == 1)
				_delay_ms(120);							// to align the centre of the bot close to the node and prevent false detection when there is an independant black line (obstracle) in the front (eg. first node)
			else
				_delay_ms(230);
			stop();
			_delay_ms(100);
			stop();
		}
	}
}

/*
*
* Function Name: left_turn_wls
* Input: int
* Output: void
* Logic: Uses white line sensors to turn left until black line is encountered, for specified no. of times as parameter
* Example Call: left_turn_wls(1); //Turns left once until black line is encountered
*
*/
void left_turn_wls(int count)
{
	while (count-- > 0)
	{
		int confidence = 0;									// counter variable for software debounce
		left();		velocity(250, 250);
		_delay_ms(150);
		for (int i = 0; i < confidence_max; i++)			// confirms if left sensor is on black line
		{
			if ((ir_array() & 0b100) == 0b100)
				confidence++;
		}
		if (confidence > confidence_thresh)
		{
			left();		velocity(250, 250);
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
				velocity(0, 250);							// soft left at critical left turns (eg. last before node)
			else
				velocity(250, 250);
			confidence = 0;
			for (int i = 0; i < confidence_max; i++)		// confirms if black line is detected by left sensor
			{
				if ((ir_array() & 0b100) == 0b100)
					confidence++;
			}
			if (confidence > confidence_thresh)				// confirms and breaks out of the loop
				break;
		}
		stop();
		_delay_ms(100);										// to stabilize after a turn
		if (line_memory_rw)
			line_memory = 0;								//	updates 'line_memory' to set line track preference to left turn at 0b101 case
	}
}

/*
*
* Function Name: right_turn_wls
* Input: int
* Output: void
* Logic: Uses white line sensors to turn right until black line is encountered, for specified no. of times as parameter
* Example Call: right_turn_wls(1); //Turns right once until black line is encountered
*
*/
void right_turn_wls(int count)
{
	while (count-- > 0)
	{
		int confidence = 0;									// counter variable for software debounce
		right();		velocity(250, 250);
		_delay_ms(150);
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
			right();		velocity(250, 250);				// Turn right until right ir sensor detects a black line
			if (line_memory == 2)
				velocity(0, 250);							// soft right at critical right turns
			else
				velocity(250, 250);
			confidence = 0;
			for (int i = 0; i < confidence_max; i++)		// confirms if black line is detected by right sensor
			{
				if ((ir_array() & 1) == 1)
					confidence++;
			}
			if (confidence > confidence_thresh)				// confirms and breaks out of the loop
				break;
		}
		stop();
		_delay_ms(100);										// to stabilize after a turn
		if (line_memory_rw)
			line_memory = 1;								//	updates 'line_memory' to set line track preference to right turn at 0b101 case
	}
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
* Logic: Use this function to cross a node, then turns left for specified no. of times and finally stop at the next node
* Example Call: L();
*
*/
void L(int count)
{
	forward_wls(1);
	left_turn_wls(count);
	line_track();
}

/*
*
* Function Name: R
* Input: int
* Output: void
* Logic: Use this function to cross a node, then turns right for specified no. of times and finally stop at the next node
* Example Call: R();
*
*/
void R(int count)
{
	forward_wls(1);
	right_turn_wls(count);
	line_track();
}

/*
*
* Function Name: filter_color
* Input: void
* Output: int
* Logic: The function filters the colour and returns the corresponding colour code for the colour of the block
* Example Call: filter_color();
*
*/
int filter_color()
{
	int r = 0, g = 0, b = 0;														// variables to store r, g, b values
	filter_red();
	r = color_sensor_pulse_count;													// gets r value
	filter_green();
	g = color_sensor_pulse_count;													// gets g value
	filter_blue();
	b = color_sensor_pulse_count;													// gets b value
	if (r > 3000 && g < 1000 && b < 1000)
		return red;																	// determines if red
	else if (r < 1000 && g>3000 && b < 1500)
		return green;																// determines if green
	else if (r > 2000 && r < 4000 && g > 1000 && g < 3000 && b > 500 && b < 2000)
		return brown;																// determines if brown
	return clear;																	// determines if none of the above colours
}

/*
*
* Function Name: pick_nut
* Input: void
* Output: int
* Logic: The function pick the nut after checking if it is a nut
* Example Call: pick_nut();
*
*/
int pick_nut()
{
	int confidence,i_c;																		// for software debouncing
	forward_wls(1);
	if (maze[curr_node][prev_node][1] == E)													// if approach from East, turn right
	{
		right();	velocity(150, 150);
		_delay_ms(100);
		right_turn_wls(1);
	}
	else if (maze[curr_node][prev_node][1] == W)											// if approach from West, turn right
	{
		left();		velocity(150, 150);
		_delay_ms(100);
		left_turn_wls(1);
	}
	_delay_ms(100);
	stop();

	// picks red nut
	confidence = 0;
	for (i_c = 0; i_c < 10; i_c++)															// confirms if red is detected
	{
		if (filter_color() == red)
			confidence++;
		_delay_ms(1);
	}
	if (confidence > 5) 
	{
		pick();
		return red;
	}

	//picks green nut
	confidence = 0;
	for (i_c = 0; i_c < 10; i_c++)															// confirms if green is detected
	{
		if (filter_color() == green)
			confidence++;
		_delay_ms(1);
	}
	if (confidence > 5) 
	{
		pick();
		return green;
	}

	//does not pick obstracle
	confidence = 0;
	for (i_c = 0; i_c < 10; i_c++) 
	{
		if (filter_color() == brown)
			confidence++;
		_delay_ms(1);
	}
	if (confidence > 5) 
	{
		return brown;
	}
	
	// no object to pick
	return clear;
}

/*
*
* Function Name: place_nut
* Input: void
* Output: void
* Logic: The function place the nut in the location
* Example Call: place_nut();
*
*/
void place_nut()
{
	forward_wls(1);									// cross the node
	if (maze[curr_node][prev_node][1] == W)			// If approach from West, turn right
	{
		right_turn_wls(1);
	}
	else if (maze[curr_node][prev_node][1] == E)	// If approach from East, turn left
	{
		left_turn_wls(1);
	}stop();
	place();										// places the nut in the location
	nuts_picked += 1;								// Increment number of nuts picked
}

/*
* Function Name: generate_path
* Input: char,char
* Output: void
* Logic: Generates the shortest/fastest path using Djikstra`s algorithm between the inputed start and end nodes and stores nodes in path[]
* Example Call: generate_path(2,13);
*/
void generate_path(char start_node, char end_node)
{
	// 1D array that stores details of the priority_node (node to be branched) to be used in Dijikstra`s algorithm
	int priority_node[3];

	// Acts as priority queue in Dijikstra`s algorithm and stores details of corresponding nodes
	int priority_queue[24][3];

	// Holds the stack of previous priority nodes and helps in creation of final path
	int node_pile[24][3];

	// index in for loop for priority queue, priority queue node, node pool and temporary swap variables
	int i_pq = 0, swap_pq[3], pq_node_repeat, i_np = 0, i_path = 1, swap_path;

	// update target node
	target_node = end_node;

	// priority_node = start_node
	priority_node[0] = start_node;	priority_node[1] = 0;	priority_node[2] = 0;

	do 
	{
		if ((start_node == 9 && end_node == 0) || start_node == end_node)								// Dead End Cases elimination
		{
			priority_node[1] = start_node;																// update priority node for initial case, and when start and end nodes are same
			break;
		}

		for (int i = 0; i < 24; i++)																	// Creating/adding to priority_queue
		{
			if (maze[priority_node[0]][i][0] != -1 && i != priority_node[1])							
			{
				pq_node_repeat = 0;
				for (int j = 0; j < i_pq; j++)
					if (priority_queue[j][0] == i) 
					{
						pq_node_repeat = 1;
						if (priority_node[2] + maze[priority_node[0]][i][0] < priority_queue[j][2])		// If node number of a branch of priority_node is same as a node in priority_queue, replace the node with lesser weight 
						{
							priority_queue[j][2] = priority_node[2] + maze[priority_node[0]][i][0];
							priority_queue[j][1] = priority_node[0];
						}
						break;
					}
				for (int j = 0; j < i_np; j++)
					if (node_pile[j][0] == i) 
					{																					// If branch node already present in node_pile don`t add it to priority_queue
						pq_node_repeat = 1;
						break;
					}
				if (pq_node_repeat == 0) 
				{																						// Adding of all (excluding above cases) branch nodes of priority_node to priority_queue
					priority_queue[i_pq][0] = i;	
					priority_queue[i_pq][1] = priority_node[0];		
					priority_queue[i_pq][2] = priority_node[2] + maze[priority_node[0]][i][0];
					i_pq += 1;
				}
			}
		}
		priority_queue[i_pq][0] = -1;																	// Terminating priority queue with '-1' value

		for (int i = 0; i < i_pq; i++)																	// Sorting priority_queue
		{
			for (int j = i + 1; j < i_pq; j++) 
			{
				if (priority_queue[j][2] < priority_queue[i][2]) 
				{
					for (int k = 0; k < 3; k++) 
					{
						swap_pq[k] = priority_queue[i][k];
						priority_queue[i][k] = priority_queue[j][k];
						priority_queue[j][k] = swap_pq[k];
					}
				}
			}
		}

		for (int i = 0; i < 3; i++)																		// Add priority_node to node_pile
		{
			node_pile[i_np][i] = priority_node[i];
		}
		i_np += 1;

		for (int i = 0; i < 3; i++)																		// Update priority_node
		{
			priority_node[i] = priority_queue[0][i];
		}
		
		for (int i = 1; i < 24 && priority_queue[i][0] != -1; i++)										// Move priority_queue by 1 step
			for (int k = 0; k < 3; k++)
			{
				priority_queue[i - 1][k] = priority_queue[i][k];
			}
		priority_queue[--i_pq][0] = -1;
	} while (priority_node[0] != end_node);																// Djikstra`s Algorithm stops only when the end node reaches the top of the priority queue

	// Creation of path matrix by backtracking
	path[0] = end_node;
	path[1] = priority_node[1];

	while (1)
	{
		if (path[1] == start_node)
			break;																						// break if the start node is same as end node
		if (node_pile[--i_np][0] == path[i_path])
		{
			path[++i_path] = node_pile[i_np][1];
			if (path[i_path] == start_node)
				break;
		}
	}
	
	for (int i = 0; i <= i_path / 2; i++)																// Reversing path matrix to ascending order of path
	{
		swap_path = path[i];
		path[i] = path[i_path - i];
		path[i_path - i] = swap_path;
	}
	path[i_path + 1] = -1;																				// set end of path as -1 // terminating case of path

	if (path[0] == path[1])																				// if first and second nodes are same in path, remove the second node
	{
		path[1] = -1;
	}
}

/*
*
* Function Name: orient(int);
* Input: int apr_dir // approach direction
* Output: void
* Logic: Use this function to move to the destination node through the generated path using the approach direction
*		The navigation is based on the approach direction and target direction
*		Cases:
*			turns right in case of N-W, W-S, S-E, E-N (APPROACH DIRECTION - TARGET DIRECTION)
*			turns left in case of N-E, E-S, S-W, W-N (APPROACH DIRECTION - TARGET DIRECTION)
*			moves forward in case of N-S, S-N, E-W, W-E (APPROACH DIRECTION - TARGET DIRECTION)
*			case of N-N, S-S, E-E, W-W (APPROACH DIRECTION - TARGET DIRECTION)
*				if no node at right, turn right once
*				if no node at left, turn left once
*				if node present at both left and right,
*					N-N and E-E : turn right twice
*					S-S and W-W : turn left twice
* Example Call: Task_1_1();
*
*/
void orient(int apr_dir)
{
	do
	{
		obstruction = 0;																	// no obstruction
		char sd_complete, sd_else;															// flags for same direction case
		for (int index = 0; path[index + 1] != -1; index++)									// traverse until generated path is complete
		{
			curr_node = path[index];														// update current node
			if (index)
				prev_node = path[index - 1];												// update previous node
			next_node = path[index + 1];													// update next node

			// target direction
			int tar_dir;

			if (!(apr_dir != -1 && index == 0))
				apr_dir = maze[curr_node][prev_node][1];									// update approach direction
			
			tar_dir = maze[curr_node][next_node][1];										// update target direction
			
			if (apr_dir - tar_dir == 1 || apr_dir - tar_dir == -3)							// turns right in case of N-W, W-S, S-E, E-N (APPROACH DIRECTION - TARGET DIRECTION)
			{
				R(1);
			}
			else if (apr_dir - tar_dir == -1 || apr_dir - tar_dir == 3)						// turns left in case of N-E, E-S, S-W, W-N (APPROACH DIRECTION - TARGET DIRECTION)
			{
				L(1);
			}
			else if (apr_dir - tar_dir == 2 || apr_dir - tar_dir == -2)						// moves forward in case of N-S, S-N, E-W, W-E (APPROACH DIRECTION - TARGET DIRECTION)
			{
				F();
			}
			else
			{																				// case of N-N, S-S, E-E, W-W (APPROACH DIRECTION - TARGET DIRECTION)
				if (apr_dir == N)															// N-N Case
				{
					sd_complete = 0; sd_else = 1;											// flag update for same direction // eg if a node to right or left of current facing is present
					for (int j = 0; j < 24; j++)
						if (maze[curr_node][j][1] == W)										// node present in West							
						{							
							sd_complete = 1;	
							break;
						}
					if (!sd_complete)														// no node present in West
					{
						R(1); sd_else = 0;													// turn right once
					}
					if (sd_complete) 
					{
						sd_complete = 0;
						for (int j = 0; j < 24; j++)
							if (maze[curr_node][j][1] == E)									// node present in East
							{
								sd_complete = 1;
								break;
							}
						if (!sd_complete)													// no node present in East
						{
							L(1); sd_else = 0;												// turn left once
						}
					}
					if (sd_else)
						R(2);																// turn right twice if node present in both East and West 
				}
				else if (apr_dir == E)														// E-E Case
				{
					sd_complete = 0; sd_else = 1;
					for (int j = 0; j < 24; j++)
						if (maze[curr_node][j][1] == N)										// node present in North
						{
							sd_complete = 1;
							break;
						}
					if (!sd_complete) 
					{
						R(1); sd_else = 0;													// turn right once
					}
					if (sd_complete) 
					{
						sd_complete = 0;
						for (int j = 0; j < 24; j++)
							if (maze[curr_node][j][1] == S)									// node present in South
							{
								sd_complete = 1;
								break;
							}
						if (!sd_complete)													// no node present in South
						{
							L(1); sd_else = 0;												// turn left once
						}
					}
					if (sd_else)
						R(2);																// turn right twice if node present in both East and West 
				}
				else if (apr_dir == W)														// W-W Case
				{
					sd_complete = 0; sd_else = 1;
					for (int j = 0; j < 24; j++)
						if (maze[curr_node][j][1] == N)										// node present in North
						{
							sd_complete = 1;
							break;
						}
					if (!sd_complete)														// no node present in North
					{
						L(1); sd_else = 0;													// turn left once
					}
					if (sd_complete) 
					{
						sd_complete = 0;
						for (int j = 0; j < 24; j++)
							if (maze[curr_node][j][1] == S)									// node present in South
							{
								sd_complete = 1;
								break;
							}
						if (!sd_complete)													// no node present in South
						{
							R(1); sd_else = 0;												// turn right once
						}
					}
					if (sd_else)
						L(2);																// turn left twice if node present in both East and West 
				}
				else
				{																			// S-S Case
					sd_complete = 0; sd_else = 1;
					for (int j = 0; j < 24; j++)
						if (maze[curr_node][j][1] == W)										// node present in West
						{
							sd_complete = 1;
							break;
						}
					if (!sd_complete)														// no node present in West
					{
						L(1); sd_else = 0;													// turn left once
					}
					if (sd_complete) 
					{
						sd_complete = 0;
						for (int j = 0; j < 24; j++)
							if (maze[curr_node][j][1] == E)									// node present in East
							{
								sd_complete = 1;
								break;
							}
						if (!sd_complete)													// no node present in East
						{
							R(1); sd_else = 0;												// turn right once
						}
					}
					if (sd_else)
						L(2);																// turn left twice if node present in both East and West 
				}
			}
			if (obstruction)																// if obstruction present
			{
				maze[curr_node][next_node][0] = -1;	maze[curr_node][next_node][1] = -1;		// block path due to obstruction
				maze[next_node][curr_node][0] = -1;	maze[next_node][curr_node][1] = -1;		// block path due to obstruction
				generate_path(curr_node, target_node);										// generate path
				if (obs_new_dir) 
				{
					apr_dir = tar_dir;														// update approach direction after 180 degree turn
				}
				else 
				{
					apr_dir = tar_dir + 2;													// update approach direction after no turn
					apr_dir %= 4;
				}
				break;																		// break from the loop 
			}
		}
	}while (obstruction);																	// repeats until obstruction is present

	if (path[1] != -1)																		// if end of path, update previous and current
	{
		prev_node = curr_node;
		curr_node = next_node;
	}
}

/*
*
* Function Name: Task_1_1
* Input: void
* Output: void
* Logic: Use this function to encapsulate the Task 1.1 logic
* Example Call: Task_1_1();
*
*/
void Task_1_1(void)
{
	_delay_ms(10);								// Wait for the microcontroller to start running this code
	line_memory = 3;							// Set left-centre line tracking
	line_memory_rw = 0;							// Set read-only mode for 'line_memory'
	line_track();
	obstracle_front = 1;						// Reduces distance travelled to align the bot after crossing a node to avoid false tracking of independant line present in vicinity
	R(1);
	obstracle_front = 0;						// Sets distance travelled to align the bot after crossing a node to default value
	line_memory_rw = 1;							// Set read-write mode for 'line_memory'
	L(1);	L(1);	L(1);	R(1);	F();
	line_memory = 3;							// Set left-centre line tracking
	line_memory_rw = 0;							// Set read-only mode for 'line_memory'
	F();
	line_memory_rw = 1;							// Set read-write mode for 'line_memory'
	R(1);	L(1);	L(1);
	line_memory = 3;							// Set left-centre line tracking
	line_memory_rw = 0;							// Set read-only mode for 'line_memory'
	L(1);
	line_memory_rw = 1;							// Set read-write mode for 'line_memory'
	R(1);
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

	_delay_ms(500);
	stop();														// Wait for the microcontroller to respond

	// switch variable to check if placed
	int toggle = 0;

	line_track();
	curr_node = 9;
	prev_node = 0;

	for (int i = 17; i <= 23; i++)								// Pick-up zone nodes traversal
	{
		if (i == 20)
			continue;											// Skip pickup from node 20 due to absence of pickup location
		generate_path(curr_node, i);							// genreates node path from current node to pickup zone node
		if (!toggle)
			orient(-1);											// traverse to throught he path towards target direction
		else
			orient(N);											// traverse to throught he path towards north
		toggle = 0;
		nut_color = pick_nut();
		if (nut_color == red)
		{
			if (!occupied[0])
			{
				generate_path(i, 1);							// genreates node path from current node to red pickup zone node
				occupied[0] = 1;								// updates occupancy status
			}
			else
			{
				generate_path(i, 4);							// genreates node path from current node to red pickup zone node
				occupied[1] = 1;								// updates occupancy status
			}
			orient(S);											// traverse to throught he path towards south
			place_nut();
			toggle = 1;
		}
		else if (nut_color == green)
		{
			if (!occupied[2])
			{
				generate_path(i, 7);							// genreates node path from current node to green pickup zone node
				occupied[2] = 1;								// updates occupancy status
			}
			else
			{
				generate_path(i, 2);							// genreates node path from current node to green pickup zone node
				occupied[3] = 1;								// updates occupancy status
			}
			orient(S);											// traverse to throught he path towards south
			place_nut();
			toggle = 1;
		}
		else if (nut_color == clear)
		{
			if (i == 23)
				break;											// break loop if last node
			generate_path(i, i + 1);							// genreates node path from current node to next node in pickup zone
			orient(S);											// traverse to throught he path towards south
		}
		if (nuts_picked == 4)
			break;
	}

	generate_path(curr_node, 9);								// genreates node path from current node to next 9
	orient(N);													// traverse to throught he path towards north
	forward();													// move to start
	_delay_ms(1000);
	left();	velocity(230, 230);									// orient as it was at start
	_delay_ms(1000);
	stop();
	_delay_ms(3000);
}
