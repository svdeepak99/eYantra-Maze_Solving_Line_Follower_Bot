
#define F_CPU 16000000

#include<asf.h>
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>
#include<stdlib.h>

// Function Declarations

void line_track(void);
void init_pins(void);
void forward(void);
void right(void);
void left(void);
void back(void);
void stop(void);
void soft_right(void);
void soft_left(void);
void velocity(int,int);
uint8_t adc_read(void);
void obstracle(int);
int line_sensor(void);
void pick(void);
uint8_t sharp_read(void);
void forward_wls(char);
void back_node_cross(void);
void right_turn_wls(int);
void left_turn_wls(int);
void F(void);
void L(int);
void R(int);
void pick(void);
int pick_nut(void);
void place(void);
void place_nut(void);
void generate_path(char,char);
void orient(int);
//Color Sensor Functions Starts
void rgb_port_config (void);
void color_sensor_pin_config(void);
void color_sensor_pin_interrupt_init(void);
void init_RGB_Color_Sensor(void);
void filter_red(void);
void filter_green(void);
void filter_blue(void);
void filter_clear(void);
void color_sensor_scaling(void);
void red_read(void);
void green_read(void);
void blue_read(void);
int filter_color(void);
//Color Sensor Functions Ends

#define confidence_max 20			//max value of confidence in software debouncing
#define confidence_thresh 14		//threshold for dececion in software debouncing

#define back_speed 500				// Velocity while back_line_tracking

// The colours are given specific numerical codes
#define CLEAR 0
#define RED 1
#define GREEN 2
#define BLUE 3
#define BROWN 4

/*
*	pulse - Color Sensor Count
*	red,greed,blue - red,blue & green counts in color sensor
*	int pulse = 0, red = 0, green = 0, blue = 0;
*/
int pulse = 0, red = 0, green = 0, blue = 0;

/*
*	array - stores ir_array digital values (eg: 0b011 represents, left sensor on white surface & remaining 2 sensors on Black line
*	confidence - confidence variable for software debouncing
*	obs_confidence - for sofware debouncing in detecting obstracles
*	i - increment variable of for loop
*	line_memory - stores last turned direction of bot before ir_sensor falls away from black line, so that it can be brought back into the line
*	s1,s2,s3 - white-line sensor states
*	partial_turn - To enable tracking functions to take partial turns
*	pick_status - Flag Variable to show if bot has nut or not
*/
int array,confidence,line_memory=0,obs_confidence,partial_turn=0,pick_status=0;
uint8_t s1,s2,s3;

/*
*
*	Counts the number of iterations in line_track();
*		+1 for forward movement
*		+0.75 for curve turn
*		+0.5 for soft turn
*		+0 for spot turn
*
*/
float iteration = 0,back_iteration=0;

//	Initializing default values for North, East, South, West
enum direction { N, E, S, W };

/*
*
*	Variable name: maze
*	Array that stores complete node details of Task 1.2 track and can be used by Dijikstra`s algorithm for path planning
*
*/
//								0		1		2		3		4		5		6		7		8		9		10		11		12		13		14		15		16		17		18		19		20		21		22		23		24
int maze[24][25][2] = {	{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{28 ,N},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},	//0
						{	{-1,-1},{-1,-1},{-1,-1},{20,N },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,S }},	//1
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{20,N },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,S }},	//2
						{	{-1,-1},{20,S },{-1,-1},{-1,-1},{24,E },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{41 ,N},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},	//3
						{	{-1,-1},{-1,-1},{-1,-1},{24,W },{-1,-1},{22,E },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,S }},	//4
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{22,W },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{51,S },{20 ,N},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},	//5
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{22,E },{-1,-1},{51,S },{-1,-1},{20 ,N},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},	//6
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{22,W },{-1,-1},{25,E },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,S }},	//7
						{	{-1,-1},{-1,-1},{20,S },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{25,W },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{41 ,N},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},	//8
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{51,E },{51, W},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1 },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},	//9
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{20 ,W},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{31 ,E},{-1,-1},{-1,-1},{28 ,N},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},	//10
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{20 ,E},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{31 ,W},{-1,-1},{-1,-1},{-1,-1},{29 ,N},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},	//11
						{	{28,S },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{31, W},{31 ,E},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{55 ,N},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},	//12
						{	{-1,-1},{-1,-1},{-1,-1},{41 ,S},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{36 ,E},{-1,-1},{58 ,N},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},	//13
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{41 ,S},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{35 ,W},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{58 ,N},{-1,-1}},	//14
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{28 ,S},{-1,-1},{-1,-1},{36 ,W},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{51 ,N},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1}},	//15
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{29 ,S},{-1,-1},{-1,-1},{35 ,E},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{51 ,N},{-1,-1},{-1,-1}},	//16
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{58 ,S},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{32,E },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,N }},	//17
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{51 ,S},{-1,-1},{32,W },{-1,-1},{25,E },{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,N }},	//18
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{25,W },{-1,-1},{23,E },{-1,-1},{-1,-1},{-1,-1},{-1,N }},	//19
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{55 ,S},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{23,W },{-1,-1},{23,E },{-1,-1},{-1,-1},{-1,-1}},	//20
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{23,W },{-1,-1},{26,E },{-1,-1},{-1,N }},	//21
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{51 ,S},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{26,W },{-1,-1},{32,E },{-1,N }},	//22
						{	{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{58 ,S},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{-1,-1},{32,W },{-1,-1},{-1,N }},	//23
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

//Function Definitions Starts Here

//Color Sensor Function Definitions Starts

//Configure RGB led
void rgb_port_config (void)
{
	/*****************************************
	Define DDR and PORT values for the port on which RGB LED is connected
	******************************************/
	DDRA  |=  0x0E;
	PORTA &=~ 0x0E;
}

//Configure Color Sensor pins
void color_sensor_pin_config(void)
{
	/*****************************************
	Define DDR and PORT values for the port on which Color sensor is connected
	******************************************/
	DDRD &=~ 0x01; //Set the direction of the PORTD pin 0 as input
	DDRB |= 0xF0;  //Set S0,S1,S2 & S3 as output pins
	PORTD |= 0x01; //Enable internal pull-up for PORTD 0 pin

}

//Configure Color Sensor interrupt for counting - INT0
void color_sensor_pin_interrupt_init(void) //Interrupt 0 enable
{
	cli(); //Clears the global interrupt
	EICRA |= 0x02; // INT0 is set to trigger with falling edge
	EIMSK |= 0x01; // Enable Interrupt INT0 for color sensor
	sei(); // Enables the global interrupt
}

//ISR for color sensor
ISR(INT0_vect) // Pass the timer number in place of n in INTn_vect
{
	//increment on receiving pulse from the color sensor
	pulse++;
}

//Initialize function for RGB led & color sensor
void init_RGB_Color_Sensor(void)
{
	cli();									//Clears the global interrupt
	rgb_port_config();							//Initialize all the ports here
	color_sensor_pin_config();						
	color_sensor_pin_interrupt_init();
	sei();									// Enables the global interrupt
}

//Filter Selection

void filter_red(void)		//Used to select red filter
{
	//Filter Select - red filter
	 //set S2 low
	 //set S3 low
	PORTB &=~ 0xC0;
}

void filter_green(void)		//Used to select green filter
{
	//Filter Select - green filter
	 //set S2 High
	//set S3 High
	PORTB |= 0xC0;
}

void filter_blue(void)		//Used to select blue filter
{
	//Filter Select - blue filter
	PORTB &=~ 0x80;		//set S3 low
	PORTB |=  0x40;		//set S2 High
}

void filter_clear(void)		//select no filter
{
	//Filter Select - no filter
	PORTB |=  0x80;		//set S3 High
	PORTB &=~ 0x40;		//set S2 Low
}

//Color Sensing Scaling
void color_sensor_scaling()		//This function is used to select the scaled down version of the original frequency of the output generated by the color sensor, generally 20% scaling is preferable, though you can change the values as per your application by referring datasheet
{
	//Output Scaling 20% from datasheet
	
	PORTB |= 0x20;		//set S1 high
	PORTB |= 0x10;		//set S0 high
}

void red_read(void) // function to select red filter and display the count generated by the sensor on LCD. The count will be more if the color is red. The count will be very less if its blue or green.
{
	//Red
	filter_red();	//select red filter
	pulse = 0;		//reset the count to 0
	_delay_ms(100);	//capture the pulses for 100 ms or 0.1 second
	red = pulse;	//store the count in variable called red

}

void green_read(void) // function to select green filter and display the count generated by the sensor on LCD. The count will be more if the color is green. The count will be very less if its blue or red.
{
	//Green
	filter_green();	//select green filter
	pulse = 0;		//reset the count to 0
	_delay_ms(100);	//capture the pulses for 100 ms or 0.1 second
	green = pulse;	//store the count in variable called green

}

void blue_read(void) // function to select blue filter and display the count generated by the sensor on LCD. The count will be more if the color is blue. The count will be very less if its red or green.
{
	//Blue
	filter_blue();	//select blue filter
	pulse = 0;		//reset the count to 0
	_delay_ms(100);	//capture the pulses for 100 ms or 0.1 second
	blue = pulse;	//store the count in variable called blue
	
}

//Returns Color Detected by Color Sensor
int filter_color(void)
{
	red_read();
	green_read();
	blue_read();
	if(red < 2800 && blue < 2800 && green < 2800)
	{
		PORTA = 0x0E;	//Switch off RGB LED in case of black detection
		return CLEAR;
	}
	else if(red > green)
	{
		if(red > blue)
		{
			PORTA = 0x02;	//Glow only RED
			return RED;
		}
		else
		{
			PORTA = 0x08;	//Glow only BLUE
			return BLUE;
		}
	}
	else if(blue > green)
	{
		PORTA = 0x08;	//Glow only BLUE
		return BLUE;
	}
	else
	{
		PORTA = 0x04;	//Glow only GREEN
		return GREEN;
	}
}
//Color Sensor Function Definitions Ends Here

/*
*	Function Name:	init_pins
*	Purpose:		Initializes all I/O pins & ADC, sets Data direction of I/O pins
*	Example call:	init_pins()
*/
void init_pins()
{
	// Color Sensor & RGB LED
	init_RGB_Color_Sensor();
	color_sensor_scaling();
	
	// Sharp Sensor
	DDRF &=~ 0x01;			// Set ADC0 as input
	
	// IR Array
	DDRK &=~ 0b00000111;	// Set ADC - 8,9,10 as input
	
	// Motor Driver
	DDRL |= 0b11111100;		// Set PL - 2,3,4,5,6,7 as output
	PORTL = 0b00000000;
	
	// Buzzer
	DDRA |= 0x01;			// Set PA0 as output
	
	//Gripper Servos
	DDRH |= 0b00011000;		// Set PH - 3,5 as output
	
	//init_adc
	ADCSRA|=((1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));	//Enables ADC & Set ADC Prescaler to 128
	
	//init pwm for Motor Driver (Left & Right Wheels)
	TCCR5A|=( (1<<WGM51)|(1<<COM5A1)|(1<<COM5C1) );
	TCCR5B|=( (1<<WGM53)|(1<<WGM52)|(1<<CS51) );			//Fast non-inverting Fast PWM with ICR5 as TOP, Prescaler=8
	OCR5A=0;
	OCR5C=0;
	ICR5=1000;												//Freq= 2KHz
	
	//init pwm for Gripper Servos
	TCCR4A|=( (1<<WGM41)|(1<<COM4A1)|(1<<COM4B1) );
	TCCR4B|=( (1<<WGM43)|(1<<WGM42)|(1<<CS41) );			//Fast non-inverting Fast PWM with ICR4 as TOP, Prescaler=8
	OCR4A=2600;												//Open Gripper - 2600 TO 3500
	OCR4B=4250;												//Lift Gripper - 1100 to 4250
	ICR4=40000;												//Freq= 50Hz
}

/*
*	Function Name:	adc_read
*	Purpose:		Reads active ADC pins value & returns the integer value on a scale of 0 to 255
*	Example call:	int ADC_value = adc_read();
*/
uint8_t adc_read()
{
	ADCSRA|=(1<<ADSC);
	while(!(ADCSRA&(1<<ADIF)));
	ADCSRA|=(1<<ADIF);
	return ADCH;
}


/*
*	Function Name:	sharp_read
*	Purpose:		Reads ADC value received from Sharp Sensor on a scale of 0 to 255
*	Return type:	uint8_t
*	Example Call:	uint8_t value = sharp_read();
*/
uint8_t sharp_read()
{
	ADCSRB&=~(1<<MUX5);
	ADMUX=0b01100000;
	return adc_read();
}

/*
*	Function Name:	line_sensor
*	Purpose:		Reads ADC ir_array input & returns a 3 bit value to be stored in array
*	Return type:	Integer
*	Example Call:	int array = line_sensor();
*/
int line_sensor()
{
	int sensor;
	ADCSRB|=(1<<MUX5);	//As we are going to read from pins ADC8,ADC9 & ADC10 only (MUX 5 is 1 for all 3 pins)
	ADMUX=0b01100000;	//Select ADC8
	s1=adc_read();
	ADMUX=0b01100001;	//Select ADC9
	s2=adc_read();
	ADMUX=0b01100010;	//Select ADC10
	s3=adc_read();
	
	sensor = ( ((s1>27)<<2)|((s2>27)<<1)|(s3>27) );	// Threshold for LED voltage of 2.13V
	
	if( (sensor==0b011) && (s2<195) && (s3<195) ){
		sensor=0b001;
		partial_turn=1;
	}
	else if( (sensor==0b110) && (s1<195) && (s2<195) ){
		sensor=0b100;
		partial_turn=2;
	}
	else if( (sensor==0b111) && (s2<195) ){
		sensor=0b010;
		partial_turn=0;
	}
	else
		partial_turn=0;
	return sensor;
}

/*	Function Name:	velocity
*	Purpose:		Set speeds of motors
*	Example Call:	velocity(250,750);
*/
void velocity(int left_speed,int right_speed)
{
	if( (left_speed >= 0) && (left_speed <= 1000) )
	OCR5A=left_speed;
	if( (right_speed >= 0) && (right_speed <= 1000) )
	OCR5C=right_speed;
}

/*	Function Name:	forward
*	Purpose:		Move the bot forward at full speed
*	Example Call:	forward();
*/
void forward()
{
	PORTL = 0b10111000;		// IN1,IN3 High; IN2,IN4 Low; EN High
}

/*	Function Name:	right
*	Purpose:		Turns bot right at full speed
*	Example Call:	right();
*/
void right()
{
	PORTL = 0b01111000;		// IN1,IN4 High; IN2,IN3 Low; EN High
}

/*	Function Name:	left
*	Purpose:		Turns bot right at full speed (using both wheels)
*	Example Call:	left()
*/
void left()
{
	PORTL = 0b10101100;		// IN2,IN3 High; IN1,IN4 Low; EN High
}

/*	Function Name:	soft_right
*	Purpose:		Turns bot right by moving left wheel forward at full speed
*	Example Call:	soft_right();
*/
void soft_right()
{
	PORTL = 0b00111000;		// IN1 High; IN2,IN3,IN4 Low; EN High
}

/*	Function Name:	soft_left
*	Purpose:		Turns bot left by moving right wheel forward at full speed
*	Example Call:	soft_left();
*/
void soft_left()
{
	PORTL = 0b10101000;		// IN3 High; IN1,IN2,IN4 Low; EN High
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
	
	int turn_speed=1000;
	if(count<0)
	{
		turn_speed=400;
		count*=-1;
	}
	while (count-- > 0)
	{
		int confidence = 0;									// counter variable for software debounce
		left();		velocity(turn_speed, turn_speed);
		if(curr_node!=15 && curr_node!=16 && curr_node!=14 && !(prev_node==15 && curr_node==13 && target_node==3))
			_delay_ms(300);
		else
			_delay_ms(100);
		stop();
		for (int i = 0; i < confidence_max; i++)			// confirms if left sensor is still on black line
		{
			if ((line_sensor() & 0b010) == 0b010)
				confidence++;
		}
		if (confidence > confidence_thresh)
		{
			left();		velocity(turn_speed, turn_speed);
			while (1)										// if left sensor is on black, turn left until it is out of the black line
			{
				confidence = 0;								// resets 'confidence' variable
				for (int i = 0; i < confidence_max; i++)	// confirms if out of line via software debouncing
				{
					if ((line_sensor() & 0b010) == 0)
					confidence++;
				}
				if (confidence > confidence_thresh)
				break;									// confirms and breaks out of the loop
			}
		}
		while (1)
		{
			left();		velocity(turn_speed, turn_speed);	// Turn left until left ir sensor detects a black line
			confidence = 0;
			for (int i = 0; i < confidence_max; i++)		// confirms if black line is detected by left sensor
			{
				if ((line_sensor() & 0b010) == 0b010)
					if(turn_speed!=400 || s2>205)
						confidence++;
			}
			if (confidence > confidence_thresh)				
			{
				break;// confirms and breaks out of the loop
			}
		}
		_delay_ms(70);
		stop();
		_delay_ms(100);
	}
}

/*	Function Name:	reverse
*	Purpose:		Moves bot backward at full speed
*	Example Call:	reverse();
*/
void back()
{
	PORTL = 0b01101100;		// IN2,IN4 High; IN1,IN3 Low; EN High
}

/*	Function Name:	stop
*	Purpose:		Stops (stalls) both motors of bot
*	Example Call:	stop();
*/
void stop()
{
	PORTL = 0x00;			// IN, EN Low
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
	if (condition==0)
	{
		back();		velocity(1000,1000);
		_delay_ms(500);
		stop();
		L(1);									// Take 180 degree turn and line track till previous node
		_delay_ms(100);
		obs_new_dir = 1;						//  Made 180 degree turn
	}
	else
	{
		back_node_cross();	//cross a node backwards
		_delay_ms(100);
		forward_wls(1);
		obs_new_dir = 0;						// MAde no turn
	}
}

/*	Function Name:	line_track
*	Purpose:		Tracks a black line until a node is reached
*	Example Call:	line_track();
*/
void line_track()
{
	iteration=0;
	forward();		velocity(1000,1000);	//to bring bot safely out of a node
	_delay_ms(100);
	stop();
	while(1)
	{
		if (sharp_read() > 115)										// Obstracle detected
		{
			stop();
			obs_confidence = 0;
			for (int i = 0; i < 10; i++)								// Confirms obstracle detection via software debouncing
			{
				if (sharp_read() > 115)
				{
					obs_confidence++;
				}
				_delay_ms(1);
			}
			if (obs_confidence > 5)										// If obstracle present, reroute the bot to previous node
			{
				if (iteration > 110)									// Good clearance for 180 degree turn
				{
					obstracle(0);
					break;
				}
				else								// No clearance, safe for on spot turn
				{
					obstracle(1);
					break;
				}
			}
		}
		_delay_ms(1);		
		
		array = line_sensor();
		confidence=0;
		
		if(array==0b010)
		{
			if(curr_node==3 && target_node==13){
				soft_right();	velocity(1000,1000);
			}
			else if(curr_node==8 && target_node==14){
				soft_left();	velocity(1000,1000);
			}
			else{
				forward();	velocity(1000,1000);
			}
			line_memory=0;	//Last moved direction of bot is forward
			iteration+=1.0;
		}
		else if(array==0b001)
		{
			if(curr_node==3 && target_node==13){
				right();	velocity(800,800);
			}
			else if(curr_node==8 && target_node==14){
				forward();	velocity(1000,1000);
			}
			else{
				soft_right();	velocity(1000,1000);
			}
			line_memory=1;	//Last moved direction of bot is right
			iteration+=0.5;
		}
		else if(array==0b100)
		{
			if(curr_node==3 && target_node==13){
				forward();	velocity(1000,1000);
			}
			else if(curr_node==8 && target_node==14){
				left();	velocity(800,800);
			}
			else
			{
				soft_left();	velocity(1000,1000);	
			}
			line_memory=2;		//Last moved direction of bot is left
			iteration+=0.5;
		}
		else if(array==0b000)
		{
			if(line_memory==0)
			{
				forward();	velocity(1000,1000);
				iteration+=1.0;
			}
			else if(line_memory==1)
			{
				if(curr_node==3 && target_node==13){
					right();	velocity(800,800);
				}
				else{
					soft_right();	velocity(1000,1000);
				}
				iteration+=0.5;
			}
			else if(line_memory==2)
			{
				if(curr_node==8 && target_node==14){
					left();	velocity(800,800);
				}
				else{
					soft_left();	velocity(1000,1000);
				}
				iteration+=0.5;
			}
		}
		else if(array==0b111 || array==0b011 || array==0b110)
		{															// Node Detection
			stop();
			confidence=0;											// Software Debouncing
			for(int i=0;i<confidence_max;i++)
			{
				array=line_sensor();
				if(array==0b111||array==0b011 || array==0b110)
				confidence++;
				_delay_us(10);
			}
			if(confidence>confidence_thresh)
			{
				break;
			}
		}
		_delay_ms(5);
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
void forward_wls(char node)
{
		int flag=0;
	if(node<0){
		flag=1;
		node*=-1;
	}
	while (node-- > 0)									// runs for 'node' number of times (number of nodes)
	{
		while(1)
		{
			forward();	velocity(1000,1000);
			if(line_sensor()!=0b111 && line_sensor()!=0b011 && line_sensor()!=0b110)
			{																			// Detecting Nodes
				stop();
				confidence=0;															// Software Debouncing
				for(int i=0;i<confidence_max;i++)
				{
					if(line_sensor()!=0b111 && line_sensor()!=0b011 && line_sensor()!=0b110)
					confidence++;
					_delay_us(10);
				}
				if(confidence>confidence_thresh)
				break;
			}
			_delay_ms(2);
		}
		if(flag==0){
			forward();	velocity(1000,1000);
			_delay_ms(50);
		}
		else{
			forward();	velocity(1000,1000);
			_delay_ms(50);
		}
		stop();
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
	int turn_speed=1000;
	if(count<0)
	{
		turn_speed=400;
		count*=-1;
	}
	while (count-- > 0)
	{
		int confidence = 0;					// counter variable for software debounce				
		right();		velocity(turn_speed, turn_speed);
		if(curr_node!=15 && curr_node!=16 && curr_node!=13 && !(prev_node==16 && curr_node==14 && target_node==8))
			_delay_ms(300);		//add if condition in acute angle turns
		else
			_delay_ms(100);
		stop();
		for (int i = 0; i < confidence_max; i++)			// confirms if right sensor is still on black line
		{
			if ((line_sensor() & 0b010) == 0b010)
				confidence++;
		}
		if (confidence > confidence_thresh)
		{
			right();	velocity(turn_speed,turn_speed);
			while (1)										// if right sensor is on black, turn right until it is out of the black line
			{
				confidence = 0;								// resets 'confidence' variable
				for (int i = 0; i < confidence_max; i++)	// confirms if out of line via software debouncing
				{
					if ((line_sensor() & 2) == 0)
					confidence++;
				}
				if (confidence > confidence_thresh)
				break;									// confirms and breaks out of the loop
			}
		}
		while (1)
		{
			right();		velocity(turn_speed, turn_speed);				// Turn right until right ir sensor detects a black line
			confidence = 0;
			for (int i = 0; i < confidence_max; i++)		// confirms if black line is detected by right sensor
			{
				if ((line_sensor() & 2) == 2)
					if(turn_speed!=400 || s2>205)
						confidence++;
			}
			if (confidence > confidence_thresh)				// confirms and breaks out of the loop
			break;
		}
		_delay_ms(70);
		stop();
		_delay_ms(100);										// to stabilize after a turn
	}
}

/*
*
* Function Name: back_node_cross
* Input: node
* Output: void
* Logic: Crosses a node backwards
* Example Call: back_node_cross()
*
*/
void back_node_cross()
{
	int back_line_array;
	while(1)
	{
		back();	velocity(500,500);
		back_line_array=line_sensor();
		if(back_line_array==0b111 || back_line_array==0b011 || back_line_array==0b110)
		{																			// Detecting Nodes
			stop();
			confidence=0;															// Software Debouncing
			for(int i=0;i<confidence_max;i++)
			{
				if(back_line_array==0b111 || back_line_array==0b011 || back_line_array==0b110)
				confidence++;
				_delay_us(10);
			}
			if(confidence>confidence_thresh)
			break;
		}
	}
	back();	velocity(1000,1000);
	_delay_ms(50);
	stop();
}


/*
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
*	Function Name:	pick
*	Purpose::		Pick block using Grippers
*	Example Call:	pick()
*/
void pick(void)
{
	back();	velocity(1000,1000);
	_delay_ms(150);
	stop();
	
	_delay_ms(100);
	
	OCR4B=1100;		//Gripper Low
	_delay_ms(2000);
	
	forward(); velocity(500,500);
	_delay_ms(450);
	stop();
	
	OCR4A=3500;		//Gripper Clamp
	_delay_ms(1000);
	OCR4B=4000;		//Gripper Raise
	_delay_ms(2000);
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
int pick_nut(void)
{
	
	int ca_t;
	int confidence_red=0,confidence_green=0,confidence_brown=0,i_c,filter;																		// for software debouncing
	if (maze[curr_node][prev_node][1] == E)													// if approach from East, turn right
	{
		forward_wls(-1);
		_delay_ms(10);
		stop();
		_delay_ms(10);
		right_turn_wls(-1);
		_delay_ms(10);
	}
	else if (maze[curr_node][prev_node][1] == W)											// if approach from West, turn right
	{
		forward_wls(-1);
		_delay_ms(10);
		stop();
		_delay_ms(10);
		left_turn_wls(-1);
		_delay_ms(10);
	}
	_delay_ms(100);
	stop();
	
	
	for (i_c = 0; i_c < 10; i_c++)															// confirms if red is detected
	{
		filter=filter_color();
		if (filter == RED)
			confidence_red++;
		else if(filter  == GREEN)
			confidence_green++;
		else if(filter == BROWN)
			confidence_brown++;
		_delay_ms(1);
	}
	
	
	PORTA|=1;			// Buzzer On
	_delay_ms(1500);
	PORTA&=~1;
	// picks red nut
	
	if (confidence_red > 5)
	{
		pick();
		pick_status=1;
		back();		velocity(1000,1000);
		_delay_ms(125);
		return RED;
	}

	//picks green nut
	if (confidence_green > 5)
	{
		pick();
		pick_status=1;
		back();		velocity(1000,1000);
		_delay_ms(125);
		return GREEN;
	}

	//does not pick obstracle
	if (confidence_brown > 5)
	{
		return BROWN;
	}
	// no object to pick
	return CLEAR;
}

/*
* Function Name:place();
* Purpose & Logic: Brings down gripper & places the nut
* Example Call: place
*/
void place()
{
	OCR4B=1100;		//Gripper Low
	_delay_ms(2000);
	
	OCR4A=2600;		//Place Nut
	_delay_ms(1000);
	
	OCR4B=4000;		//Gripper High
	_delay_ms(2000);
	
	back_node_cross();
	forward_wls(1);
	stop();
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
		right_turn_wls(-1);
	}
	else if (maze[curr_node][prev_node][1] == E)	// If approach from East, turn left
	{
		left_turn_wls(-1);
	}
	stop();
	// add align
	forward_wls(1);
	back_node_cross();
	forward_wls(1);
	place();										// places the nut in the location
	nuts_picked += 1;								// Increment number of nuts picked
	pick_status=0;
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
		if ((start_node == 12 && end_node == 0) || start_node == end_node)								// Dead End Cases elimination
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
* Example Call: orient(N);
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
			
			//Since nodes 15 & 16 are only nodes to have SW & SE directions respectively, they need an exceptional case
			if( (curr_node==16 && apr_dir==S && tar_dir!=E) || (curr_node==15 && apr_dir==N && tar_dir!=W) ){
				forward_wls(1);
				left_turn_wls(1);
			}
			else if( (curr_node==16 && apr_dir==N && tar_dir!=E) || (curr_node==15 && apr_dir==S && tar_dir!=W) ){
				forward_wls(1);
				right_turn_wls(1);
			}
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
					for (int j = 0; j < 25; j++)
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
						for (int j = 0; j < 25; j++)
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
					for (int j = 0; j < 25; j++)
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
						for (int j = 0; j < 25; j++)
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
					for (int j = 0; j < 25; j++)
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
						for (int j = 0; j < 25; j++)
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
					for (int j = 0; j < 25; j++)
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
						for (int j = 0; j < 25; j++)
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

/*	Function Name:	main
*	Purpose:		Does the Progress Task 4
*/
int main (void)
{
	board_init();
	
	init_pins();		// initialize pins
	
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
		if (nut_color == RED)
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
		else if (nut_color == GREEN)
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
		else if (nut_color == CLEAR)
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
	
	PORTA|=1;
	while(1)
		_delay_ms(100);	// To keep micro-controller active
	
	return 0;
	
}
