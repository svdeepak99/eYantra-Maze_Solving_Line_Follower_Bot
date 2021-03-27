/*
*
* Team Id: 1508
* Author List: Gudapati Nitish, Deepak S V, Jinesh R, Himadri Poddar
* Filename: NS_Task_1_Sandbox.h
* Theme: Nutty Squirrel -- Specific to eYRC
* Functions: forward_wls(unsigned char), left_turn_wls(void), right_turn_wls(void), Square(void), Task_1_1(void), Task_1_2(void)
* Global Variables: color_sensor_pulse_count
*
*/

#include "NS_Task_1_Predef.h"
extern unsigned int color_sensor_pulse_count;

/*
* Function Name: forward_wls
* Input: node
* Output: void
* Logic: Uses white line sensors to go forward by the nodes specified
* Example Call: forward_wls(2) //Goes forward by two nodes
*
*/
void forward_wls(unsigned char node);

/*
*
* Function Name: left_turn_wls
* Input: void
* Output: void
* Logic: Uses white line sensors to turn left until black line is encountered
* Example Call: left_turn_wls(); //Turns left until black line is encountered
*
*/
void left_turn_wls(void);

/*
*
* Function Name: right_turn_wls
* Input: void
* Output: void
* Logic: Uses white line sensors to turn right until black line is encountered
* Example Call: right_turn_wls(); //Turns right until black line is encountered
*
*/
void right_turn_wls(void);

/*
*
* Function Name: Square
* Input: void
* Output: void
* Logic: Use this function to make the robot trace a square path on the arena
* Example Call: Square();
*/
void Square(void);

/*
*
* Function Name: Task_1_1
* Input: void
* Output: void
* Logic: Use this function to encapsulate your Task 1.1 logic
* Example Call: Task_1_1();
*/
void Task_1_1(void);

/*
*
* Function Name: Task_1_2
* Input: void
* Output: void
* Logic: Use this function to encapsulate your Task 1.2 logic
* Example Call: Task_1_2();
*/
void Task_1_2(void);