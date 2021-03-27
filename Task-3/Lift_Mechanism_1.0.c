/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

#define F_CPU 16000000

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="https://www.microchip.com/support/">Microchip Support</a>
 */
#include <asf.h>
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>


#define confidence_max 50
#define confidence_thresh 35

#define IR   (PIND&(1<<PIND2))>>PIND2
#define SWT0 (PIND&(1<<PIND3))>>PIND3
#define SWT1 (PIND&(1<<PIND4))>>PIND4

#define IN1 PIND5
#define IN2 PIND6
#define EN1 PIND7

bool ir_state,swt0_state,swt1_state;
unsigned char lift_state=0;	//0 - lower floor , 1 - upper floor , 2 - transition state 
/*
 D2- ir sensor , D3- Bottom end switch , D4- Top end switch , D5- in1 , D6- in2 , D7- en1
*/
void init_pins(void)
{
	DDRD=0b11100010;
	EICRA|=(1<<ISC11)|(1<<ISC10)|(1<<ISC01);	//INT1 - Trigger at rising edge , INT0 - Trigger at falling edge
	PCMSK2|=(1<<PCINT20);	//D4 as pin change interrupt	
}

void ascend(void)
{
	lift_state=2; //lift in movement
	PORTD&=~(1<<IN1);
	PORTD|= (1<<IN2)|(1<<EN1);
	PCICR|=(1<<PCIE2);
	while(swt1_state == 0)
		_delay_ms(10);
	PCICR&=~(1<<PCIE2);
	PORTD&=~( (1<<IN1)|(1<<IN2)|(1<<EN1) );
	swt0_state=0; //swt0 is not pressed
	lift_state=1; //lift has reached 1st floor
}

void decend(void)
{
	lift_state=2; //lift in movement
	PORTD&=~(1<<IN2);
	PORTD|= (1<<IN1)|(1<<EN1);
	EIMSK|=(1<<INT1);
	while(swt0_state == 0)
		_delay_ms(10);
	EIMSK&=~(1<<INT1);
	PORTD&=~( (1<<IN1)|(1<<IN2)|(1<<EN1) );
	swt1_state=0; //swt1 is not pressed
	lift_state=0; //lift has reached ground floor
}

void ir_high_update()
{
	unsigned int confidence_ir_high=0;
	if(ir_state == 0)
	{
		for(int i=0;i<confidence_max;i++)
		{
			if(IR)
				confidence_ir_high++;
			_delay_ms(1);
		}
		if(confidence_ir_high>confidence_thresh)
			ir_state=1;
	}
}

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */

	board_init();

	/* Insert application code here, after the board has been initialized. */
	
	init_pins();
	sei();
	_delay_ms(100);
	
	if(swt0_state)
		lift_state=0;
	else if(swt1_state)
		lift_state=1;
	ir_high_update();
	
	EIMSK|=(1<<INT0);
	while(1)
	{
		if(ir_state == 0)
		{
			EIMSK&=~(1<<INT0);
			if(lift_state == 0){
				ascend();
				_delay_ms(7000);
				EIMSK|=(1<<INT0);
				ir_high_update();
			}
			else if(lift_state == 1){
				decend();
				_delay_ms(7000);
				EIMSK|=(1<<INT0);
				ir_high_update();
			}
		}
		_delay_ms(10);
	}
	return 0;
}

ISR(INT0_vect)
{
	unsigned int confidence_ir=0;
	if(ir_state == 1)
	{
		for(int i=0;i<confidence_max;i++)
		{
			if(~IR)
				confidence_ir++;
			_delay_ms(1);
		}
		if(confidence_ir>confidence_thresh)
			ir_state=0;
	}
}

ISR(INT1_vect)
{
	unsigned int confidence_swt0=0;
	if(swt0_state == 0)
	{
		for(int i=0;i<confidence_max;i++)
		{
			if(SWT0)
				confidence_swt0++;
			_delay_ms(1);
		}
		if(confidence_swt0>confidence_thresh)
			swt0_state=1;
	}
}

ISR(PCINT2_vect)
{
	unsigned int confidence_swt1=0;
	if(swt1_state == 0)
	{
		for(int i=0;i<confidence_max;i++)
		{
			if(SWT1)
				confidence_swt1++;
			_delay_ms(1);
		}
		if(confidence_swt1>confidence_thresh)
			swt1_state=1;
	}
}
