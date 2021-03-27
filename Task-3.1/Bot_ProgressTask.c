#include<asf.h>
#include<avr/io.h>
#include<avr/interrupt.h>
#include<util/delay.h>

#define F_CPU 16000000

int node=0;

void init_pins()
{
	// Color Sensor
	DDRB |= 0b11110000;	// PB - 4,5,6,7 // Set S0, S1, S2, S3 as output
	DDRD &=~ 0x01; // Set pin PD0 as input
	PORTD |= 0x01 //Enable internal pull-up for PORTD pin 0
	
	// Sharp Sensor
	DDRF &=~ 0x01;	// Set ADC0 as input
	
	// IR Array
	DDRK &=~ 0b00000111;	// Set ADC - 8,9,10 as input
	
	// Motor Driver
	DDRL |= 0b11111100;	// Set PL - 2,3,4,5,6,7 as output
	PORTL = 0b00000000;
	
	// Buzzer
	DDRA |= 0x01;	// Set PA0 as output
	
	//init_adc
	ADCSRA|=((1<<ADEN)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0));	//Enables ADC & Set ADC Prescaler to 128
	ADCSRB|=(1<<MUX5);	//As we are going to read from pins ADC8,ADC9 & ADC10 only (MUX 5 is 1 for all 3 pins)
	
	//init_pwm for Motor Driver (Left & Right Wheels)
	TCCR5A|=( (1<<WGM51)|(1<<COM5A1)|(1<<COM5C1) );
	TCCR5B|=( (1<<WGM53)|(1<<WGM52)|(1<<CS51) );	//Fast non-inverting Fast PWM with ICR5 as TOP, Prescaler=8 
	OCR5A=0;
	OCR5C=0;
	ICR5=1000;	//Freq= 2KHz
}

uint16_t adc_read()
{
	ADCSRA|=(1<<ADSC);
	while(!(ADCSRA&(1<<ADIF)));
	ADCSRA|=(1<<ADIF);
	return ADC;
}

int line_sensor()
{
	uint16_t s1,s2,s3;
	
	ADMUX=0b01100000;	//Select ADC8
	s1=adc_read();
	ADMUX=0b01100001;	//Select ADC9
	s2=adc_read();
	ADMUX=0b01100010;	//Select ADC10
	s3=adc_read();
	
	return ( ((s1<350)<<2)|((s2<350)<<1)|(s3<350) );
}

void forward()
{
	OCR5A=1000;	//Left  Motor 100% duty cycle
	OCR5C=1000;	//Right Motor 100% duty cycle
	PORTL = 0b10111000;		// IN1,IN3 High; IN2,IN4 Low; EN High 
}

void right()
{
	OCR5A=1000;
	OCR5C=1000;
	PORTL = 0b01111000;		// IN1,IN4 High; IN2,IN3 Low; EN High
}

void left()
{
	OCR5A=1000;
	OCR5C=1000;
	PORTL = 0b10101100;		// IN2,IN3 High; IN1,IN4 Low; EN High
}

void soft_right(int ratio)
{
	OCR5A=1000;
	OCR5C=1000-ratio;	//right motor ( 1- (ratio/10) )% duty cycle
	PORTL = 0b10111000;		// IN1,IN3 High; IN2,IN4 Low; EN High
}

void soft_left(int ratio)
{
	OCR5A=1000-ratio;
	OCR5C=1000;
	PORTL = 0b10111000;		// IN1,IN3 High; IN2,IN4 Low; EN High
}

void reverse()
{
	OCR5A=1000;
	OCR5C=1000;
	PORTL = 0b01101100;		// IN2,IN4 High; IN1,IN3 Low; EN High
}

void stop()
{
	OCR5A=1000;
	OCR5B=1000;				//Both ENs are set to HIGH for enabling Forced Braking
	PORTL = 0b00101000;			// IN, EN High
}

void line_track()
{
	int array = line_sensor();
	int line_memory=0;
	
	if(array==0b010)
	{
		forward();
		line_memory=0;
	}
	else if(array==0b001)
	{
		soft_right(600);
		line_memory=1;
	}
	else if(array==0b011)
	{
		soft_right(300);
		line_memory=1;
	}
	else if(array==0b100)
	{
		soft_left(600);
		line_memory=2;
	}
	else if(array==0b110)
	{
		soft_left(300);
		line_memory=2;
	}
	else if(array==0b000)
	{
		if(line_memory==0)
		{
			forward();
		}
		else if(line_memory==1)
		{
			soft_right(1000);
		}
		else if(line_memory==2)
		{
			soft_left(1000);
		}
	}
	else if(array==0b111)
	{
		stop();
		node++;
		_delay_ms(10000);
		forward();
		_delay_ms(3000);
	}
}

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */

	board_init();

	/* Insert application code here, after the board has been initialized. */
	init_pins();
	
	while(1)
	{
		line_track();
	}
	
}
