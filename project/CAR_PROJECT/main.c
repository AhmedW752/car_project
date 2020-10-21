/*
 * main.c
 *
 *  Created on: Oct 8, 2020
 *      Author: ahmed
 */

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
unsigned char position;
int TimerOverflow = 0;
void car_switch_lane (void);
unsigned char obs_Distance (unsigned char sensor_order);
void car_velo (unsigned char acc);
void car_position (void);

ISR(TIMER1_OVF_vect)
{
	TimerOverflow++;	/* Increment Timer Overflow count */
}

unsigned char obs_Distance (unsigned char sensor_order)
{
	switch (sensor_order)
		{
	case 'B': // left side sensor to detect distance to left side of car track
		DDRA |= (1<<1);
		PORTA |= (1 << PA1); // trigger pin of left sensor to be turned on for 10 us
		_delay_us(10);
		PORTA &= (~(1 << PA1));
		break;
	case 'C': // right side sensor to detect distance to right side of car track
		DDRA |= (1<<2);
		PORTA |= (1 << PA2); //trigger pin of right sensor to be turned on for 10 us
		_delay_us(10);
		PORTA &= (~(1 << PA2));
		break;
	default : // main sensor in the front to detect distance if there is obstacles in car route
		DDRA |= (1<<0);
		PORTA |= (1 << PA0); //trigger pin of main front sensor to be turned on for 10 us
		_delay_us(10);
		PORTA &= (~(1 << PA0));
		break;
		}

		long count;
		double distance;
		PORTD = 0xFF;		/* Turn on Pull-up */

		SREG|=(1<<7);			/* Enable global interrupt */
		TIMSK = (1 << TOIE1);	/* Enable Timer1 overflow interrupts */
		TCCR1A = 0;		/* Set all bit to zero Normal operation */

		while(1)
		{
			/* Give 10us trigger pulse on trig. pin to HC-SR04 */

			TCNT1 = 0;	/* Clear Timer counter */
			TCCR1B = 0x41;	/* Capture on rising edge*/
			TIFR = (1<<ICF1);	/* Clear ICP flag (Input Capture flag) */
			TIFR = (1<<TOV1);	/* Clear Timer Overflow flag */

			/*Calculate width of Echo by Input Capture */

			while ((TIFR & (1 << ICF1)) == 0);/* Wait for rising edge */
			TCNT1 = 0;	/* Clear Timer counter */
			TCCR1B = 0x01;	/* Capture on falling edge */
			TIFR = 1<<ICF1;	/* Clear ICP flag (Input Capture flag) */
			TIFR = 1<<TOV1;	/* Clear Timer Overflow flag */
			TimerOverflow = 0;/* Clear Timer overflow count */

			while ((TIFR & (1 << ICF1)) == 0);/* Wait for falling edge */
			count = ICR1 + (65535 * TimerOverflow);	/* Take count */
			/* 8MHz Timer frequency, sound speed =343 m/s */
			distance = (double)count / 466.47;
			return distance;
			}
}

void car_velo (unsigned char acc) // function to control motor movement
{
	DDRB|=(1<<3);
	TCCR0|=(1<<3)|(1<<6) | (1<<5)|(1<<1);
	OCR0=acc;

}

void car_position (void)
{
double Lef,Rig; /* position corresponding to lane*/
	Lef=obs_Distance('B'); // distance read of left sensor
	Rig=obs_Distance('C'); // distance read of right sensor
	if(Lef>Rig)
	{
		position='R';
	}
	else if(Rig>Lef)
	{
		position='L';
	}
	else
	{
		PORTC|=(1<<PC3); // change motor angle to step right
		_delay_ms(3000); // wait until lane is switched
		PORTC&=(~(1<<PC3)); // return to normal condition
		position='R';
	}
}

void car_switch_lane (void)
{
	DDRC|=(1<<0)|(1<<3); // initializing motor pins to be output pins
	PORTC=0X00; // initially stepper motor has no angle
	car_velo(100); // car slows down
	_delay_ms(20);
		if(position=='L')
			{
			PORTC|=(1<<PC3); // change motor angle to step right
			_delay_ms(3000); // wait until lane is switched
			PORTC&=(~(1<<PC3)); // return to normal condition
			}
			else if (position=='R')
			{
			PORTC|=(1<<PC0); // change motor angle to step left
			_delay_ms(3000); // wait until lane is switched
			PORTC&=(~(1<<PC0)); // return to normal condition
			}

}

int main (void)
{

	car_velo(50); // velocity until car detects it's current position
	car_position(); // detect position of car
	car_velo(200); // speed up to default speed
	while(1)
	{
		if (obs_Distance('e')<15) // condition to detect obstacles
		{
			car_switch_lane(); // changes car lane
			car_position(); // detects car new location after dodging the obstacle if existed
		}

	}
}
