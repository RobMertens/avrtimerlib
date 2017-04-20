/*******************************************************************************
 * avr-timer-lib
 * timer_fastpwm_test.ino
 *
 * Test file for timer interrupts.
 * I am using an arduino MEGA ADK (AVR atmega2560).
 * 
 * @author: 	Rob Mertens
 * @date:	18/04/2017
 * @version:	1.1.1   
 ******************************************************************************/
//INCLUDES.
#include <avr/io.h>
#include <settings.h>
#include <timer8.h>

//LED INDICATOR.
#define DDRLED DDRB
#define OUTLED PORTB

//TIMER2.
timer8 t0(0, &TCCR0A, &TCCR0B, &TCNT0, &TIMSK0, &OCR0A, &OCR0B);

//INIT.
void setup()
{	
	//LED.
	DDRLED |= 0x80;
  	OUTLED &= 0x7F;
	
	//TIMER2 SETTINGS.	
	t0.initialize(t_mode::PWM_F, t_channel::A, false);		// Non-inverted FAST PWM on channel A (T0 OC1A : PB7).
									// I have a LED connected to PB7.
	t0.setPrescaler(1024); 						// Available prescalers for T2: 1, 8, 64, 256, 1024.
	t0.setDutyCycleA(1.0);
	t0.reset();
}

//MAIN PROGRAM.
void loop()
{
	for(double i=0; i<=1; i+=0.1)
	{
		t0.setDutyCycleA(i);
	}
}



