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
timer8 t2(2, &TCCR2A, &TCCR2B, &TCNT2, &TIMSK2, &OCR2A, &OCR2B);

//INIT.
void setup()
{
	//LED.
	DDRLED |= 0x80;
  	OUTLED &= 0x7F;
	
	//TIMER2 SETTINGS.	
	t2.setMode(t_mode::PWM, t_pwm::FAST);
	t2.setPrescaler(64); 				// Available prescalers for T2: 1, 8, 32, 64.
	t2.reset();
}

//MAIN PROGRAM.
void loop(){;;}



