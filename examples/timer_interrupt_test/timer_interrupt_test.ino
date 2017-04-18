/*******************************************************************************
 * avr-timer-lib
 * timer_interrupt_test.ino
 *
 * Test file for timer interrupts.
 * I am using an arduino MEGA ADK (AVR atmega2560).
 * 
 * @author: 	Rob Mertens
 * @date:	18/04/2017
 * @version:	1.1.1   
 ******************************************************************************/

#include <avr/io.h>
#include <settings.h>
#include <timer8.h>

//TIMER2.
timer8 t2(2, &TCCR2A, &TCCR2B, &TCNT2, &TIMSK2, &OCR2A, &OCR2B);

//INIT.
void setup()
{
	//SERIAL.
	Serial.begin(9600);
	
	//TIMER2 SETTINGS.	
	t2.setMode(t_mode::NORMAL);
	t2.setInterruptMode(t_interrupt::OVF);
	t2.setPrescaler(64); 				// Available prescalers for T2: 1, 8, 32, 64.
	t2.reset();
}

//MAIN PROGRAM.
void loop()
{
	Serial.println(t2.getOverflowCount());
}



