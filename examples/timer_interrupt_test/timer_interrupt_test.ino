/*******************************************************************************
 * avr-timer-lib
 * timer_interrupt_test.ino
 *
 * Test file for timer interrupts.
 * I am using an arduino MEGA ADK (AVR atmega2560).
 *
 * Available timers for MEGA:
 * 
 * 	| 8-bit | 16-bit
 *   T0	|   x   |   
 *   T1	|       |   x
 *   T2	|   x   |   
 *   T3	|       |   x
 *   T4	|       |   x
 *   T5	|       |   x
 * 
 * @author: 	Rob Mertens
 * @date:	18/04/2017
 * @version:	1.1.1   
 ******************************************************************************/

#include <settings.h>
#include <timer8.h>
#include <timer16.h>

//TIMER2.
timer16 t1(t_alias::T1);
timer8  t2(t_alias::T2);
timer16 t4(t_alias::T4);

//INIT.
void setup()
{
	//SERIAL.
	Serial.begin(9600);

	//TIMER1 SETTINGS.	
	t1.initialize(t_mode::NORMAL, t_interrupt::OVF);
	t1.setPrescaler(64); 				// Available prescalers for T1: 1, 8, 64, 256, 1024.
	t1.reset();
	
	//TIMER2 SETTINGS.	
	t2.initialize(t_mode::NORMAL, t_interrupt::OVF);
	t2.setPrescaler(1); 				// Available prescalers for T2: 1, 8, 32, 64.
	t2.reset();
  
	//TIMER4 SETTINGS.  
	t4.initialize(t_mode::CTC, t_interrupt::COMPB, 256);
	t4.setPrescaler(64);         // Available prescalers for T1: 1, 8, 64, 256, 1024.
	t4.reset();
}

//MAIN PROGRAM.
void loop()
{
	Serial.print(t1.getOverflowCount());
	Serial.print("\t");
	Serial.print(t2.getOverflowCount());
	Serial.print("\t");
	Serial.println(t4.getCompareCount());
}
