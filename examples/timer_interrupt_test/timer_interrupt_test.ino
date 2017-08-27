/*******************************************************************************
 * avr-timer-lib
 * timer_interrupt_test.ino
 *
 * Test file for timer interrupts.
 * I am using an arduino MEGA ADK (AVR atmega2560).
 *
 * Available timers for MEGA:
 *
 * 			| 8-bit | 16-bit
 *   T0	|   x   |
 *   T1	|       |   x
 *   T2	|   x   |
 *   T3	|       |   x
 *   T4	|       |   x
 *   T5	|       |   x
 *
 * @author:		Rob Mertens
 * @date:			18/04/2017
 * @version: 	1.1.1
 ******************************************************************************/

#include "settings.h"
#include "timer8.h"
#include "timer16.h"

//TIMER2.
timer8	t0(t_alias::T0);
timer16 t1(t_alias::T1);
timer8	t2(t_alias::T2);
timer16 t3(t_alias::T3);
timer16 t4(t_alias::T4);
timer16 t5(t_alias::T5);

//INIT.
void setup()
{
	//SERIAL.
	Serial.begin(9600);

	//TIMER0 SETTINGS.
	t0.initialize(t_mode::CTC, t_interrupt::COMPA);		// TIMER0_OVF_vect not available (used by arduino).
	t0.setCompareValueA(200);
	t0.setPrescaler(1); 					// Available prescalers for T0: 1, 8, 64, 256, 1024.
	t0.reset();

	//TIMER1 SETTINGS.
	t1.initialize(t_mode::NORMAL, t_interrupt::OVF);
	t1.setPrescaler(1); 					// Available prescalers for T1: 1, 8, 64, 256, 1024.
	t1.reset();

	//TIMER2 SETTINGS.
	t2.initialize(t_mode::NORMAL, t_interrupt::OVF);
	t2.setPrescaler(32); 					// Available prescalers for T2: 1, 8, 32, 64.
	t2.reset();

	//TIMER3 SETTINGS.
	t3.initialize(t_mode::CTC, t_interrupt::COMPB);
	t4.setCompareValueB(2000);
	t3.setPrescaler(64);         				// Available prescalers for T3: 1, 8, 64, 256, 1024.
	t3.reset();

	//TIMER4 SETTINGS.
	t4.initialize(t_mode::CTC, t_interrupt::COMPC);
	t4.setCompareValueC(50000);
	t4.setPrescaler(256);         				// Available prescalers for T4: 1, 8, 64, 256, 1024.
	t4.reset();

	//TIMER5 SETTINGS.
	t5.initialize(t_mode::NORMAL, t_interrupt::NONE);
	t5.setPrescaler(1024);			        	// Available prescalers for T5: 1, 8, 64, 256, 1024.
	t5.reset();
}

//MAIN PROGRAM.
void loop()
{
	Serial.print("T0::");
	Serial.print(t0.getOverflows());
	Serial.print("\tT1::");
	Serial.print(t1.getOverflows());
	Serial.print("\tT2::");
	Serial.print(t2.getOverflows());
	Serial.print("\tT3::");
	Serial.print(t3.getOverflows());
	Serial.print("\tT4::");
	Serial.print(t4.getOverflows());
	Serial.print("\tT5::");
	Serial.println(t5.getOverflows());
}
