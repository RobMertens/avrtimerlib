#include <avr/io.h>
#include <settings.h>
#include <timer8.h>

timer8 t2(2, &TCCR2A, &TCCR2B, &TCNT2, &TIMSK2, &OCR2A, &OCR2B);

// INIT
void setup()
{
	Serial.begin(9600);
	
	t2.setMode(t_mode::NORMAL);
	t2.setInterruptMode(t_interrupt::OVF);
	t2.setPrescaler(64);
	t2.reset();
}

// MAIN PROGRAM
void loop()
{
	Serial.println(t2.getOverflowCount());
}



