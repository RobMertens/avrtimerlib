#ifndef _TIMER8_H_
#define _TIMER8_H_

#include <avr/interrupt.h>
#include <interrupt.h>
#include <settings.h>

// ATMEGA2560
extern "C" void TIMER0_OVF_vect(void) __attribute__ ((signal));
extern "C" void TIMER2_OVF_vect(void) __attribute__ ((signal));

class timer8 : private interrupt::handler
{
	public:
		//Constructors ***************************************************************
		timer8(uint8_t, volatile uint8_t *, volatile uint8_t *, volatile uint8_t *, volatile uint8_t *, volatile uint8_t *, volatile uint8_t *);
		
		//Setters ********************************************************************
		void setAlias(uint8_t);
		
		void setPrescaler(uint16_t);
		void set(uint8_t);
		void reset();
		void hardReset();
		
		virtual void interruptServiceRoutine(void);
		virtual void enable(void);
		virtual void disable(void);
		virtual void clear(void);
		
		//Getters ********************************************************************
		int8_t setMode(t_mode, t_interrupt=t_interrupt::NONE, uint8_t=0x00);
		int8_t setMode(t_pwm);
		
		uint8_t getCount();
		uint16_t getNonResetCount();
		uint16_t getOverflowCount();		
		
	private:		
		// Timer operation settings.
		uint8_t _alias;
		uint16_t _prescale;
		
		t_mode _mode;
		t_interrupt _interrupt;
		t_pwm _pwm;
				
		// Registers.
		volatile uint8_t * _tcntx;			// TIMER COUNT
		volatile uint8_t * _tccrxa;			// PRESCALER
		volatile uint8_t * _tccrxb;			// PRESCALER
		volatile uint8_t * _timskx;			// Timer Interrupt Mask register.
		volatile uint8_t * _ocrxa;
		volatile uint8_t * _ocrxb;
		
		// Overflow.		
		uint16_t _interruptCount;
		uint16_t _overflowCount;			// TODO::remember number of overflows.
		uint16_t _nonResetCount;
		
		// Friend void.	
		friend void TIMER0_OVF_vect(void);
		friend void TIMER2_OVF_vect(void);
		
		// Modes.
		void setMode2Normal();
		void setMode2Ctc();
		void setMode2FastPwm();
		void setMode2PhaseCorrectPwm();		
		void setInterruptMode(t_interrupt, uint8_t);
		
		
};
#endif
