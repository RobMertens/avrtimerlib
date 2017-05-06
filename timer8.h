#ifndef _TIMER8_H_
#define _TIMER8_H_

#include <avr/interrupt.h>
#include <cores/interrupt.h>
#include <cores/settings.h>

// ATMEGA2560
extern "C" void TIMER0_OVF_vect(void) __attribute__((signal));
extern "C" void TIMER2_OVF_vect(void) __attribute__((signal));

extern "C" void TIMER0_COMPA_vect(void) __attribute__((signal));
extern "C" void TIMER2_COMPA_vect(void) __attribute__((signal));

extern "C" void TIMER0_COMPB_vect(void) __attribute__((signal));
extern "C" void TIMER2_COMPB_vect(void) __attribute__((signal));

class timer8 : private interrupt::handler
{
	public:
		//Constructors ***************************************************************
		timer8(t_alias);
		timer8();		
		timer8(volatile uint8_t *, volatile uint8_t *, volatile uint8_t *, volatile uint8_t *, volatile uint8_t *, volatile uint8_t *);
		
		//Setters ********************************************************************
		void setCompareValueA(uint8_t);
		void setCompareValueB(uint8_t);
		
		void set(uint8_t);
		void reset();
		void hardReset();
		
		virtual void interruptServiceRoutine(void);
		virtual void enable(void);
		virtual void disable(void);
		virtual void clear(void);
		
		//Getters ********************************************************************
		int8_t setAlias(t_alias);
		
		int8_t initialize(t_mode, t_interrupt);
		int8_t initialize(t_mode, t_channel, t_inverted);
		
		int8_t setPrescaler(uint16_t);
		
		int8_t setDutyCycleA(float);
		int8_t setDutyCycleB(float);
		
		uint8_t getCount();
		uint32_t getOverflows();
		uint32_t getNonResetCount();
		
		t_alias		getAlias();
		t_mode		getMode();
		t_interrupt	getInterruptMode();	
		t_channel	getChannel();
		t_inverted	getInverted();
		
	private:		
		// Registers.
		volatile uint8_t * _tcntx;			// TIMER COUNT
		volatile uint8_t * _tccrxa;			// PRESCALER
		volatile uint8_t * _tccrxb;			// PRESCALER
		volatile uint8_t * _timskx;			// Timer Interrupt Mask register.
		volatile uint8_t * _ocrxa;
		volatile uint8_t * _ocrxb;

		// Timer operation settings.
		t_alias _alias;
		uint16_t _prescale;
		
		t_mode		_mode;
		t_interrupt	_interrupt;
		t_channel	_channel;
		t_inverted	_inverted;
		
		// Overflows.		
		uint32_t _overflows;
		
		// ALIAS.
		void setRegistersT0();
		void setRegistersT2();
		
		// Modes.
		int8_t setMode(t_mode);
		// Functions for NORMAL or CTC.
		void setMode2Normal();
		void setMode2Ctc();	
		int8_t setInterruptMode(t_interrupt);
		// Functions for PWM.
		void setMode2FastPwm();
		void setMode2PhaseCorrectPwm();	
		int8_t setPwmChannel(t_channel, t_inverted);
		
		static timer8 * _t8[7];
		
		// Friend void.	
		friend void TIMER0_OVF_vect(void);
		friend void TIMER2_OVF_vect(void);
		
		friend void TIMER0_COMPA_vect(void);
		friend void TIMER2_COMPA_vect(void);
		
		friend void TIMER0_COMPB_vect(void);
		friend void TIMER2_COMPB_vect(void);
		
};
#endif
