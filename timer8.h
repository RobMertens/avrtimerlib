#ifndef _TIMER8_H_
#define _TIMER8_H_

#include <stdint.h>
#include <avr/interrupt.h>
#include "settings.h"
#include "interrupt.h"

extern "C" void TIMER0_OVF_vect(void) __attribute__((signal));
extern "C" void TIMER2_OVF_vect(void) __attribute__((signal));
extern "C" void TIMER0_COMPA_vect(void) __attribute__((signal));
extern "C" void TIMER2_COMPA_vect(void) __attribute__((signal));
extern "C" void TIMER0_COMPB_vect(void) __attribute__((signal));
extern "C" void TIMER2_COMPB_vect(void) __attribute__((signal));

class interrupt;

class timer8 : private interrupt::handler
{
	public:
		//Constructors *************************************************************
		timer8(void);
		timer8(const t_alias);
		timer8(volatile uint8_t *, volatile uint8_t *, volatile uint8_t *, volatile uint8_t *, volatile uint8_t *, volatile uint8_t *);

		//Setters ******************************************************************
		void setCompareValueA(const uint8_t);
		void setCompareValueB(const uint8_t);
		void set(const uint8_t);
		void reset(void);
		void hardReset(void);
		virtual void interruptServiceRoutine(void);
		virtual void enable(void);
		virtual void disable(void);
		virtual void clear(void);

		//Getters ******************************************************************
		int8_t setAlias(const t_alias);
		int8_t initialize(const t_mode, const t_interrupt);
		int8_t initialize(const t_mode, const t_channel, const t_inverted);
		int8_t setPrescaler(const uint16_t);
		int8_t setDutyCycleA(float);
		int8_t setDutyCycleB(float);
		uint8_t getCount(void);
		uint32_t getOverflows(void);
		uint32_t getNonResetCount(void);
		t_alias	getAlias(void);
		t_mode getMode(void);
		t_interrupt	getInterruptMode(void);
		t_channel	getChannel(void);
		t_inverted getInverted(void);

	private:
		//Variables ****************************************************************
		t_alias _alias;
		t_mode _mode;
		t_interrupt _interrupt;
		t_channel _channel;
		t_inverted _inverted;
		uint16_t _prescale;
		uint32_t _overflows;

		//Registers ****************************************************************
		volatile uint8_t * _tcntx;			// TIMER COUNT
		volatile uint8_t * _tccrxa;			// PRESCALER
		volatile uint8_t * _tccrxb;			// PRESCALER
		volatile uint8_t * _timskx;			// Timer Interrupt Mask register.
		volatile uint8_t * _ocrxa;
		volatile uint8_t * _ocrxb;

		//Setters ******************************************************************
		void setRegistersT0(void);
		void setRegistersT2(void);
		void setMode2Normal(void);
		void setMode2Ctc(void);
		void setMode2FastPwm(void);
		void setMode2PhaseCorrectPwm(void);

		//Getters ******************************************************************
		int8_t setMode(const t_mode);
		int8_t setInterruptMode(const t_interrupt);
		int8_t setPwmChannel(const t_channel, const t_inverted);

		//Interrupt vectors ********************************************************
		static timer8 * _t8[7];
		friend void TIMER0_OVF_vect(void);
		friend void TIMER2_OVF_vect(void);
		friend void TIMER0_COMPA_vect(void);
		friend void TIMER2_COMPA_vect(void);
		friend void TIMER0_COMPB_vect(void);
		friend void TIMER2_COMPB_vect(void);

};
#endif
