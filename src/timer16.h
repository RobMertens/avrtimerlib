#ifndef _TIMER16_H_
#define _TIMER16_H_

#include <stdint.h>
#include <avr/interrupt.h>
#include "settings.h"
#include "interrupt.h"

extern "C" void TIMER1_OVF_vect(void) __attribute__ ((signal));
extern "C" void TIMER3_OVF_vect(void) __attribute__ ((signal));
extern "C" void TIMER4_OVF_vect(void) __attribute__ ((signal));
extern "C" void TIMER5_OVF_vect(void) __attribute__ ((signal));
extern "C" void TIMER1_COMPA_vect(void) __attribute__ ((signal));
extern "C" void TIMER3_COMPA_vect(void) __attribute__ ((signal));
extern "C" void TIMER4_COMPA_vect(void) __attribute__ ((signal));
extern "C" void TIMER5_COMPA_vect(void) __attribute__ ((signal));
extern "C" void TIMER1_COMPB_vect(void) __attribute__ ((signal));
extern "C" void TIMER3_COMPB_vect(void) __attribute__ ((signal));
extern "C" void TIMER4_COMPB_vect(void) __attribute__ ((signal));
extern "C" void TIMER5_COMPB_vect(void) __attribute__ ((signal));
extern "C" void TIMER1_COMPC_vect(void) __attribute__ ((signal));
extern "C" void TIMER3_COMPC_vect(void) __attribute__ ((signal));
extern "C" void TIMER4_COMPC_vect(void) __attribute__ ((signal));
extern "C" void TIMER5_COMPC_vect(void) __attribute__ ((signal));

class timer16 : private interrupt::handler
{
	public:
		//Constructors *************************************************************
		timer16(void);
		timer16(const t_alias);
		timer16(volatile uint8_t *, volatile uint8_t *, volatile uint8_t *, volatile uint8_t *, volatile uint8_t *, volatile uint8_t *, volatile uint8_t *, volatile uint8_t *, volatile uint8_t *);

		//Setters ******************************************************************
		void setCompareValueA(const uint16_t);
		void setCompareValueB(const uint16_t);
		void setCompareValueC(const uint16_t);
		void set(const uint16_t);
		void reset(void);
		void hardReset(void);
		virtual void interruptServiceRoutine(void); 																//ISR.
		virtual void enable(void);																									//Enable interrupts.
		virtual void disable(void);																									//Disable interrupts.
		virtual void clear(void);																										//Clear interrupts.

		//Getters ******************************************************************
		int8_t setAlias(const t_alias);
		int8_t initialize(const t_mode, const t_interrupt);
		int8_t initialize(const t_mode, const t_channel, const t_inverted);
		int8_t setPrescaler(const uint16_t);
		int8_t setDutyCycleA(float);
		int8_t setDutyCycleB(float);
		int8_t setDutyCycleC(float);
		uint16_t getCount(void);
		uint32_t getOverflows(void);
		uint32_t getNonResetCount(void);
		t_alias getAlias(void);
		t_mode getMode(void);
		t_interrupt getInterruptMode(void);
		t_channel getChannel(void);
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
		volatile uint8_t * _tcntxl;																									//Timer count low byte.
		volatile uint8_t * _tcntxh;																									//Timer count high byte.
		volatile uint8_t * _tccrxa;																									//Prescaler setting byte A.
		volatile uint8_t * _tccrxb;																									//Prescaler setting byte B.
		volatile uint8_t * _timskx;																									//Timer Interrupt Mask register.
		volatile uint8_t * _ocrxal;
		volatile uint8_t * _ocrxah;
		volatile uint8_t * _ocrxbl;
		volatile uint8_t * _ocrxbh;
		volatile uint8_t * _ocrxcl;
		volatile uint8_t * _ocrxch;

		//Setters ******************************************************************
		void setRegistersT1(void);
		void setRegistersT3(void);
		void setRegistersT4(void);
		void setRegistersT5(void);
		void setMode2Normal(void);
		void setMode2Ctc(void);
		void setMode2FastPwm(void);
		void setMode2PhaseCorrectPwm(void);
		void setMode2FrequencyCorrectPwm(void);

		//Getters ******************************************************************
		int8_t setMode(const t_mode);
		int8_t setInterruptMode(const t_interrupt);
		int8_t setPwmChannel(const t_channel, const t_inverted);

		//Interrupt vectors ********************************************************
		static timer16 * _t16[17];
		friend void TIMER1_OVF_vect(void);
		friend void TIMER3_OVF_vect(void);
		friend void TIMER4_OVF_vect(void);
		friend void TIMER5_OVF_vect(void);
		friend void TIMER1_COMPA_vect(void);
		friend void TIMER3_COMPA_vect(void);
		friend void TIMER4_COMPA_vect(void);
		friend void TIMER5_COMPA_vect(void);
		friend void TIMER1_COMPB_vect(void);
		friend void TIMER3_COMPB_vect(void);
		friend void TIMER4_COMPB_vect(void);
		friend void TIMER5_COMPB_vect(void);
		friend void TIMER1_COMPC_vect(void);
		friend void TIMER3_COMPC_vect(void);
		friend void TIMER4_COMPC_vect(void);
		friend void TIMER5_COMPC_vect(void);

};
#endif
