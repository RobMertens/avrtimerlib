#ifndef _TIMER16_H_
#define _TIMER16_H_

#include <avr/interrupt.h>
#include <interrupt.h>
#include <settings.h>

// ATMEGA2560
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
		//Constructors ***************************************************************
		timer16(t_alias);
		timer16();
		timer16(volatile uint8_t *, volatile uint8_t *, volatile uint8_t *, volatile uint8_t *, volatile uint8_t *, volatile uint8_t *, volatile uint8_t *, volatile uint8_t *, volatile uint8_t *);
		
		//Setters ********************************************************************
		void setCompareValueA(uint16_t);
		void setCompareValueB(uint16_t);
		void setCompareValueC(uint16_t);
		
		void set(uint16_t);
		void reset();
		void hardReset();
		
		virtual void interruptServiceRoutine(void);
		virtual void enable(void);
		virtual void disable(void);
		virtual void clear(void);
		
		//Getters ********************************************************************
		int8_t setAlias(t_alias);
		
		int8_t initialize(t_mode, t_interrupt, uint16_t=0x0000);
		int8_t initialize(t_mode, t_channel, bool);
		
		int8_t setPrescaler(uint16_t);
		
		int8_t setDutyCycleA(double);
		int8_t setDutyCycleB(double);
		int8_t setDutyCycleAB(double, double);
		
		uint16_t getCount();
		uint16_t getNonResetCount();
		uint16_t getOverflowCount();
		uint16_t getCompareCount();

		t_alias getAlias();
		t_mode getMode();
		t_interrupt getInterruptMode();	
		t_channel getPwmChannel();	
		
	private:		
		t_alias 	_alias;				// Timer operation settings.
		uint16_t 	_prescale;
		
		t_mode 		_mode;
		t_interrupt 	_interrupt;
		t_channel 	_channel;
		bool 		_inverted;
				
		// Registers.
		volatile uint8_t * _tcntxl;			// TIMER COUNT
		volatile uint8_t * _tcntxh;			// TIMER COUNT
		volatile uint8_t * _tccrxa;			// PRESCALER
		volatile uint8_t * _tccrxb;			// PRESCALER
		volatile uint8_t * _timskx;			// Timer Interrupt Mask register.
		volatile uint8_t * _ocrxal;
		volatile uint8_t * _ocrxah;
		volatile uint8_t * _ocrxbl;
		volatile uint8_t * _ocrxbh;
		volatile uint8_t * _ocrxcl;
		volatile uint8_t * _ocrxch;
		
		// Overflow.		
		uint16_t _overflowCount;
		uint16_t _compareCount;
		uint16_t _nonResetCount;
		
		// ALIAS.
		void setRegistersT1();
		void setRegistersT3();
		void setRegistersT4();
		void setRegistersT5();
		
		// Modes.
		int8_t setMode(t_mode);

		void setMode2Normal();				// Functions for NORMAL or CTC.
		void setMode2Ctc();	
		int8_t setInterruptMode(t_interrupt, uint16_t=0x0000);
		
		void setMode2FastPwm();				// Functions for PWM.
		void setMode2PhaseCorrectPwm();	
		void setMode2FrequencyCorrectPwm();
		int8_t setPwmChannel(t_channel, bool);
		
		// Static self.
		static timer16 * _t16[16];
		
		// Friend void.	
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
