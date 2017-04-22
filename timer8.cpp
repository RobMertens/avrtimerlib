/******************************************************************************
 * avr-timer-lib
 * TIMER8.CPP
 *  
 * This file contains functions for 8bit avr-timers.
 *
 * TODO::auto pin-map w/ alias function for different arduino's (UNO/MEGA/...).
 *
 * @author:	Rob Mertens
 * @version: 	1.0.1 
 * @date: 	17/04/2017
 ******************************************************************************/

#include <timer8.h>

/*******************************************************************************
 * Constructor for a 16-bit timer, e.g.: timer0 on MEGA2560.
 * 
 * TODO::auto pin-map w/ alias function.
 *
 * @param: tccr The timer control register.
 * @param: tcnt The timer count register.
 ******************************************************************************/
timer8::timer8(t_alias alias)
{
	setAlias(alias);
	
	_overflowCount  = 0;
	_compareCount   = 0;
	_nonResetCount  = 0;	
}

/*******************************************************************************
 * Constructor for a 16-bit timer, e.g.: timer0 on MEGA2560.
 * 
 * TODO::auto pin-map w/ alias function.
 *
 * @param: tccr The timer control register.
 * @param: tcnt The timer count register.
 ******************************************************************************/
timer8::timer8()
{
	_alias = t_alias::NONE;
	
	_overflowCount  = 0;
	_compareCount   = 0;
	_nonResetCount  = 0;	
}

/*******************************************************************************
 * Constructor for a 8-bit timer, e.g.: timer0 on MEGA2560.
 * 
 * TODO::auto pin-map w/ alias function.
 *
 * @param: tccr The timer control register.
 * @param: tcnt The timer count register.
 ******************************************************************************/
timer8::timer8(volatile uint8_t * tccrxa, volatile uint8_t * tccrxb, volatile uint8_t * tcntx, volatile uint8_t * timskx, volatile uint8_t * ocrxa, volatile uint8_t * ocrxb)
{
	_tccrxa = tccrxa;						// Timer Count Control Register.
	_tccrxb = tccrxb;						// Timer Count Control Register.
	_tcntx  = tcntx;						// Timer Count register.
	_timskx = timskx;						// Timer Interrupt Mask register.
	_ocrxa  = ocrxa;
	_ocrxb  = ocrxb;
	
	_alias = t_alias::TX;
	
	_overflowCount  = 0;
	_compareCount   = 0;
	_nonResetCount  = 0;
	
	_t8[6] = this;							// Instance of itself for ISR.
}

/*******************************************************************************
 * Method for setting the timer alias and corresponding registers.
 *
 * @param: alias The timer alias.
 ******************************************************************************/
int8_t timer8::setAlias(t_alias alias)
{
	int8_t ret = 0;
	
	_alias = t_alias::NONE;
	
	switch(alias)
	{
		case t_alias::T0 :
			setRegistersT0();
			break;
		
		case t_alias::T2 :
			setRegistersT2();
			break;
		
		case t_alias::NONE :
		case t_alias::T1 :
		case t_alias::T3 :
		case t_alias::T4 :
		case t_alias::T5 :		
		case t_alias::TX :
		default :
			ret = -1;
			return ret;
	}
	
	_alias = alias;
	
	return ret;
}

/*******************************************************************************
 * Method for setting the registers as T0 (atmega2560).
 ******************************************************************************/
void timer8::setRegistersT0()
{
	_tccrxa = (volatile uint8_t *)0x44;
	_tccrxb = (volatile uint8_t *)0x45;
	_tcntx  = (volatile uint8_t *)0x46;
	_timskx = (volatile uint8_t *)0x6E;
	_ocrxa  = (volatile uint8_t *)0x47;
	_ocrxb  = (volatile uint8_t *)0x48;
}

/*******************************************************************************
 * Method for setting the registers as T2 (atmega2560).
 ******************************************************************************/
void timer8::setRegistersT2()
{
	_tccrxa = (volatile uint8_t *)0xB0;
	_tccrxb = (volatile uint8_t *)0xB1;
	_tcntx  = (volatile uint8_t *)0xB2;
	_timskx = (volatile uint8_t *)0x70;
	_ocrxa  = (volatile uint8_t *)0xB3;
	_ocrxb  = (volatile uint8_t *)0xB4;
}

/*******************************************************************************
 * Method for setting the timer mode 2 NORMAL or CTC. This method cannot set
 * mode 2 PWM.
 *
 * @param: mode The timer operation mode, only NORMAL or CTC.
 * @param: interrupt The interrupt mode, default is NONE.
 * @param: compare TODO
 ******************************************************************************/
int8_t timer8::initialize(t_mode mode, t_interrupt interrupt, uint8_t compare)
{
	int8_t ret = 0;
	
	//t_mode
	if(mode==t_mode::NORMAL or mode==t_mode::CTC)ret = setMode(mode);
	if(ret==-1)return ret;
	
	//t_channel.
	ret = setPwmChannel(t_channel::NONE, false);
	if(ret==-1)return ret;
	
	//t_interrupt.
	ret = setInterruptMode(interrupt);
	if(ret==-1)return ret;
	
	//Compare value.
	if(_interrupt==t_interrupt::COMPA)setCompareValueA(compare);
	else if(_interrupt==t_interrupt::COMPB)setCompareValueB(compare);
	else{ret=-1;}
	
	return ret;
}

/*******************************************************************************
 * Method for setting the timer mode 2 PWM. This method cannot set mode 2 NORMAL
 * or CTC.
 *
 * @param: mode The timer operation mode, only PWM.
 * @param: interrupt The interrupt mode, default is NONE.
 * @param: inverted TODO
 ******************************************************************************/
int8_t timer8::initialize(t_mode mode, t_channel channel, bool inverted)
{
	int8_t ret = 0;
	
	//t_mode
	if(mode==t_mode::PWM_F or mode==t_mode::PWM_PC)ret = setMode(mode);
	if(ret==-1)return ret;
	
	//t_interrupt.
	ret = setInterruptMode(t_interrupt::NONE);
	if(ret==-1)return ret;
	
	//t_channel.
	ret = setPwmChannel(channel, inverted);
	
	return ret;
}

/*******************************************************************************
 * Method for setting the timer mode.
 *
 * @param: mode The timer operation mode.
 ******************************************************************************/
int8_t timer8::setMode(t_mode mode)
{
	int8_t ret = 0;
	
	//Non-operational mode.
	setPrescaler(0);
	_mode = t_mode::NONE;
	
	//MODES.
	switch(mode)
	{
		case t_mode::NORMAL :
			setMode2Normal();
			break;
	
		case t_mode::CTC :
			setMode2Ctc();
			break;
		
		case t_mode::PWM_F :
			setMode2FastPwm();
			break;
	
		case t_mode::PWM_PC :
			setMode2PhaseCorrectPwm();
			break;
		
		case t_mode::NONE :
			//Nothing.
			break;
		
		case t_mode::PWM_FC :
		default :
			ret = -1;
			return ret;
	}

	return ret;
}

/*******************************************************************************
 * Private method for setting the timer mode 2 NORMAL.
 ******************************************************************************/
void timer8::setMode2Normal()
{
	//Normal mode.
	*_tccrxa &= 0x0C;					// WGMx0  = 0;
								// WGMx1  = 0;
								// COMxB0 = 0;
								// COMxB1 = 0;
								// COMxA0 = 0;
								// COMxA1 = 0;
	*_tccrxb &= ~(1 << 3); 					// WGMx2  = 0;
	
	//Set var.
	_mode = t_mode::NORMAL;
}

/*******************************************************************************
 * Private method for setting the timer mode 2 CTC.
 ******************************************************************************/
void timer8::setMode2Ctc()
{
	//CTC Mode.
	*_tccrxa &= 0x0C;					// WGMx0  = 0;
	*_tccrxa |=  (1 << 2);					// WGMx1  = 1;
								// COMxB0 = 0;
								// COMxB1 = 0;
								// COMxA0 = 0;
								// COMxA1 = 0;
	*_tccrxb &= ~(1 << 3); 					// WGMx2  = 0;
	
	//Set var.
	_mode = t_mode::CTC;
}

/*******************************************************************************
 * Method for setting the timer interrupt mode.
 *
 * @param: interrupt The timer interrupt mode.
 ******************************************************************************/
int8_t timer8::setInterruptMode(t_interrupt interrupt)
{
	int8_t ret = 0;
	
	_interrupt = t_interrupt::NONE;
	*_timskx = 0x00;
	
	//INTERRUPT MODE.
	switch(interrupt)
	{
		case t_interrupt::NONE : 
			//Nothing.
			break;
	
		case t_interrupt::OVF : 
			*_timskx = 0x01;
			if(_alias==t_alias::T0)_t8[0] = this;
			else{_t8[1] = this;}
			break;
	
		case t_interrupt::COMPA :
			*_timskx = 0x02;
			if(_alias==t_alias::T0)_t8[2] = this;
			else{_t8[3] = this;}
			break;
	
		case t_interrupt::COMPB :
			*_timskx = 0x04;
			if(_alias==t_alias::T0)_t8[4] = this;
			else{_t8[5] = this;}
			break;
		
		case t_interrupt::COMPC :
		default :
			ret = -1;
			return ret;
	}
	
	_interrupt = interrupt;
	
	return ret;
}

/*******************************************************************************
 * Private method for setting the timer mode 2 fast PWM.
 ******************************************************************************/
void timer8::setMode2FastPwm()
{
	//Fast PWM.
	*_tccrxa &= 0x0C;					// WGMx0  = 1;
	*_tccrxa |=  (1 << 0);					// WGMx1  = 1;
	*_tccrxa |=  (1 << 1);					// COMxB0 = 0;
								// COMxB1 = 0;
								// COMxA0 = 0;
								// COMxA1 = 0;
	//*_tccrxb |=  (1 << 3); 					// WGMx2  = 1;
	*_tccrxb &= ~(1 << 3); 
	
	//Set var.
	_mode = t_mode::PWM_F;
}

/*******************************************************************************
 * Private method for setting the timer mode 2 phase correct PWM.
 ******************************************************************************/
void timer8::setMode2PhaseCorrectPwm()
{
	//Phase Correct PWM.
	*_tccrxa &= 0x0C;					// WGMx0  = 1;
	*_tccrxa |=  (1 << 0);					// WGMx1  = 0;
	*_tccrxa &= ~(1 << 1);					// COMxB0 = 0;
								// COMxB1 = 0;
								// COMxA0 = 0;
								// COMxA1 = 0;
	//*_tccrxb |=  (1 << 3); 					// WGMx2  = 1;
	*_tccrxb &= ~(1 << 3); 
	
	//Set var.
	_mode = t_mode::PWM_PC;
}

/*******************************************************************************
 * Method for setting the timer interrupt mode.
 *
 * @param: interrupt The timer interrupt mode.
 ******************************************************************************/
int8_t timer8::setPwmChannel(t_channel channel, bool inverted)
{
	int8_t ret = 0;
	
	_channel = t_channel::NONE;
	*_tccrxa &= 0x0F;

	//PWM channel.
	switch(channel)
	{
		case t_channel::NONE : 
			//Nothing.
			break;
	
		case t_channel::A : 
			if(!inverted)*_tccrxa |= 0x80;
			else{*_tccrxa |= 0xC0;}
			break;
	
		case t_channel::B :
			if(!inverted)*_tccrxa |= 0x20;
			else{*_tccrxa |= 0x30;}
			break;
	
		case t_channel::AB :
			if(!inverted)*_tccrxa |= 0xA0;
			else{*_tccrxa |= 0xF0;}
			break;
		
		case t_channel::C :
		case t_channel::BC :
		case t_channel::AC :
		case t_channel::ABC :
		default :
			ret = -1;
			return ret;
	}
	
	_channel = channel;
	_inverted = inverted;
	
	return ret;
}

/*******************************************************************************
 * Method for setting the timer prescaler value.
 * 
 *    VALUE | TIMER 0 | TIMER 2
 *     	  1 |    x    |   x
 * 	  8 |    x    |   x
 *       32 |	      |   x
 *       64 |    x    |   x
 *      256 |    x    |   
 *     1024 |    x    |
 * 
 * TODO::don't allow prescaler with wrong timer.
 * 
 * @param: prescale The prescaler value.   
 ******************************************************************************/
int8_t timer8::setPrescaler(uint16_t prescale)
{
	int8_t ret = 0;
	
	_prescale = 0;
	*_tccrxb &= 0xF8;
	
	if(_alias!=t_alias::T0 or _alias!=t_alias::T2)
	{
		ret = -1;
		return ret;
	}
	
	switch(prescale)
	{
		case 0 :
			//Timer inactive.
			//Nothing.
			break;
		
		case 1 :
			*_tccrxb |= 0x01;
			break;
		
		case 8 :
			*_tccrxb |= 0x02;
			break;
		
		case 32 :
			if(_alias==t_alias::T2)*_tccrxb |= 0x03;
			else{ret=-1;return ret;}
			break;
		
		case 64 :
			if(_alias==t_alias::T0)*_tccrxb |= 0x03;
			else{*_tccrxb |= 0x04;}
			break;
			
		case 256 :
			if(_alias==t_alias::T0)*_tccrxb |= 0x04;
			else{ret=-1;return ret;}
			break;
	
		case 1024 :
			if(_alias==t_alias::T0)*_tccrxb |= 0x05;
			else{ret=-1;return ret;}
			break;
		
		default :
			ret = -1;
			return ret;
	}
	
	_prescale = prescale;
	
	return ret;
}

/*******************************************************************************
 * Method for setting the OCRxA register.
 *
 * @param compare The compare value (unsigned byte).
 ******************************************************************************/
void timer8::setCompareValueA(uint8_t compare)
{
	*_ocrxa = compare;
}

/*******************************************************************************
 * Method for setting the OCRxB register.
 *
 * @param compare The compare value (unsigned byte).
 ******************************************************************************/
void timer8::setCompareValueB(uint8_t compare)
{
	*_ocrxb = compare;
}

/*******************************************************************************
 * 
 ******************************************************************************/
int8_t timer8::setDutyCycleA(double dutyCycle)
{
	int8_t ret = 0;
	uint8_t compare = 0;
	
	if(_mode==t_mode::PWM_F or _mode==t_mode::PWM_PC)
	{
		if(dutyCycle > 1.0)dutyCycle=1.0;
		else if(dutyCycle < 0.0)dutyCycle=0.0;
		
		compare = (uint8_t)(dutyCycle*0xFF - 1);
		
		setCompareValueA(compare);
	}
	else
	{
		ret = -1;
	}
	
	return ret;
}

/*******************************************************************************
 * 
 ******************************************************************************/
int8_t timer8::setDutyCycleB(double dutyCycle)
{
	int8_t ret = 0;
	uint8_t compare = 0;
	
	if(_mode==t_mode::PWM_F or _mode==t_mode::PWM_PC)
	{
		if(dutyCycle > 1.0)dutyCycle=1.0;
		else if(dutyCycle < 0.0)dutyCycle=0.0;
		
		compare = (uint8_t)(dutyCycle*0xFF - 1);
		
		setCompareValueB(compare);
	}
	else
	{
		ret = -1;
	}
	
	return ret;
}

/*******************************************************************************
 * 
 ******************************************************************************/
int8_t timer8::setDutyCycleAB(double dutyCycleA, double dutyCycleB)
{
	//TODO::
	
	return -1;
}

/*******************************************************************************
 * 
 ******************************************************************************/
void timer8::set(uint8_t value)
{
	*_tcntx = value;
}

/*******************************************************************************
 * TODO::clear interrupt bits.
 ******************************************************************************/
void timer8::reset()
{
	set(0x00);
}

/*******************************************************************************
 * TODO::clear interrupt bits.
 ******************************************************************************/
void timer8::hardReset()
{
	setMode(t_mode::NORMAL);
	setInterruptMode(t_interrupt::NONE);
	setPrescaler(0);
	reset();
}

/*******************************************************************************
 * 
 ******************************************************************************/
uint8_t timer8::getCount()
{
	return *_tcntx;
}

/*******************************************************************************
 * Method for obtaining the total summized count since the last reset. Thus
 * overflows are accounted.
 * 
 * REMARK: This method only works if the timer interrupts with an timer_ovf_vect()
 * 
 * @return: _nonResetCount The count value since last reset.
 ******************************************************************************/
uint16_t timer8::getNonResetCount()
{
	//TODO::calculations.
	
	return _nonResetCount;
}

/*******************************************************************************
 * 
 ******************************************************************************/
uint16_t timer8::getOverflowCount()
{
	return _overflowCount;
}

/*******************************************************************************
 * 
 ******************************************************************************/
uint16_t timer8::getCompareCount()
{
	return _compareCount;
}

/*******************************************************************************
 * 
 ******************************************************************************/
t_alias timer8::getAlias()
{
	return _alias;
}

/*******************************************************************************
 * 
 ******************************************************************************/
t_mode timer8::getMode()
{
	return _mode;
}

/*******************************************************************************
 * 
 ******************************************************************************/
t_interrupt timer8::getInterruptMode()
{
	return _interrupt;
}

/*******************************************************************************
 * 
 ******************************************************************************/
t_channel timer8::getPwmChannel()
{
	return _channel;
}

/*******************************************************************************
 * ISR for the timer class.
 ******************************************************************************/
void timer8::interruptServiceRoutine(void)
{
	if(_interrupt==t_interrupt::OVF)
	{
		_overflowCount++;
	}
	else if(_interrupt==t_interrupt::COMPA or _interrupt==t_interrupt::COMPB)
	{
		_compareCount++;
	}
}

/*******************************************************************************
 * Enable timer interrupts.
 ******************************************************************************/
void timer8::enable(void)
{
	//TODO::
}

/*******************************************************************************
 * Disable timer interrupts.
 ******************************************************************************/
void timer8::disable(void)
{
	//TODO::
}

/*******************************************************************************
 * Clear timer interrupts.
 ******************************************************************************/
void timer8::clear(void)
{
	//TODO::
}

/*******************************************************************************
 * Global forward declaration.
 ******************************************************************************/
timer8 * timer8::_t8[6] = {};

/*******************************************************************************
 * OVF -> ISR().
 * 
 * TODO::different avrs.
 * 
 * Call from self.
 ******************************************************************************/
#define TIMER_OVF(t, n)								\
ISR(TIMER ## t ## _OVF_vect)							\
{										\
	if(timer8::_t8[n])timer8::_t8[n] -> interruptServiceRoutine();		\
}

#if defined(TIMER0_OVF_vect)
//TIMER_OVF(0, 0)
#endif
#if defined(TIMER2_OVF_vect)
TIMER_OVF(2, 1)
#endif

/*******************************************************************************
 * COMPA -> ISR().
 * 
 * TODO::different avrs.
 * 
 * Call from self.
 ******************************************************************************/
#define TIMER_COMPA(t, n)							\
ISR(TIMER ## t ## _COMPA_vect)							\
{										\
	if(timer8::_t8[n])timer8::_t8[n] -> interruptServiceRoutine();		\
}

#if defined(TIMER0_COMPA_vect)
TIMER_COMPA(0, 2)
#endif
#if defined(TIMER2_COMPA_vect)
TIMER_COMPA(2, 3)
#endif

/*******************************************************************************
 * COMPB -> ISR().
 * 
 * TODO::different avrs.
 * 
 * Call from self.
 ******************************************************************************/
#define TIMER_COMPB(t, n)							\
ISR(TIMER ## t ## _COMPB_vect)							\
{										\
	if(timer8::_t8[n])timer8::_t8[n] -> interruptServiceRoutine();		\
}

#if defined(TIMER0_COMPB_vect)
TIMER_COMPB(0, 4)
#endif
#if defined(TIMER2_COMPB_vect)
TIMER_COMPB(2, 5)
#endif
 
