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

#include "timer8.h"

/*******************************************************************************
 * Constructor for a 16-bit timer, e.g.: timer0 on MEGA2560.
 *
 * TODO::auto pin-map w/ alias function.
 *
 * @param: tccr The timer control register.
 * @param: tcnt The timer count register.
 ******************************************************************************/
timer8::timer8(void)
{
	_alias = t_settings::alias::NONE;

	_overflows = 0;
}

/*******************************************************************************
 * Constructor for a 16-bit timer, e.g.: timer0 on MEGA2560.
 *
 * TODO::auto pin-map w/ alias function.
 *
 * @param: tccr The timer control register.
 * @param: tcnt The timer count register.
 ******************************************************************************/
timer8::timer8(const t_settings::alias& alias)
{
	setAlias(alias);

	_overflows = 0;
}

/*******************************************************************************
 * Constructor for a 8-bit timer, e.g.: timer0 on MEGA2560.
 *
 * TODO::auto pin-map w/ alias function.
 *
 * @param: tccr The timer control register.
 * @param: tcnt The timer count register.
 ******************************************************************************/
timer8::timer8(volatile uint8_t const * const& tccrxa,
							 volatile uint8_t const * const& tccrxb,
							 volatile uint8_t const * const& tcntx,
							 volatile uint8_t const * const& timskx,
							 volatile uint8_t const * const& ocrxa,
							 volatile uint8_t const * const& ocrxb)
{
	_tccrxa = tccrxa;						// Timer Count Control Register.
	_tccrxb = tccrxb;						// Timer Count Control Register.
	_tcntx  = tcntx;						// Timer Count register.
	_timskx = timskx;						// Timer Interrupt Mask register.
	_ocrxa  = ocrxa;
	_ocrxb  = ocrxb;

	_alias = t_settings::alias::TX;

	_overflows = 0;

	_t8[6] = this;							// Instance of itself for ISR.
}

/*******************************************************************************
 * Method for setting the timer alias and corresponding registers.
 *
 * @param: alias The timer alias.
 ******************************************************************************/
int8_t timer8::setAlias(const t_settings::alias& alias)
{
	int8_t ret = 0;

	_alias = t_settings::alias::NONE;

	switch(alias)
	{
		case t_settings::alias::T0 :
			setRegistersT0();
			_t8[0] = this;
			break;

		case t_settings::alias::T2 :
			setRegistersT2();
			_t8[1] = this;
			break;

		case t_settings::alias::NONE :
		case t_settings::alias::T1 :
		case t_settings::alias::T3 :
		case t_settings::alias::T4 :
		case t_settings::alias::T5 :
		case t_settings::alias::TX :
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
void timer8::setRegistersT0(void)
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
void timer8::setRegistersT2(void)
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
int8_t timer8::initialize(const t_settings::mode& mode,
													const t_settings::interrupt& interrupt)
{
	int8_t ret = 0;

	//t_settings::mode
	if(mode==t_settings::mode::NORMAL or mode==t_settings::mode::CTC)ret = setMode(mode);
	else{ret=-1;}
	if(ret==-1)return ret;

	//t_settings::channel.
	ret = setPwmChannel(t_settings::channel::NONE, t_settings::inverted::NONE);
	if(ret==-1)return ret;

	//t_settings::interrupt.
	ret = setInterruptMode(interrupt);
	if(ret==-1)return ret;

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
int8_t timer8::initialize(const t_settings::mode& mode,
													const t_settings::channel& channel,
													const t_settings::inverted& inverted)
{
	int8_t ret = 0;

	//t_settings::mode
	if(mode==t_settings::mode::PWM_F or mode==t_settings::mode::PWM_PC)ret = setMode(mode);
	else{ret=-1;}
	if(ret==-1)return ret;

	//t_settings::interrupt.
	ret = setInterruptMode(t_settings::interrupt::NONE);
	if(ret==-1)return ret;

	//t_settings::channel.
	ret = setPwmChannel(channel, inverted);

	return ret;
}

/*******************************************************************************
 * Method for setting the timer mode.
 *
 * @param: mode The timer operation mode.
 ******************************************************************************/
int8_t timer8::setMode(const t_settings::mode& mode)
{
	int8_t ret = 0;

	//Non-operational mode.
	setPrescaler(0);
	_mode = t_settings::mode::NONE;

	//MODES.
	switch(mode)
	{
		case t_settings::mode::NORMAL :
			setMode2Normal();
			break;

		case t_settings::mode::CTC :
			setMode2Ctc();
			break;

		case t_settings::mode::PWM_F :
			setMode2FastPwm();
			break;

		case t_settings::mode::PWM_PC :
			setMode2PhaseCorrectPwm();
			break;

		case t_settings::mode::NONE :
			//Nothing.
			break;

		case t_settings::mode::PWM_FC :
		default :
			ret = -1;
			return ret;
	}

	return ret;
}

/*******************************************************************************
 * Private method for setting the timer mode 2 NORMAL.
 ******************************************************************************/
void timer8::setMode2Normal(void)
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
	_mode = t_settings::mode::NORMAL;
}

/*******************************************************************************
 * Private method for setting the timer mode 2 CTC.
 ******************************************************************************/
void timer8::setMode2Ctc(void)
{
	//CTC Mode.
	*_tccrxa &= 0x0C;					// WGMx0  = 0;
	*_tccrxa |=  (1 << 1);					// WGMx1  = 1;
								// COMxB0 = 0;
								// COMxB1 = 0;
								// COMxA0 = 0;
								// COMxA1 = 0;
	*_tccrxb &= ~(1 << 3); 					// WGMx2  = 0;

	//Set var.
	_mode = t_settings::mode::CTC;
}

/*******************************************************************************
 * Method for setting the timer interrupt mode.
 *
 * @param: interrupt The timer interrupt mode.
 ******************************************************************************/
int8_t timer8::setInterruptMode(const t_settings::interrupt& interrupt)
{
	int8_t ret = 0;

	//RESET VARS.
	_interrupt = t_settings::interrupt::NONE;
	*_timskx = 0x00;
	_t8[7] = {};

	//CHECK ALIAS.
	if(_alias!=t_settings::alias::T0 and _alias!=t_settings::alias::T2 and _alias!=t_settings::alias::TX)
	{
		ret = -1;
		return ret;
	}

	//INTERRUPT MODE.
	switch(interrupt)
	{
		case t_settings::interrupt::NONE :
			//Nothing.
			break;

		case t_settings::interrupt::OVF :
			*_timskx = 0x01;
			if(_alias==t_settings::alias::T0)_t8[0]=this;
			else{_t8[1]=this;}
			break;

		case t_settings::interrupt::COMPA :
			*_timskx = 0x02;
			if(_alias==t_settings::alias::T0)_t8[2]=this;
			else{_t8[3]=this;}
			break;

		case t_settings::interrupt::COMPB :
			*_timskx = 0x04;
			if(_alias==t_settings::alias::T0)_t8[4]=this;
			else{_t8[5]=this;}
			break;

		case t_settings::interrupt::COMPC :
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
void timer8::setMode2FastPwm(void)
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
	_mode = t_settings::mode::PWM_F;
}

/*******************************************************************************
 * Private method for setting the timer mode 2 phase correct PWM.
 ******************************************************************************/
void timer8::setMode2PhaseCorrectPwm(void)
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
	_mode = t_settings::mode::PWM_PC;
}

/*******************************************************************************
 * Method for setting the timer interrupt mode.
 *
 * @param: interrupt The timer interrupt mode.
 ******************************************************************************/
int8_t timer8::setPwmChannel(const t_settings::channel& channel,
														 const t_settings::inverted& inverted)
{
	int8_t ret = 0;

	_channel = t_settings::channel::NONE;
	*_tccrxa &= 0x0F;
	*_tccrxb &= 0xFB;

	//PWM channel.
	switch(channel)
	{
		//No PWM channel.
		case t_settings::channel::NONE :
			//Nothing.
			break;

		//Channel A.
		case t_settings::channel::A :
			if(inverted==t_settings::inverted::NORMAL)*_tccrxa |= 0x80;
			else{*_tccrxa |= 0xC0;}
			break;

		//Channel B.
		case t_settings::channel::B :
			if(inverted==t_settings::inverted::NORMAL)*_tccrxa |= 0x20;
			else{*_tccrxa |= 0x30;}
			break;

		//Channel B w/ specified TOP-value.
		case t_settings::channel::B_TOP :
			*_tccrxb |= 0x04;
			if(inverted==t_settings::inverted::NORMAL)*_tccrxa |= 0xA0;
			else{*_tccrxa |= 0xF0;}
			break;

		//Channel A & B.
		case t_settings::channel::AB :
			if(inverted==t_settings::inverted::NORMAL)*_tccrxa |= 0xA0;
			else{*_tccrxa |= 0xF0;}
			break;

		//Invalid options.
		case t_settings::channel::C :
		case t_settings::channel::C_TOP :
		case t_settings::channel::AC :
		case t_settings::channel::BC :
		case t_settings::channel::BC_TOP :
		case t_settings::channel::ABC :
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
 * 	  		8 |    x    |   x
 *       32 |	      	|   x
 *       64 |    x    |   x
 *      256 |    x    |
 *     1024 |    x    |
 *
 * TODO::don't allow prescaler with wrong timer.
 *
 * @param: prescale The prescaler value.
 ******************************************************************************/
int8_t timer8::setPrescaler(const uint16_t& prescale)
{
	int8_t ret = 0;

	_prescale = 0;
	*_tccrxb &= 0xF8;

	if(_alias!=t_settings::alias::T0 and _alias!=t_settings::alias::T2)
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
			if(_alias==t_settings::alias::T2)*_tccrxb |= 0x03;
			else{ret=-1;return ret;}
			break;

		case 64 :
			if(_alias==t_settings::alias::T0)*_tccrxb |= 0x03;
			else{*_tccrxb |= 0x04;}
			break;

		case 256 :
			if(_alias==t_settings::alias::T0)*_tccrxb |= 0x04;
			else{ret=-1;return ret;}
			break;

		case 1024 :
			if(_alias==t_settings::alias::T0)*_tccrxb |= 0x05;
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
void timer8::setCompareValueA(const uint8_t& compare)
{
	*_ocrxa = compare;
}

/*******************************************************************************
 * Method for setting the OCRxB register.
 *
 * @param compare The compare value (unsigned byte).
 ******************************************************************************/
void timer8::setCompareValueB(const uint8_t& compare)
{
	*_ocrxb = compare;
}

/*******************************************************************************
 *
 ******************************************************************************/
int8_t timer8::setDutyCycleA(float& dutyCycle)
{
	int8_t ret = 0;
	uint8_t compare = 0;

	if(_mode==t_settings::mode::PWM_F or _mode==t_settings::mode::PWM_PC)
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
int8_t timer8::setDutyCycleB(float& dutyCycle)
{
	int8_t ret = 0;
	uint8_t compare = 0;

	if(_mode==t_settings::mode::PWM_F or _mode==t_settings::mode::PWM_PC)
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
void timer8::set(const uint8_t& value)
{
	*_tcntx = value;
}

/*******************************************************************************
 * TODO::clear interrupt bits.
 ******************************************************************************/
void timer8::reset(void)
{
	set(0x00);
}

/*******************************************************************************
 * TODO::clear interrupt bits.
 ******************************************************************************/
void timer8::hardReset(void)
{
	if(_interrupt!=t_settings::interrupt::NONE)setInterruptMode(t_settings::interrupt::NONE);
	if(_channel!=t_settings::channel::NONE)setPwmChannel(t_settings::channel::NONE, t_settings::inverted::NONE);
	setMode(t_settings::mode::NORMAL);
	reset();
}

/*******************************************************************************
 *
 ******************************************************************************/
uint8_t timer8::getCount(void)
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
uint32_t timer8::getNonResetCount(void)
{
	uint8_t count = getCount();
	uint8_t top = 0xFE;
	uint32_t nonResetCount = 0x00000000;

	//TODO::set the top value according to the timer mode.
	//if(_channel==t_settings::channel::B_TOP or _channel==t_settings::channel::C_TOP or _channel==t_settings::channel::BC_TOP)
	nonResetCount = (_overflows*(uint32_t)top) | ((0x0000 << 8) | count);

	return nonResetCount;
}

/*******************************************************************************
 *
 ******************************************************************************/
uint32_t timer8::getOverflows(void)
{
	return _overflows;
}

/*******************************************************************************
 *
 ******************************************************************************/
t_settings::alias timer8::getAlias(void)
{
	return _alias;
}

/*******************************************************************************
 *
 ******************************************************************************/
t_settings::mode timer8::getMode(void)
{
	return _mode;
}

/*******************************************************************************
 *
 ******************************************************************************/
t_settings::interrupt timer8::getInterruptMode(void)
{
	return _interrupt;
}

/*******************************************************************************
 *
 ******************************************************************************/
t_settings::channel timer8::getChannel(void)
{
	return _channel;
}

/*******************************************************************************
 *
 ******************************************************************************/
t_settings::inverted timer8::getInverted(void)
{
	return _inverted;
}

/*******************************************************************************
 * ISR for the timer class.
 ******************************************************************************/
void timer8::interruptServiceRoutine(void)
{
	_overflows++;
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
timer8 * timer8::_t8[7] = {};

/*******************************************************************************
 * __vector_x -> ISR().
 *
 * TODO::different avrs.
 *
 * Call from self.
 ******************************************************************************/
#define TIMER_ISR(t, vect, n)																		\
ISR(TIMER ## t ## _ ## vect ## _vect)														\
{																																\
	if(timer8::_t8[n])timer8::_t8[n] -> interruptServiceRoutine();\
}

//OVERFLOW.
#if not defined(ARDUINO)
	#if defined(TIMER0_OVF_vect)
	TIMER_ISR(0, OVF, 0)		//Used by arduino.
	#endif
#endif
#if defined(TIMER2_OVF_vect)
TIMER_ISR(2, OVF, 1)
#endif

//COMPARE A.
#if defined(TIMER0_COMPA_vect)
TIMER_ISR(0, COMPA, 2)
#endif
#if defined(TIMER2_COMPA_vect)
TIMER_ISR(2, COMPA, 3)
#endif

//COMPARE B.
#if defined(TIMER0_COMPB_vect)
TIMER_ISR(0, COMPB, 4)
#endif
#if defined(TIMER2_COMPB_vect)
TIMER_ISR(2, COMPB, 5)
#endif
