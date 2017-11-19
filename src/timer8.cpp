/******************************************************************************
 * avr-timer-lib
 * Timer8.cpp
 *
 * This file contains functions for 8bit avr-timers.
 *
 * TODO::auto pin-map w/ alias function for different arduino's (UNO/MEGA/...).
 *
 * @author:		Rob Mertens
 * @version: 	1.1.1
 * @date: 		17/04/2017
 ******************************************************************************/

//Include library headers.
#include "timer8.hpp"

namespace avr
{

/*******************************************************************************
 * Constructor for a 16-bit timer, e.g.: timer0 on MEGA2560.
 *
 * TODO::auto pin-map w/ alias function.
 *
 * @param: tccr The timer control register.
 * @param: tcnt The timer count register.
 ******************************************************************************/
Timer8::Timer8(void) : Timer()
{
	alias_ = t_alias::NONE;

	overflows_ = 0;
}

/*******************************************************************************
 * Constructor for a 16-bit timer, e.g.: timer0 on MEGA2560.
 *
 * TODO::auto pin-map w/ alias function.
 *
 * @param: tccr The timer control register.
 * @param: tcnt The timer count register.
 ******************************************************************************/
Timer8::Timer8(const t_alias alias) : Timer()
{
	setAlias(alias);

	overflows_ = 0;
}

/*******************************************************************************
 * Constructor for a 8-bit timer, e.g.: timer0 on MEGA2560.
 *
 * TODO::auto pin-map w/ alias function.
 *
 * @param: tccr The timer control register.
 * @param: tcnt The timer count register.
 ******************************************************************************/
Timer8::Timer8(volatile uint8_t const * const& tccrxa,
	volatile uint8_t const * const& tccrxb, volatile uint8_t const * const& tcntx,
	volatile uint8_t const * const& timskx, volatile uint8_t const * const& ocrxa,
	volatile uint8_t const * const& ocrxb) : Timer()
{
	tccrxa_ = tccrxa;						// Timer Count Control Register.
	tccrxb_ = tccrxb;						// Timer Count Control Register.
	tcntx_  = tcntx;						// Timer Count register.
	timskx_ = timskx;						// Timer Interrupt Mask register.
	ocrxa_  = ocrxa;
	ocrxb_  = ocrxb;

	alias_ = t_alias::TX;

	overflows_ = 0;

	__T8__[6] = this;							// Instance of itself for ISR.
}

Timer8::~Timer8(void)
{
	//Free the register pointers.
	delete tccrxa_;
	delete tccrxb_;
	delete tcntx_;
	delete timskx_;
	delete ocrxa_;
	delete ocrxb_;

	//Delete this instance from static list.
	for(uint8_t n = 0; n < 7; ++n)
	{
		if(this == __T8__[n])delete __T8__[n];
	}
}

Timer8& Timer8::operator=(const Timer8& other)
{
	if(this != &other)
	{
		//Timer settings.
		alias_ = other.alias_;
		mode_ = other.mode_;
		interrupt_ = other.interrupt_;
		channel_ = other.channel_;
		inverted_ = other.inverted_;
		prescale_ = other.prescale_;

		//Timer runtime variables.
		overflows_ = other.overflows_;

		//Free the register pointers.
		uint8_t *tccrxa(new volatile uint8_t(*other.tccrxa_));
		delete tccrxa_;
		tccrxa_ = tccrxa;
		uint8_t *tccrxb(new volatile uint8_t(*other.tccrxb_));
		delete tccrxb_;
		tccrxb_ = tccrxb;
		uint8_t *tcntx(new volatile uint8_t(*other.tcntx_));
		delete tcntx_;
		tcntx_ = tcntx;
		uint8_t *timskx(new volatile uint8_t(*other.timskx_));
		delete timskx_;
		timskx_ = timskx;
		uint8_t *ocrxa(new volatile uint8_t(*other.ocrxa_));
		delete ocrxa_;
		ocrxa_ = ocrxa;
		uint8_t *ocrxb(new volatile uint8_t(*other.ocrxb_));
		delete ocrxb_;
		ocrxb_ = ocrxb;

	}
	return (*this);
}

/*******************************************************************************
 * Method for setting the timer alias and corresponding registers.
 *
 * @param: alias The timer alias.
 ******************************************************************************/
int8_t Timer8::setAlias(const t_alias alias)
{
	int8_t ret = 0;

	alias_ = t_alias::NONE;

	switch(alias)
	{
		case t_alias::T0 :
			setRegistersT0();
			__T8__[0] = this;
			break;

		case t_alias::T2 :
			setRegistersT2();
			__T8__[1] = this;
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

	alias_ = alias;

	return ret;
}

/*******************************************************************************
 * Method for setting the registers as T0 (atmega2560).
 ******************************************************************************/
void Timer8::setRegistersT0(void)
{
	tccrxa_ = (volatile uint8_t *)0x44;
	tccrxb_ = (volatile uint8_t *)0x45;
	tcntx_  = (volatile uint8_t *)0x46;
	timskx_ = (volatile uint8_t *)0x6E;
	ocrxa_  = (volatile uint8_t *)0x47;
	ocrxb_  = (volatile uint8_t *)0x48;
}

/*******************************************************************************
 * Method for setting the registers as T2 (atmega2560).
 ******************************************************************************/
void Timer8::setRegistersT2(void)
{
	tccrxa_ = (volatile uint8_t *)0xB0;
	tccrxb_ = (volatile uint8_t *)0xB1;
	tcntx_  = (volatile uint8_t *)0xB2;
	timskx_ = (volatile uint8_t *)0x70;
	ocrxa_  = (volatile uint8_t *)0xB3;
	ocrxb_  = (volatile uint8_t *)0xB4;
}

/*******************************************************************************
 * Method for setting the timer mode 2 NORMAL or CTC. This method cannot set
 * mode 2 PWM.
 *
 * @param: mode The timer operation mode, only NORMAL or CTC.
 * @param: interrupt The interrupt mode, default is NONE.
 * @param: compare TODO
 ******************************************************************************/
int8_t Timer8::initialize(const t_mode mode, const t_interrupt interrupt)
{
	int8_t ret = 0;

	//t_mode
	if(mode==t_mode::NORMAL or mode==t_mode::CTC)ret = setMode(mode);
	else{ret=-1;}
	if(ret==-1)return ret;

	//t_channel.
	ret = setPwmChannel(t_channel::NONE, t_inverted::NONE);
	if(ret==-1)return ret;

	//t_interrupt.
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
int8_t Timer8::initialize(const t_mode mode, const t_channel channel,
	const t_inverted inverted)
{
	int8_t ret = 0;

	//t_mode
	if(mode==t_mode::PWM_F or mode==t_mode::PWM_PC)ret = setMode(mode);
	else{ret=-1;}
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
int8_t Timer8::setMode(const t_mode mode)
{
	int8_t ret = 0;

	//Non-operational mode.
	setPrescaler(0);
	mode_ = t_mode::NONE;

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
void Timer8::setMode2Normal(void)
{
	//Normal mode.
	*tccrxa_ &= 0x0C;					// WGMx0  = 0;
								// WGMx1  = 0;
								// COMxB0 = 0;
								// COMxB1 = 0;
								// COMxA0 = 0;
								// COMxA1 = 0;
	*tccrxb_ &= ~(1 << 3); 					// WGMx2  = 0;

	//Set var.
	mode_ = t_mode::NORMAL;
}

/*******************************************************************************
 * Private method for setting the timer mode 2 CTC.
 ******************************************************************************/
void Timer8::setMode2Ctc(void)
{
	//CTC Mode.
	*tccrxa_ &= 0x0C;					// WGMx0  = 0;
	*tccrxa_ |=  (1 << 1);					// WGMx1  = 1;
								// COMxB0 = 0;
								// COMxB1 = 0;
								// COMxA0 = 0;
								// COMxA1 = 0;
	*tccrxb_ &= ~(1 << 3); 					// WGMx2  = 0;

	//Set var.
	mode_ = t_mode::CTC;
}

/*******************************************************************************
 * Method for setting the timer interrupt mode.
 *
 * @param: interrupt The timer interrupt mode.
 ******************************************************************************/
int8_t Timer8::setInterruptMode(const t_interrupt interrupt)
{
	int8_t ret = 0;

	//RESET VARS.
	interrupt_ = t_interrupt::NONE;
	*timskx_ = 0x00;
	//__T8__[7] = {}; This is bad...

	//CHECK ALIAS.
	if(alias_!=t_alias::T0 and alias_!=t_alias::T2 and alias_!=t_alias::TX)
	{
		ret = -1;
		return ret;
	}

	//INTERRUPT MODE.
	switch(interrupt)
	{
		case t_interrupt::NONE :
			//Nothing.
			break;

		case t_interrupt::OVF :
			*timskx_ = 0x01;
			if(alias_==t_alias::T0)__T8__[0]=this;
			else{__T8__[1]=this;}
			break;

		case t_interrupt::COMPA :
			*timskx_ = 0x02;
			if(alias_==t_alias::T0)__T8__[2]=this;
			else{__T8__[3]=this;}
			break;

		case t_interrupt::COMPB :
			*timskx_ = 0x04;
			if(alias_==t_alias::T0)__T8__[4]=this;
			else{__T8__[5]=this;}
			break;

		case t_interrupt::COMPC :
		default :
			ret = -1;
			return ret;
	}

	interrupt_ = interrupt;

	return ret;
}

/*******************************************************************************
 * Private method for setting the timer mode 2 fast PWM.
 ******************************************************************************/
void Timer8::setMode2FastPwm(void)
{
	//Fast PWM.
	*tccrxa_ &= 0x0C;					// WGMx0  = 1;
	*tccrxa_ |=  (1 << 0);					// WGMx1  = 1;
	*tccrxa_ |=  (1 << 1);					// COMxB0 = 0;
								// COMxB1 = 0;
								// COMxA0 = 0;
								// COMxA1 = 0;
	//*tccrxb_ |=  (1 << 3); 					// WGMx2  = 1;
	*tccrxb_ &= ~(1 << 3);

	//Set var.
	mode_ = t_mode::PWM_F;
}

/*******************************************************************************
 * Private method for setting the timer mode 2 phase correct PWM.
 ******************************************************************************/
void Timer8::setMode2PhaseCorrectPwm(void)
{
	//Phase Correct PWM.
	*tccrxa_ &= 0x0C;					// WGMx0  = 1;
	*tccrxa_ |=  (1 << 0);					// WGMx1  = 0;
	*tccrxa_ &= ~(1 << 1);					// COMxB0 = 0;
								// COMxB1 = 0;
								// COMxA0 = 0;
								// COMxA1 = 0;
	//*tccrxb_ |=  (1 << 3); 					// WGMx2  = 1;
	*tccrxb_ &= ~(1 << 3);

	//Set var.
	mode_ = t_mode::PWM_PC;
}

/*******************************************************************************
 * Method for setting the timer interrupt mode.
 *
 * @param: interrupt The timer interrupt mode.
 ******************************************************************************/
int8_t Timer8::setPwmChannel(const t_channel channel,
	const t_inverted inverted)
{
	int8_t ret = 0;

	channel_ = t_channel::NONE;
	*tccrxa_ &= 0x0F;
	*tccrxb_ &= 0xFB;

	//PWM channel.
	switch(channel)
	{
		//No PWM channel.
		case t_channel::NONE :
			//Nothing.
			break;

		//Channel A.
		case t_channel::A :
			if(inverted==t_inverted::NORMAL)*tccrxa_ |= 0x80;
			else{*tccrxa_ |= 0xC0;}
			break;

		//Channel B.
		case t_channel::B :
			if(inverted==t_inverted::NORMAL)*tccrxa_ |= 0x20;
			else{*tccrxa_ |= 0x30;}
			break;

		//Channel B w/ specified TOP-value.
		case t_channel::B_TOP :
			*tccrxb_ |= 0x04;
			if(inverted==t_inverted::NORMAL)*tccrxa_ |= 0xA0;
			else{*tccrxa_ |= 0xF0;}
			break;

		//Channel A & B.
		case t_channel::AB :
			if(inverted==t_inverted::NORMAL)*tccrxa_ |= 0xA0;
			else{*tccrxa_ |= 0xF0;}
			break;

		//Invalid options.
		case t_channel::C :
		case t_channel::C_TOP :
		case t_channel::AC :
		case t_channel::BC :
		case t_channel::BC_TOP :
		case t_channel::ABC :
		default :
			ret = -1;
			return ret;
	}

	channel_ = channel;
	inverted_ = inverted;

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
int8_t Timer8::setPrescaler(const uint16_t prescale)
{
	int8_t ret = 0;

	prescale_ = 0;
	*tccrxb_ &= 0xF8;

	if(alias_!=t_alias::T0 and alias_!=t_alias::T2)
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
			*tccrxb_ |= 0x01;
			break;

		case 8 :
			*tccrxb_ |= 0x02;
			break;

		case 32 :
			if(alias_==t_alias::T2)*tccrxb_ |= 0x03;
			else{ret=-1;return ret;}
			break;

		case 64 :
			if(alias_==t_alias::T0)*tccrxb_ |= 0x03;
			else{*tccrxb_ |= 0x04;}
			break;

		case 256 :
			if(alias_==t_alias::T0)*tccrxb_ |= 0x04;
			else{ret=-1;return ret;}
			break;

		case 1024 :
			if(alias_==t_alias::T0)*tccrxb_ |= 0x05;
			else{ret=-1;return ret;}
			break;

		default :
			ret = -1;
			return ret;
	}

	prescale_ = prescale;

	return ret;
}

/*******************************************************************************
 * Method for setting the OCRxA register.
 *
 * @param compare The compare value (unsigned byte).
 ******************************************************************************/
void Timer8::setCompareValueA(const size_t compare)
{
	*ocrxa_ = static_cast<uint8_t>(compare);
}

/*******************************************************************************
 * Method for setting the OCRxB register.
 *
 * @param compare The compare value (unsigned byte).
 ******************************************************************************/
void Timer8::setCompareValueB(const size_t compare)
{
	*ocrxb_ = static_cast<uint8_t>(compare);
}

/*******************************************************************************
 *
 ******************************************************************************/
int8_t Timer8::setDutyCycleA(double dutyCycle)
{
	int8_t ret = 0;
	uint8_t compare = 0;

	if(mode_==t_mode::PWM_F or mode_==t_mode::PWM_PC)
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
int8_t Timer8::setDutyCycleB(double dutyCycle)
{
	int8_t ret = 0;
	uint8_t compare = 0;

	if(mode_==t_mode::PWM_F or mode_==t_mode::PWM_PC)
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
void Timer8::set(const size_t value)
{
	*tcntx_ = static_cast<uint8_t>(value);
}

/*******************************************************************************
 * TODO::clear interrupt bits.
 ******************************************************************************/
void Timer8::reset(void)
{
	set(0x00);
}

/*******************************************************************************
 * TODO::clear interrupt bits.
 ******************************************************************************/
void Timer8::hardReset(void)
{
	//
	if(interrupt_!=t_interrupt::NONE)setInterruptMode(t_interrupt::NONE);
	if(channel_!=t_channel::NONE)setPwmChannel(t_channel::NONE, t_inverted::NONE);
	setMode(t_mode::NORMAL);
	reset();

	//Delete this instance from static list.
	for(uint8_t n = 0; n < 7; ++n)
	{
		if(__T8__[n]==this)delete __T8__[n];
	}
}

/*******************************************************************************
 *
 ******************************************************************************/
size_t Timer8::getCount(void)
{
	return static_cast<size_t>(*tcntx_);
}

/*******************************************************************************
 * Method for obtaining the total summized count since the last reset. Thus
 * overflows are accounted.
 *
 * REMARK: This method only works if the timer interrupts with an timer_ovf_vect()
 *
 * @return: _nonResetCount The count value since last reset.
 ******************************************************************************/
uint32_t Timer8::getNonResetCount(void)
{
	uint8_t count = getCount();
	uint8_t top = 0xFE;
	uint32_t nonResetCount = 0x00000000;

	//TODO::set the top value according to the timer mode.
	//if(channel_==t_channel::B_TOP or channel_==t_channel::C_TOP or channel_==t_channel::BC_TOP)
	nonResetCount = (overflows_*(uint32_t)top) | ((0x0000 << 8) | count);

	return nonResetCount;
}

/** Interrupt functionality overrides *****************************************/
/*void Timer8::interruptServiceRoutine(void)
{
	overflows_++;
}

void Timer16::enable(void)
{
	//TODO::
}

void Timer16::disable(void)
{
	//TODO::
}

void Timer16::clear(void)
{
	//TODO::
}*/

/*******************************************************************************
 * Global forward declaration.
 ******************************************************************************/
Timer8::Ptr Timer8::__T8__[7] = {};

/*******************************************************************************
 * __vector_x -> ISR().
 *
 * TODO::different avrs.
 *
 * Call from self.
 ******************************************************************************/
#define TIMER_ISR(t, vect, n)																					\
ISR(TIMER ## t ## _ ## vect ## _vect)																	\
{																																			\
	if(Timer8::__T8__[n])Timer8::__T8__[n] -> interruptServiceRoutine();\
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

}; //End AVR namespace.
