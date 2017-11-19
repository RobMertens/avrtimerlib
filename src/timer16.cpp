/******************************************************************************
 * avr-timer-lib
 * Timer16.CPP
 *
 * This file contains functions for 16bit avr-timers.
 *
 * TODO::auto pin-map w/ alias function for different arduino's (UNO/MEGA/...).
 *
 * @author:	Rob Mertens
 * @version: 	1.1.1
 * @date: 	20/04/2017
 ******************************************************************************/

#include "timer16.hpp"

/**
 * @brief The project namespace.
 */
namespace avr
{

/** Constructors/destructor/overloading ***************************************/
Timer16::Timer16(void) : Timer()
{
	alias_ = t_alias::NONE;

	overflows_ = 0;
}

Timer16::Timer16(const t_alias alias) : Timer()
{
	setAlias(alias);

	overflows_ = 0;
}

Timer16::Timer16(const volatile uint8_t * const& tccrxa,
	const volatile uint8_t * const& tccrxb,
	const volatile uint8_t * const& tcntxh,
	const volatile uint8_t * const& tcntxl,
	const volatile uint8_t * const& timskx,
	const volatile uint8_t * const& ocrxah,
	const volatile uint8_t * const& ocrxbh,
	const volatile uint8_t * const& ocrxal,
	const volatile uint8_t * const& ocrxbl) : Timer()
{
	tccrxa_ = tccrxa;						// Timer Count Control Register.
	tccrxb_ = tccrxb;						// Timer Count Control Register.
	tcntxl_ = tcntxl;						// Timer Count register.
	tcntxh_ = tcntxh;
	timskx_ = timskx;						// Timer Interrupt Mask register.
	ocrxal_ = ocrxal;
	ocrxah_ = ocrxah;
	ocrxbl_ = ocrxbl;
	ocrxbh_ = ocrxbh;

	alias_ = t_alias::TX;
	__T16__[16] = this;						// Instance of itself for ISR.

	overflows_ = 0;
}

Timer16::~Timer16(void)
{
	//Free the register pointers.
	delete tccrxa_;
	delete tccrxb_;
	delete tcntxl_;
	delete tcntxh_;
	delete timskx_;
	delete ocrxal_;
	delete ocrxah_;
	delete ocrxbl_;
	delete ocrxbh_;
	delete ocrxcl_;
	delete ocrxch_;

	//Delete this instance from static list.
	for(uint8_t n = 0; n < 17; ++n)
	{
		//Do both pointers point to the same adress?
		//If so, delete pointer in the static list.
		if(__T16__[n]==this)delete __T16__[n];
	}
}

Timer16& Timer16::operator=(const Timer16& other)
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
		uint8_t *tcntxl(new volatile uint8_t(*other.tcntxl_));
		delete tcntxl_;
		tcntxl_ = tcntxl;
		uint8_t *tcntxh(new volatile uint8_t(*other.tcntxh_));
		delete tcntxh_;
		tcntxh_ = tcntxh;
		uint8_t *timskx(new volatile uint8_t(*other.timskx_));
		delete timskx_;
		timskx_ = timskx;
		uint8_t *ocrxal(new volatile uint8_t(*other.ocrxal_));
		delete ocrxal_;
		ocrxal_ = ocrxal;
		uint8_t *ocrxah(new volatile uint8_t(*other.ocrxah_));
		delete ocrxah_;
		ocrxah_ = ocrxah;
		uint8_t *ocrxbl(new volatile uint8_t(*other.ocrxbl_));
		delete ocrxbl_;
		ocrxbl_ = ocrxbl;
		uint8_t *ocrxbh(new volatile uint8_t(*other.ocrxbh_));
		delete ocrxbh_;
		ocrxbh_ = ocrxbh;
		uint8_t *ocrxcl(new volatile uint8_t(*other.ocrxcl_));
		delete ocrxcl_;
		ocrxcl_ = ocrxcl;
		uint8_t *ocrxch(new volatile uint8_t(*other.ocrxch_));
		delete ocrxch_;
		ocrxch_ = ocrxch;

	}
	return (*this);
}

/** Timer setting functions ***************************************************/
int8_t Timer16::setAlias(const t_alias alias)
{
	int8_t ret = 0;
	alias_ = t_alias::NONE;

	switch(alias)
	{
		case t_alias::T1 :
			setRegistersT1();
			break;

		case t_alias::T3 :
			setRegistersT3();
			break;

		case t_alias::T4 :
			setRegistersT4();
			break;

		case t_alias::T5 :
			setRegistersT5();
			break;

		case t_alias::NONE :
		case t_alias::T0 :
		case t_alias::T2 :
		case t_alias::TX :
		default :
			ret = -1;
			return ret;
	}

	alias_ = alias;

	return ret;
}

void Timer16::setRegistersT1(void)
{
	tccrxa_ = (volatile uint8_t *)0x80;
	tccrxb_ = (volatile uint8_t *)0x81;
	tcntxl_ = (volatile uint8_t *)0x84;
	tcntxh_ = (volatile uint8_t *)0x85;
	timskx_ = (volatile uint8_t *)0x6F;
	ocrxal_ = (volatile uint8_t *)0x88;
	ocrxah_ = (volatile uint8_t *)0x89;
	ocrxbl_ = (volatile uint8_t *)0x8A;
	ocrxbh_ = (volatile uint8_t *)0x8B;
	ocrxcl_ = (volatile uint8_t *)0x8C;
	ocrxch_ = (volatile uint8_t *)0x8D;
}

void Timer16::setRegistersT3(void)
{
	tccrxa_ = (volatile uint8_t *)0x90;
	tccrxb_ = (volatile uint8_t *)0x91;
	tcntxl_ = (volatile uint8_t *)0x94;
	tcntxh_ = (volatile uint8_t *)0x95;
	timskx_ = (volatile uint8_t *)0x71;
	ocrxal_ = (volatile uint8_t *)0x98;
	ocrxah_ = (volatile uint8_t *)0x99;
	ocrxbl_ = (volatile uint8_t *)0x9A;
	ocrxbh_ = (volatile uint8_t *)0x9B;
	ocrxcl_ = (volatile uint8_t *)0x9C;
	ocrxch_ = (volatile uint8_t *)0x9D;
}

void Timer16::setRegistersT4(void)
{
	tccrxa_ = (volatile uint8_t *)0xA0;
	tccrxb_ = (volatile uint8_t *)0xA1;
	tcntxl_ = (volatile uint8_t *)0xA4;
	tcntxh_ = (volatile uint8_t *)0xA5;
	timskx_ = (volatile uint8_t *)0x72;
	ocrxal_ = (volatile uint8_t *)0xA8;
	ocrxah_ = (volatile uint8_t *)0xA9;
	ocrxbl_ = (volatile uint8_t *)0xAA;
	ocrxbh_ = (volatile uint8_t *)0xAB;
	ocrxcl_ = (volatile uint8_t *)0xAC;
	ocrxch_ = (volatile uint8_t *)0xAD;
}

void Timer16::setRegistersT5(void)
{
	tccrxa_ = (volatile uint8_t *)0x120;
	tccrxb_ = (volatile uint8_t *)0x121;
	tcntxl_ = (volatile uint8_t *)0x124;
	tcntxh_ = (volatile uint8_t *)0x125;
	timskx_ = (volatile uint8_t *)0x73;
	ocrxal_ = (volatile uint8_t *)0x128;
	ocrxah_ = (volatile uint8_t *)0x129;
	ocrxbl_ = (volatile uint8_t *)0x12A;
	ocrxbh_ = (volatile uint8_t *)0x12B;
	ocrxcl_ = (volatile uint8_t *)0x12C;
	ocrxch_ = (volatile uint8_t *)0x12D;
}

int8_t Timer16::initialize(const t_mode mode, const t_interrupt interrupt)
{
	int8_t ret = 0;

	//t_mode
	if(mode==t_mode::NORMAL or mode==t_mode::CTC)ret = setMode(mode);
	if(ret==-1)return ret;

	//t_channel.
	ret = setPwmChannel(t_channel::NONE, t_inverted::NONE);
	if(ret==-1)return ret;

	//t_interrupt.
	ret = setInterruptMode(interrupt);
	if(ret==-1)return ret;

	return ret;
}

int8_t Timer16::initialize(const t_mode mode, const t_channel channel,
	const t_inverted inverted)
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

int8_t Timer16::setMode(const t_mode mode)
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

		case t_mode::PWM_FC :
			setMode2FrequencyCorrectPwm();
			break;

		case t_mode::NONE :
			//Nothing.
			break;

		default :
			ret = -1;
			return ret;
	}

	return ret;
}

void Timer16::setMode2Normal(void)
{
	//Normal mode.
	*tccrxa_ &= 0x0C;					// WGMx0  = 0;
								// WGMx1  = 0;
								// COMxB0 = 0;
								// COMxB1 = 0;
								// COMxA0 = 0;
								// COMxA1 = 0;
	*tccrxb_ &= 0xF7; 					// WGMx2  = 0;

	//Set var.
	mode_ = t_mode::NORMAL;
}

void Timer16::setMode2Ctc(void)
{
	//CTC Mode.
	*tccrxa_ &= 0x0C;					// WGMx0  = 0;
	*tccrxa_ |=  (1 << 1);					// WGMx1  = 1;
								// COMxB0 = 0;
								// COMxB1 = 0;
								// COMxA0 = 0;
								// COMxA1 = 0;
	*tccrxb_ &= 0xF7; 					// WGMx2  = 0;

	//Set var.
	mode_ = t_mode::CTC;
}

int8_t Timer16::setInterruptMode(const t_interrupt interrupt)
{
	int8_t ret = 0;

	//RESET VARS.
	interrupt_ = t_interrupt::NONE;
	*timskx_ = 0x00;
	*__T16__[17] = {};

	//CHECK ALIAS.
	if(alias_!=t_alias::T1 and alias_!=t_alias::T3 and alias_!=t_alias::T4
		and alias_!=t_alias::T5 and alias_!=t_alias::TX)
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
			if(alias_==t_alias::T1)__T16__[0]=this;
			else if(alias_==t_alias::T3)__T16__[1]=this;
			else if(alias_==t_alias::T4)__T16__[2]=this;
			else{__T16__[3]=this;}
			break;

		case t_interrupt::COMPA :
			*timskx_ = 0x02;
			if(alias_==t_alias::T1)__T16__[4]=this;
			else if(alias_==t_alias::T3)__T16__[5]=this;
			else if(alias_==t_alias::T4)__T16__[6]=this;
			else{__T16__[7]=this;}
			break;

		case t_interrupt::COMPB :
			*timskx_ = 0x04;
			if(alias_==t_alias::T1)__T16__[8]=this;
			else if(alias_==t_alias::T3)__T16__[9]=this;
			else if(alias_==t_alias::T4)__T16__[10]=this;
			else{__T16__[11]=this;}
			break;

		case t_interrupt::COMPC :
			*timskx_ = 0x08;
			if(alias_==t_alias::T1)__T16__[12]=this;
			else if(alias_==t_alias::T3)__T16__[13]=this;
			else if(alias_==t_alias::T4)__T16__[14]=this;
			else{__T16__[15]=this;}
			break;

		default :
			ret = -1;
			return ret;
	}

	interrupt_ = interrupt;

	return ret;
}

void Timer16::setMode2FastPwm(void)
{
	//Fast PWM.
	*tccrxa_ &= 0x0C;					// WGMx0  = 1;
	*tccrxa_ |=  (1 << 0);					// WGMx1  = 1;
	*tccrxa_ |=  (1 << 1);					// COMxB0 = 0;
								// COMxB1 = 0;
								// COMxA0 = 0;
	*tccrxb_ &= 0xF7;					// COMxA1 = 0; TODO::check 0xF7
	//*tccrxb_ |= (1 << 3);		 				// WGMx2  = 1;

	//Set var.
	mode_ = t_mode::PWM_F;
}

void Timer16::setMode2PhaseCorrectPwm(void)
{
	//Phase Correct PWM.
	*tccrxa_ &= 0x0C;					// WGMx0  = 1;
	*tccrxa_ |=  (1 << 0);		// WGMx1  = 0;
														// COMxB0 = 0;
														// COMxB1 = 0;
														// COMxA0 = 0;
	*tccrxb_ &= 0xF7;					// COMxA1 = 0; TODO::check 0xF7
	//*tccrxb_ |= (1 << 3);			 // WGMx2  = 1;

	//Set var.
	mode_ = t_mode::PWM_PC;
}

void Timer16::setMode2FrequencyCorrectPwm(void)
{
	//Phase Correct PWM.
	*tccrxa_ &= 0x0C;					// WGMx0  = 1;
	//*tccrxa_ |= (1 << 0);			// WGMx1  = 0;
														// COMxB0 = 0;
														// COMxB1 = 0;
														// COMxA0 = 0;
	*tccrxb_ &= 0xF7;					// COMxA1 = 0; TODO::check 0xF7
	//*tccrxb_ |= (1 << 3); 		// WGMx2  = 1;

	//Set var.
	mode_ = t_mode::PWM_FC;
}

int8_t Timer16::setPwmChannel(const t_channel channel,
	const t_inverted inverted)
{
	int8_t ret = 0;

	//RESET VARS.
	channel_ = t_channel::NONE;
	inverted_ = t_inverted::NONE;
	*tccrxa_ &= 0x0F;
	*tccrxb_ &= 0xFB;

	//PWM channel.
	switch(channel)
	{
		case t_channel::NONE :
			//Nothing.
			break;

		case t_channel::A :
			if(inverted==t_inverted::NORMAL)*tccrxa_ |= 0x80;
			else{*tccrxa_ |= 0xC0;}
			break;

		case t_channel::B :
			if(inverted==t_inverted::NORMAL)*tccrxa_ |= 0x20;
			else{*tccrxa_ |= 0x30;}
			break;

		case t_channel::B_TOP : //TODO::check tccrxa
			*tccrxb_ |= 0x04;
			if(inverted==t_inverted::NORMAL)*tccrxa_ |= 0xA0;
			else{*tccrxa_ |= 0xF0;}
			break;

		case t_channel::C :
			if(inverted==t_inverted::NORMAL)*tccrxa_ |= 0x08;
			else{*tccrxa_ |= 0x0C;}
			break;

		case t_channel::C_TOP : //TODO::check tccrxa
			*tccrxb_ |= 0x04;
			if(inverted==t_inverted::NORMAL)*tccrxa_ |= 0xA0;
			else{*tccrxa_ |= 0xF0;}
			break;

		case t_channel::AB :
			if(inverted==t_inverted::NORMAL)*tccrxa_ |= 0xA0;
			else{*tccrxa_ |= 0xF0;}
			break;

		case t_channel::AC :
			if(inverted==t_inverted::NORMAL)*tccrxa_ |= 0x88;
			else{*tccrxa_ |= 0xCC;}
			break;

		case t_channel::BC :
			if(inverted==t_inverted::NORMAL)*tccrxa_ |= 0x28;
			else{*tccrxa_ |= 0x3C;}
			break;

		case t_channel::BC_TOP : //TODO::check tccrxa
			*tccrxb_ |= 0x04;
			if(inverted==t_inverted::NORMAL)*tccrxa_ |= 0x28;
			else{*tccrxa_ |= 0x3C;}
			break;

		case t_channel::ABC :
			if(inverted==t_inverted::NORMAL)*tccrxa_ |= 0xA8;
			else{*tccrxa_ |= 0xFC;}
			break;

		default :
			ret = -1;
			return ret;
	}

	channel_ = channel;
	inverted_ = inverted;

	return ret;
}

int8_t Timer16::setPrescaler(const uint16_t prescale)
{
	int8_t ret = 0;

	prescale_ = 0;
	*tccrxb_ &= 0xF8;

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

		case 64 :
			*tccrxb_ |= 0x03;
			break;

		case 256 :
			*tccrxb_ |= 0x04;
			break;

		case 1024 :
			*tccrxb_ |= 0x05;
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
void Timer16::setCompareValueA(const size_t compare)
{
	*ocrxah_ = (uint8_t)((static_cast<uint16_t>(compare) >> 8) & 0xFF);
	*ocrxal_ = (uint8_t)(static_cast<uint16_t>(compare) & 0xFF);
}

/*******************************************************************************
 * Method for setting the OCRxB register.
 *
 * @param compare The compare value (unsigned byte).
 ******************************************************************************/
void Timer16::setCompareValueB(const size_t compare)
{
	*ocrxbh_ = (uint8_t)((static_cast<uint16_t>(compare) >> 8) & 0xFF);
	*ocrxbl_ = (uint8_t)(static_cast<uint16_t>(compare) & 0xFF);
}

/*******************************************************************************
 * Method for setting the OCRxB register.
 *
 * @param compare The compare value (unsigned byte).
 ******************************************************************************/
void Timer16::setCompareValueC(const size_t compare)
{
	*ocrxch_ = (uint8_t)((static_cast<uint16_t>(compare) >> 8) & 0xFF);
	*ocrxcl_ = (uint8_t)(static_cast<uint16_t>(compare) & 0xFF);
}

/*******************************************************************************
 *
 ******************************************************************************/
int8_t Timer16::setDutyCycleA(double dutyCycle)
{
	int8_t ret = 0;
	uint16_t compare = 0;

	if(mode_==t_mode::PWM_F or mode_==t_mode::PWM_PC or mode_==t_mode::PWM_FC)
	{
		if(dutyCycle > 1.0)dutyCycle=1.0;
		else if(dutyCycle < 0.0)dutyCycle=0.0;

		compare = uint8_t(dutyCycle*0xFFFF-1);

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
int8_t Timer16::setDutyCycleB(double dutyCycle)
{
	int8_t ret = 0;
	uint16_t compare = 0;

	if(mode_==t_mode::PWM_F or mode_==t_mode::PWM_PC or mode_==t_mode::PWM_PC)
	{
		if(dutyCycle > 1.0)dutyCycle=1.0;
		else if(dutyCycle < 0.0)dutyCycle=0.0;

		if(*ocrxal_==0x00)compare = uint8_t(dutyCycle*0xFFFF-1);
		else{compare = uint8_t(dutyCycle*((*ocrxah_ << 8) | (*ocrxal_ & 0xFF))-1);}

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
int8_t Timer16::setDutyCycleC(double dutyCycle)
{
	int8_t ret = 0;
	uint16_t compare = 0;

	if(mode_==t_mode::PWM_F or mode_==t_mode::PWM_PC or mode_==t_mode::PWM_PC)
	{
		if(dutyCycle > 1.0)dutyCycle=1.0;
		else if(dutyCycle < 0.0)dutyCycle=0.0;

		if(*ocrxal_==0x00)compare = uint8_t(dutyCycle*0xFFFF-1);
		else{compare = uint8_t(dutyCycle*(((*ocrxah_ & 0xFF )<< 8) | (*ocrxal_ & 0xFF))-1);}

		setCompareValueC(compare);
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
void Timer16::set(const size_t value)
{
	*tcntxh_ = (uint8_t)((static_cast<uint16_t>(value) >> 8) & 0xFF);
	*tcntxl_ = (uint8_t)(static_cast<uint16_t>(value) & 0xFF);
}

/*******************************************************************************
 * TODO::clear interrupt bits.
 ******************************************************************************/
void Timer16::reset(void)
{
	set(0x0000);
}

/*******************************************************************************
 * TODO::clear interrupt bits.
 ******************************************************************************/
void Timer16::hardReset(void)
{
	if(interrupt_!=t_interrupt::NONE)setInterruptMode(t_interrupt::NONE);
	if(channel_!=t_channel::NONE)setPwmChannel(t_channel::NONE, t_inverted::NONE);
	setMode(t_mode::NORMAL);
	reset();

	//Delete this instance from the list.
	for(uint8_t n = 0; n < 17; ++n)
	{
		if(__T16__[n]==this)delete __T16__[n];
	}
}

/*******************************************************************************
 *
 ******************************************************************************/
size_t Timer16::getCount(void)
{
	uint16_t count;
	count = (((*tcntxh_ & 0xFF) << 8) | (*tcntxl_ & 0xFF));
	return static_cast<size_t>(count);
}

/*******************************************************************************
 * Method for obtaining the total summized count since the last reset. Thus
 * overflows are accounted.
 *
 * REMARK: This method only works if the timer interrupts with an timer_ovf_vect()
 *
 * @return: _nonResetCount The count value since last reset.
 ******************************************************************************/
uint32_t Timer16::getNonResetCount(void)
{
	uint8_t count = getCount();
	uint16_t top = 0xFFFE;
	uint32_t nonResetCount = 0x00000000;

	//TODO::set the top value according to the timer mode.
	//if(channel_==t_channel::B_TOP or channel_==t_channel::C_TOP or channel_==t_channel::BC_TOP)
	nonResetCount = (overflows_*(uint32_t)top) | ((0x0000 << 8) | count);

	return nonResetCount;
}

/** Interrupt functionality overrides *****************************************/
/*void Timer16::enable(void)
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
}

void Timer16::interruptServiceRoutine(void)
{
	overflows_++;
}*/

/*******************************************************************************
 * Global forward declaration.
 ******************************************************************************/
Timer16 * Timer16::__T16__[17] = {};

/*******************************************************************************
 * _vector__x -> ISR().
 *
 * TODO::different avrs.
 *
 * Call from self.
 ******************************************************************************/
#define TIMER_ISR(t, vect, n)																					\
ISR(TIMER ## t ## _ ## vect ## _vect)																	\
{																																			\
	if(Timer16::__T16__[n])Timer16::__T16__[n] -> interruptServiceRoutine();	\
}

#if defined(TIMER1_OVF_vect)
TIMER_ISR(1, OVF, 0)
#endif
#if defined(TIMER3_OVF_vect)
TIMER_ISR(3, OVF, 1)
#endif
#if defined(TIMER4_OVF_vect)
TIMER_ISR(4, OVF, 2)
#endif
#if defined(TIMER5_OVF_vect)
TIMER_ISR(5, OVF, 3)
#endif

#if defined(TIMER1_COMPA_vect)
TIMER_ISR(1, COMPA, 4)
#endif
#if defined(TIMER3_COMPA_vect)
TIMER_ISR(3, COMPA, 5)
#endif
#if defined(TIMER4_COMPA_vect)
TIMER_ISR(4, COMPA, 6)
#endif
#if defined(TIMER5_COMPA_vect)
TIMER_ISR(5, COMPA, 7)
#endif

#if defined(TIMER1_COMPB_vect)
TIMER_ISR(1, COMPB, 8)
#endif
#if defined(TIMER3_COMPB_vect)
TIMER_ISR(3, COMPB, 9)
#endif
#if defined(TIMER4_COMPB_vect)
TIMER_ISR(4, COMPB, 10)
#endif
#if defined(TIMER5_COMPB_vect)
TIMER_ISR(5, COMPB, 11)
#endif

#if defined(TIMER1_COMPC_vect)
TIMER_ISR(1, COMPC, 12)
#endif
#if defined(TIMER3_COMPC_vect)
TIMER_ISR(3, COMPC, 13)
#endif
#if defined(TIMER4_COMPC_vect)
TIMER_ISR(4, COMPC, 14)
#endif
#if defined(TIMER5_COMPC_vect)
TIMER_ISR(5, COMPC, 15)
#endif

}; //End AVR namespace.
