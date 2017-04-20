/******************************************************************************
 * avr-timer-lib
 * TIMER16.CPP
 *  
 * This file contains functions for 16bit avr-timers.
 *
 * TODO::auto pin-map w/ alias function for different arduino's (UNO/MEGA/...).
 *
 * @author:	Rob Mertens
 * @version: 	1.0.1 
 * @date: 	20/04/2017
 ******************************************************************************/

#include <timer16.h>

/*******************************************************************************
 * Global forward declaration.
 * 
 * TODO::this static instance is used for all timer16 instances, does this work?
 *	 for each timer instance a new static instance and reference to ISR.
 ******************************************************************************/
static timer16 *_t;

/*******************************************************************************
 * Constructor for a 8-bit timer, e.g.: timer0 on MEGA2560.
 * 
 * TODO::auto pin-map w/ alias function.
 *
 * @param: tccr The timer control register.
 * @param: tcnt The timer count register.
 ******************************************************************************/
timer16::timer16(uint8_t alias, volatile uint8_t * tccrxa, volatile uint8_t * tccrxb, volatile uint8_t * tcntx, volatile uint8_t * timskx, volatile uint16_t * ocrxa, volatile uint16_t * ocrxb)
{
	_alias  = alias;						// TIMER0 or TIMER2.
	_tccrxa = tccrxa;						// Timer Count Control Register.
	_tccrxb = tccrxb;						// Timer Count Control Register.
	_tcntx  = tcntx;						// Timer Count register.
	_timskx = timskx;						// Timer Interrupt Mask register.
	_ocrxa  = ocrxa;
	_ocrxb  = ocrxb;
	
	_interruptCount = 0;
	_overflowCount = 0;
	_nonResetCount = 0;
	
	_t = this;							// Instance of itself for ISR.
}

/*******************************************************************************
 * Method for setting the timer alias.
 *
 * TODO::auto pin-map.
 *
 * @param: timer The timer alias.
 ******************************************************************************/
void timer16::setAlias(uint8_t alias)
{
	_alias	= alias;
}

/*******************************************************************************
 * Method for setting the timer mode 2 NORMAL or CTC. This method cannot set
 * mode 2 PWM.
 *
 * @param: mode The timer operation mode, only NORMAL or CTC.
 * @param: interrupt The interrupt mode, default is NONE.
 * @param: compare TODO
 ******************************************************************************/
int8_t timer16::initialize(t_mode mode, t_interrupt interrupt, uint16_t compare)
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
int8_t timer16::initialize(t_mode mode, t_channel channel, bool inverted)
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
int8_t timer16::setMode(t_mode mode)
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
		
		case t_mode::PWM_PC :
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

/*******************************************************************************
 * Private method for setting the timer mode 2 NORMAL.
 ******************************************************************************/
void timer16::setMode2Normal()
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
void timer16::setMode2Ctc()
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
int8_t timer16::setInterruptMode(t_interrupt interrupt)
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
			break;
	
		case t_interrupt::COMPA :
			*_timskx = 0x02;
			break;
	
		case t_interrupt::COMPB :
			*_timskx = 0x04;
			break;
		
		case t_interrupt::COMPC :
			//TODO::
			break;
		
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
void timer16::setMode2FastPwm()
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
void timer16::setMode2PhaseCorrectPwm()
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
 * Private method for setting the timer mode 2 phase correct PWM.
 ******************************************************************************/
void timer16::setMode2PhaseCorrectPwm()
{
	//Frequency Correct PWM.
	//TODO::
}

/*******************************************************************************
 * Method for setting the timer interrupt mode.
 *
 * @param: interrupt The timer interrupt mode.
 ******************************************************************************/
int8_t timer16::setPwmChannel(t_channel channel, bool inverted)
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
		
		case t_channel::C :
			//TODO::
			break;
		
		case t_channel::AB :
			if(!inverted)*_tccrxa |= 0xA0;
			else{*_tccrxa |= 0xF0;}
			break;
		
		case t_channel::BC :
			//TODO::
			break;
		
		case t_channel::AC :
			//TODO::
			break;
		
		case t_channel::ABC :
			//TODO::
			break;
		
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
int8_t timer16::setPrescaler(uint16_t prescale)
{
	int8_t ret = 0;
	
	_prescale = 0;
	*_tccrxb &= 0xF8;
	
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
		
		case 64 :
			*_tccrxb |= 0x03;
			break;
		
		case 256 :
			*_tccrxb |= 0x04;
			break;
		
		case 1024 :
			*_tccrxb |= 0x05;
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
void timer16::setCompareValueA(uint16_t compare)
{
	*_ocrxa = compare;
}

/*******************************************************************************
 * Method for setting the OCRxB register.
 *
 * @param compare The compare value (unsigned byte).
 ******************************************************************************/
void timer16::setCompareValueB(uint16_t compare)
{
	*_ocrxb = compare;
}

/*******************************************************************************
 * 
 ******************************************************************************/
int8_t timer16::setDutyCycleA(double dutyCycle)
{
	int8_t ret = 0;
	uint16_t compare = 0;
	
	if(_mode==t_mode::PWM_F or _mode==t_mode::PWM_PC)
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
int8_t timer16::setDutyCycleB(double dutyCycle)
{
	int8_t ret = 0;
	uint16_t compare = 0;
	
	if(_mode==t_mode::PWM_F or _mode==t_mode::PWM_PC)
	{
		if(dutyCycle > 1.0)dutyCycle=1.0;
		else if(dutyCycle < 0.0)dutyCycle=0.0;
		
		compare = uint8_t(dutyCycle*0xFFFF-1);
		
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
int8_t timer16::setDutyCycleAB(double dutyCycleA, double dutyCycleB)
{
	//TODO::
	
	return -1;
}

/*******************************************************************************
 * 
 ******************************************************************************/
void timer16::set(uint8_t value)
{
	*_tcntx = value;
}

/*******************************************************************************
 * TODO::clear interrupt bits.
 ******************************************************************************/
void timer16::reset()
{
	set(0x0000);
}

/*******************************************************************************
 * TODO::clear interrupt bits.
 ******************************************************************************/
void timer16::hardReset()
{
	setMode(t_mode::NORMAL);
	setInterruptMode(t_interrupt::NONE);
	setPrescaler(0);
	reset();
}

/*******************************************************************************
 * 
 ******************************************************************************/
uint8_t timer16::getCount()
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
uint16_t timer16::getNonResetCount()
{
	//TODO::calculations.
	
	return _nonResetCount;
}

/*******************************************************************************
 * 
 ******************************************************************************/
uint16_t timer16::getOverflowCount()
{
	return _overflowCount;
}

/*******************************************************************************
 * 
 ******************************************************************************/
t_interrupt timer16::getInterruptMode()
{
	return _interrupt;
}

/*******************************************************************************
 * ISR for the timer class.
 ******************************************************************************/
void timer16::interruptServiceRoutine(void)
{
	//TODO::check which interrupt is active.
	_overflowCount++;						// timer_ovf_vect()
	//_interruptCount++;						// timer_comp_vect()
}

/*******************************************************************************
 * Enable timer interrupts.
 ******************************************************************************/
void timer16::enable(void)
{
	//TODO::
}

/*******************************************************************************
 * Disable timer interrupts.
 ******************************************************************************/
void timer16::disable(void)
{
	//TODO::
}

/*******************************************************************************
 * Clear timer interrupts.
 ******************************************************************************/
void timer16::clear(void)
{
	//TODO::
}

/*******************************************************************************
 * ISR -> myISR().
 * 
 * TODO::different vectors and chips.
 * 
 * Call from self.
 ******************************************************************************/
ISR(TIMER1_OVF_vect)
{
	if(_t)_t -> interruptServiceRoutine();
}

 
