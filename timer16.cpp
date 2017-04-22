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
 * Constructor for a 16-bit timer, e.g.: timer0 on MEGA2560.
 * 
 * TODO::auto pin-map w/ alias function.
 *
 * @param: tccr The timer control register.
 * @param: tcnt The timer count register.
 ******************************************************************************/
timer16::timer16(t_alias alias)
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
timer16::timer16()
{
	_alias = t_alias::NONE;
	
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
timer16::timer16(volatile uint8_t * tccrxa, volatile uint8_t * tccrxb, volatile uint8_t * tcntxh, volatile uint8_t * tcntxl, volatile uint8_t * timskx, volatile uint8_t * ocrxah, volatile uint8_t * ocrxal, volatile uint8_t * ocrxbh, volatile uint8_t * ocrxbl)
{
	_tccrxa = tccrxa;						// Timer Count Control Register.
	_tccrxb = tccrxb;						// Timer Count Control Register.
	_tcntxl = tcntxl;						// Timer Count register.
	_tcntxh = tcntxh;
	_timskx = timskx;						// Timer Interrupt Mask register.
	_ocrxal = ocrxal;
	_ocrxah = ocrxah;
	_ocrxbl = ocrxbl;
	_ocrxbh = ocrxbh;
	
	_alias = t_alias::TX;
	_t16[6] = this;							// Instance of itself for ISR.
	
	_overflowCount  = 0;
	_compareCount   = 0;
	_nonResetCount  = 0;
}

/*******************************************************************************
 * Method for setting the timer alias and corresponding registers.
 *
 * @param: alias The timer alias.
 ******************************************************************************/
int8_t timer16::setAlias(t_alias alias)
{
	int8_t ret = 0;
	_alias = t_alias::NONE;
	
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
	
	_alias = alias;
	
	_t16[(uint8_t)(_alias)] = this;
	
	return ret;
}

/*******************************************************************************
 * Method for setting the registers as T1 (atmega2560).
 ******************************************************************************/
void timer16::setRegistersT1()
{
	_tccrxa = (volatile uint8_t *)0x80;
	_tccrxb = (volatile uint8_t *)0x81;
	_tcntxl = (volatile uint8_t *)0x84;
	_tcntxh = (volatile uint8_t *)0x85;
	_timskx = (volatile uint8_t *)0x6F;
	_ocrxal = (volatile uint8_t *)0x88;
	_ocrxah = (volatile uint8_t *)0x89;
	_ocrxbl = (volatile uint8_t *)0x8A;
	_ocrxbh = (volatile uint8_t *)0x8B;
	_ocrxcl = (volatile uint8_t *)0;
	_ocrxch = (volatile uint8_t *)0;
}

/*******************************************************************************
 * Method for setting the registers as T3 (atmega2560).
 ******************************************************************************/
void timer16::setRegistersT3()
{
	_tccrxa = (volatile uint8_t *)0x90;
	_tccrxb = (volatile uint8_t *)0x91;
	_tcntxl = (volatile uint8_t *)0x94;
	_tcntxh = (volatile uint8_t *)0x95;
	_timskx = (volatile uint8_t *)0x71;
	_ocrxal = (volatile uint8_t *)0x98;
	_ocrxah = (volatile uint8_t *)0x99;
	_ocrxbl = (volatile uint8_t *)0x9A;
	_ocrxbh = (volatile uint8_t *)0x9B;
	_ocrxcl = (volatile uint8_t *)0;
	_ocrxch = (volatile uint8_t *)0;
}

/*******************************************************************************
 * Method for setting the registers as T4 (atmega2560).
 ******************************************************************************/
void timer16::setRegistersT4()
{
	_tccrxa = (volatile uint8_t *)0xA0;
	_tccrxb = (volatile uint8_t *)0xA1;
	_tcntxl = (volatile uint8_t *)0xA4;
	_tcntxh = (volatile uint8_t *)0xA5;
	_timskx = (volatile uint8_t *)0x72;
	_ocrxal = (volatile uint8_t *)0xA8;
	_ocrxah = (volatile uint8_t *)0xA9;
	_ocrxbl = (volatile uint8_t *)0xAA;
	_ocrxbh = (volatile uint8_t *)0xAB;
	_ocrxcl = (volatile uint8_t *)0;
	_ocrxch = (volatile uint8_t *)0;
}

/*******************************************************************************
 * Method for setting the registers as T5 (atmega2560).
 ******************************************************************************/
void timer16::setRegistersT5()
{
	_tccrxa = (volatile uint8_t *)0x120;
	_tccrxb = (volatile uint8_t *)0x121;
	_tcntxl = (volatile uint8_t *)0x124;
	_tcntxh = (volatile uint8_t *)0x125;
	_timskx = (volatile uint8_t *)0x73;
	_ocrxal = (volatile uint8_t *)0x128;
	_ocrxah = (volatile uint8_t *)0x129;
	_ocrxbl = (volatile uint8_t *)0x12A;
	_ocrxbh = (volatile uint8_t *)0x12B;
	_ocrxcl = (volatile uint8_t *)0;
	_ocrxch = (volatile uint8_t *)0;
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
	ret = setInterruptMode(interrupt, compare);
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
int8_t timer16::setInterruptMode(t_interrupt interrupt, uint16_t compare)
{
	int8_t ret = 0;
	
	_interrupt = t_interrupt::NONE;
	*_timskx = 0x00;
	
	//Reset compare registers.
	setCompareValueA(0x0000);
	setCompareValueB(0x0000);
	setCompareValueC(0x0000);
	
	//INTERRUPT MODE.
	switch(interrupt)
	{
		case t_interrupt::NONE : 
			//Nothing.
			break;
	
		case t_interrupt::OVF : 
			*_timskx = 0x01;
			
			if(_alias==t_alias::T1)_t16[0] = this;
			else if(_alias==t_alias::T3)_t16[1] = this;
			else if(_alias==t_alias::T4)_t16[2] = this;
			else{_t16[3] = this;}
			break;
	
		case t_interrupt::COMPA :
			*_timskx = 0x02;
			
			if(_alias==t_alias::T1)_t16[4] = this;
			else if(_alias==t_alias::T3)_t16[5] = this;
			else if(_alias==t_alias::T4)_t16[6] = this;
			else{_t16[7] = this;}
			
			setCompareValueA(compare);
			break;
	
		case t_interrupt::COMPB :
			*_timskx = 0x04;
			
			if(_alias==t_alias::T1)_t16[8] = this;
			else if(_alias==t_alias::T3)_t16[9] = this;
			else if(_alias==t_alias::T4)_t16[10] = this;
			else{_t16[11] = this;}
			
			setCompareValueB(compare);
			break;
		
		case t_interrupt::COMPC :
			//TODO::
			
			
			if(_alias==t_alias::T1)_t16[12] = this;
			else if(_alias==t_alias::T3)_t16[13] = this;
			else if(_alias==t_alias::T4)_t16[14] = this;
			else{_t16[15] = this;}
			
			setCompareValueC(compare);
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
void timer16::setMode2FrequencyCorrectPwm()
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
 *    VALUE | TIMER 1 | TIMER 3 | TIMER 4 | TIMER 5
 *     	  1 |    x    |    x	|    x    |    x
 * 	  8 |    x    |    x	|    x    |    x
 *       64 |    x    |    x	|    x    |    x
 *      256 |    x    |    x    |    x    |    x   
 *     1024 |    x    |    x    |    x    |    x
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
	*_ocrxah = (uint8_t)((compare >> 8) & 0xFF);
	*_ocrxal = (uint8_t)(compare & 0xFF);
}

/*******************************************************************************
 * Method for setting the OCRxB register.
 *
 * @param compare The compare value (unsigned byte).
 ******************************************************************************/
void timer16::setCompareValueB(uint16_t compare)
{
	*_ocrxbh = (uint8_t)((compare >> 8) & 0xFF);
	*_ocrxbl = (uint8_t)(compare & 0xFF);;
}

/*******************************************************************************
 * Method for setting the OCRxB register.
 *
 * @param compare The compare value (unsigned byte).
 ******************************************************************************/
void timer16::setCompareValueC(uint16_t compare)
{
	//*_ocrxch = (uint8_t)((compare >> 8) & 0xFF);
	//*_ocrxcl = (uint8_t)(compare & 0xFF);;
}

/*******************************************************************************
 * 
 ******************************************************************************/
int8_t timer16::setDutyCycleA(double dutyCycle)
{
	int8_t ret = 0;
	uint16_t compare = 0;
	
	if(_mode==t_mode::PWM_F or _mode==t_mode::PWM_PC or _mode==t_mode::PWM_FC)
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
	
	if(_mode==t_mode::PWM_F or _mode==t_mode::PWM_PC or _mode==t_mode::PWM_PC)
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
void timer16::set(uint16_t value)
{
	*_tcntxh = (uint8_t)((value >> 8) & 0xFF);
	*_tcntxl = (uint8_t)(value & 0xFF);
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
uint16_t timer16::getCount()
{
	uint16_t count;
	
	count = ((*_tcntxh << 8) | (*_tcntxl & 0xFF));
	
	return count;
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
uint16_t timer16::getCompareCount()
{
	return _compareCount;
}

/*******************************************************************************
 * 
 ******************************************************************************/
t_alias timer16::getAlias()
{
	return _alias;
}

/*******************************************************************************
 * 
 ******************************************************************************/
t_mode timer16::getMode()
{
	return _mode;
}

/*******************************************************************************
 * 
 ******************************************************************************/
t_interrupt timer16::getInterruptMode()
{
	return _interrupt;
}

/*******************************************************************************
 * 
 ******************************************************************************/
t_channel timer16::getPwmChannel()
{
	return _channel;
}

/*******************************************************************************
 * ISR for the timer class.
 ******************************************************************************/
void timer16::interruptServiceRoutine(void)
{
	if(_interrupt==t_interrupt::OVF)
	{
		_overflowCount++;
	}
	else if(_interrupt==t_interrupt::COMPA or _interrupt==t_interrupt::COMPB or _interrupt==t_interrupt::COMPC)
	{
		_compareCount++;
	}
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
 * Global forward declaration.
 ******************************************************************************/
timer16 * timer16::_t16[16] = {};

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
	if(timer16::_t16[n])timer16::_t16[n] -> interruptServiceRoutine();	\
}

#if defined(TIMER1_OVF_vect)
TIMER_OVF(1, 0)
#endif
#if defined(TIMER3_OVF_vect)
TIMER_OVF(3, 1)
#endif
#if defined(TIMER4_OVF_vect)
TIMER_OVF(4, 2)
#endif
#if defined(TIMER5_OVF_vect)
TIMER_OVF(5, 3)
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
	if(timer16::_t16[n])timer16::_t16[n] -> interruptServiceRoutine();	\
}

#if defined(TIMER1_COMPA_vect)
TIMER_COMPA(1, 4)
#endif
#if defined(TIMER3_COMPA_vect)
TIMER_COMPA(3, 5)
#endif
#if defined(TIMER4_COMPA_vect)
TIMER_COMPA(4, 6)
#endif
#if defined(TIMER5_COMPA_vect)
TIMER_COMPA(5, 7)
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
	if(timer16::_t16[n])timer16::_t16[n] -> interruptServiceRoutine();	\
}

#if defined(TIMER1_COMPB_vect)
TIMER_COMPB(1, 8)
#endif
#if defined(TIMER3_COMPB_vect)
TIMER_COMPB(3, 9)
#endif
#if defined(TIMER4_COMPB_vect)
TIMER_COMPB(4, 10)
#endif
#if defined(TIMER5_COMPB_vect)
TIMER_COMPB(5, 11)
#endif

/*******************************************************************************
 * COMPC -> ISR().
 * 
 * TODO::different avrs.
 * 
 * Call from self.
 ******************************************************************************/
#define TIMER_COMPC(t, n)							\
ISR(TIMER ## t ## _COMPC_vect)							\
{										\
	if(timer16::_t16[n])timer16::_t16[n] -> interruptServiceRoutine();	\
}

#if defined(TIMER1_COMPC_vect)
TIMER_COMPC(1, 12)
#endif
#if defined(TIMER3_COMPC_vect)
TIMER_COMPC(3, 13)
#endif
#if defined(TIMER4_COMPC_vect)
TIMER_COMPC(4, 14)
#endif
#if defined(TIMER5_COMPC_vect)
TIMER_COMPC(5, 15)
#endif