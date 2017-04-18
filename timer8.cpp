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
 * Global forward declaration.
 ******************************************************************************/
static timer8 *_t;

/*******************************************************************************
 * Constructor for a 8-bit timer, e.g.: timer0 on MEGA2560.
 * 
 * TODO::auto pin-map w/ alias function.
 *
 * @param: tccr The timer control register.
 * @param: tcnt The timer count register.
 ******************************************************************************/
timer8::timer8(uint8_t alias, volatile uint8_t * tccrxa, volatile uint8_t * tccrxb, volatile uint8_t * tcntx, volatile uint8_t * timskx, volatile uint8_t * ocrxa, volatile uint8_t * ocrxb)
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
void timer8::setAlias(uint8_t alias)
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
int8_t timer8::setMode(t_mode mode, t_interrupt interrupt, uint8_t compare)
{
	int8_t ret = 0;
	
	_mode = t_mode::NONE;
	_interrupt = t_interrupt::NONE;
	
	//MODES.
	switch(mode)
	{
		case t_mode::NORMAL :
			setMode2Normal();
			break;
	
		case t_mode::CTC :
			setMode2Ctc();
			break;
		
		case t_mode::PWM :
		case t_mode::NONE :
		default :
			//TODO::reset timer.
			ret = -1;
			return ret;
	}
	
	//TODO::INTERRUPTS.
	ret = setInterruptMode(interrupt, compare);
	
	//RETURN.
	if(!ret)
	{
		_mode = mode;
		_interrupt = interrupt;
	}
	else
	{
		hardReset();
	}
	
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
int8_t timer8::setMode(t_mode mode, t_pwm pwm)
{
	int8_t ret = 0;
	
	_mode = t_mode::NONE;
	_pwm = t_pwm::NONE;
	
	//MODES.
	if(mode==t_mode::PWM)
	{
		switch(pwm)
		{
			case t_pwm::FAST :
				setMode2Ctc();
				break;
		
			case t_pwm::PC :
				ret = -1;
				break;
			
			case t_pwm::NONE :
			default :
				//TODO::reset timer.
				ret = -1;
				break;
		}
	}
	else
	{
		ret = -1;
	}
	
	//TODO::INVERTED MODE.
	
	//RETURN.
	if(!ret)
	{
		_mode = mode;
		_interrupt = interrupt;
	}
	else
	{
		hardReset();
	}
	
	return ret;
}

void timer8::setMode2Normal()
{
	//Normal mode.
	*_tccrxa &= ~(1 << 0);					// WGMx0 = 0;
	*_tccrxa &= ~(1 << 1);					// WGMx1 = 0;
	*_tccrxb &= ~(1 << 3); 					// WGMx2 = 0;
	
	//TODO::OCRxA
	//TODO::normal + interrupt mode.
}

void timer8::setMode2Ctc()
{
	//CTC Mode.
	*_tccrxa &= ~(1 << 0);					// WGMx0 = 0;
	*_tccrxa |=  (1 << 1);					// WGMx1 = 1;
	*_tccrxb &= ~(1 << 3); 					// WGMx2 = 0;
	
	//TODO::OCRxA
	//TODO::interrupt mode.
}

void timer8::setMode2FastPwm()
{
	//Fast PWM.
	*_tccrxa |=  (1 << 0);				// WGMx0 = 1;
	*_tccrxa |=  (1 << 1);				// WGMx1 = 1;
	*_tccrxb |=  (1 << 3); 				// WGMx2 = 1;
	
	//TODO::OCRxA
	//TODO::inverted (y/n)
}

void timer8::setMode2PhaseCorrectPwm()
{
	//Phase Correct PWM.
	*_tccrxa |= ~(1 << 0);				// WGMx0 = 1;
	*_tccrxa &=  (1 << 1);				// WGMx1 = 0;
	*_tccrxb |= ~(1 << 3); 				// WGMx2 = 1;
	
	//TODO::OCRxA
	//TODO::inverted (y/n)
}

/*******************************************************************************
 * Method for setting the timer interrupt mode (NONE/OVF/COMPA/COMPB).
 *
 * @param: interrupt The timer interrupt mode.
 ******************************************************************************/
int8_t timer8::setInterruptMode(t_interrupt interrupt, uint8_t compareValue)
{
	int8_t ret = 0;
	
	_interrupt = interrupt;
	
	//INTERRUPT MODE.
	switch(_interrupt)
	{
		case t_interrupt::NONE : 
			*_timskx = 0x00;
			break;
	
		case t_interrupt::OVF : 
			*_timskx = 0x01;
			break;
	
		case t_interrupt::COMPA :
			*_timskx = 0x02;
			*_ocrxa  = comp;
			break;
	
		case t_interrupt::COMPB :
			*_timskx = 0x04;
			*_ocrxb  = comp;
			break;
		default :
			ret = -1;
			break;
	}
	
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
void timer8::setPrescaler(uint16_t prescale)
{
	_prescale = prescale;
	
	switch(_prescale)
	{
		case 0 :
			// Timer inactive.
			*_tccrxb &= ~(1 << 0);					// CSx0 = 0.
			*_tccrxb &= ~(1 << 1);					// CSx1 = 0.
			*_tccrxb &= ~(1 << 2);					// CSx2 = 0.
			break;
		
		case 1 :
			*_tccrxb |=  (1 << 0);					// CSx0 = 1.
			*_tccrxb &= ~(1 << 1);					// CSx1 = 0.
			*_tccrxb &= ~(1 << 2);					// CSx2 = 0.
			break;
		
		case 8 :
			*_tccrxb &= ~(1 << 0);					// CSx0 = 0.
			*_tccrxb |=  (1 << 1);					// CSx1 = 1.
			*_tccrxb &= ~(1 << 2);					// CSx2 = 0.
			break;
	
		case 32 : 
			*_tccrxb |=  (1 << 0);					// CS20 = 1.
			*_tccrxb |=  (1 << 1);					// CS21 = 1.
			*_tccrxb &= ~(1 << 2);					// CS22 = 0.
			break;
		
		case 64 :
			if(_alias==0)
			{
				*_tccrxb |=  (1 << 0);				// CS00 = 1.
				*_tccrxb |=  (1 << 1);				// CS01 = 1.
				*_tccrxb &= ~(1 << 2);				// CS02 = 0.
			}
			else
			{
				*_tccrxb &= ~(1 << 0);				// CS20 = 0.
				*_tccrxb &= ~(1 << 1);				// CS21 = 0.
				*_tccrxb |=  (1 << 2);				// CS22 = 1.
			}
			break;
		
		case 256 :
			*_tccrxb &= ~(1 << 0);					// CS00 = 0.
			*_tccrxb &= ~(1 << 1);					// CS01 = 0.
			*_tccrxb |=  (1 << 2);					// CS02 = 1.
			break;
	
		case 1024 :
			*_tccrxb |=  (1 << 0);					// CS00 = 1.
			*_tccrxb &= ~(1 << 1);					// CS01 = 0.
			*_tccrxb |=  (1 << 2);					// CS02 = 1.
			break;
	}
	
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

/**
 * 
 */
uint16_t timer8::getOverflowCount()
{
	return _overflowCount;
}

/*******************************************************************************
 * ISR for the timer class.
 ******************************************************************************/
void timer8::interruptServiceRoutine(void)
{
	//TODO::check which interrupt is active.
	_overflowCount++;						// timer_ovf_vect()
	//_interruptCount++;						// timer_comp_vect()
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
 * ISR -> myISR().
 * 
 * TODO::different vectors and chips.
 * 
 * Call from self.
 ******************************************************************************/
ISR(TIMER2_OVF_vect)
{
	if(_t)_t -> interruptServiceRoutine();
}

 
