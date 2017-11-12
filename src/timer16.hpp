/*******************************************************************************
 * avr-timer-lib
 * Timer16.hpp
 *
 * @brief This file belongs to the avr-timer-lib project. It provides functions
 *        for the arduino MEGA2560 16-bit timers.
 *
 * @author 	Rob Mertens
 * @date		11/11/2017
 * @version	1.1.1
 ******************************************************************************/

#ifndef avr_TIMER16_H
#define avr_TIMER16_H

//Include standard headers.
#include <stdint.h>

//Include avr headers.
#include <avr/interrupt.h>

//Include library headers.
#include "interrupt.hpp"
#include "timer.hpp"

/**
 * @brief The project namespace.
 */
namespace avr
{

/**
 * @brief Extern C timer1 overflow vector.
 */
extern "C" void TIMER1_OVF_vect(void) __attribute__ ((signal));

/**
 * @brief Extern C timer3 overflow vector.
 */
extern "C" void TIMER3_OVF_vect(void) __attribute__ ((signal));

/**
 * @brief Extern C timer4 overflow vector.
 */
extern "C" void TIMER4_OVF_vect(void) __attribute__ ((signal));

/**
 * @brief Extern C timer5 overflow vector.
 */
extern "C" void TIMER5_OVF_vect(void) __attribute__ ((signal));

/**
 * @brief Extern C timer1 compare A vector.
 */
extern "C" void TIMER1_COMPA_vect(void) __attribute__ ((signal));

/**
 * @brief Extern C timer3 compare A vector.
 */
extern "C" void TIMER3_COMPA_vect(void) __attribute__ ((signal));

/**
 * @brief Extern C timer4 compare A vector.
 */
extern "C" void TIMER4_COMPA_vect(void) __attribute__ ((signal));

/**
 * @brief Extern C timer5 compare A vector.
 */
extern "C" void TIMER5_COMPA_vect(void) __attribute__ ((signal));

/**
 * @brief Extern C timer1 compare B vector.
 */
extern "C" void TIMER1_COMPB_vect(void) __attribute__ ((signal));

/**
 * @brief Extern C timer3 compare B vector.
 */
extern "C" void TIMER3_COMPB_vect(void) __attribute__ ((signal));

/**
 * @brief Extern C timer4 compare B vector.
 */
extern "C" void TIMER4_COMPB_vect(void) __attribute__ ((signal));

/**
 * @brief Extern C timer5 compare B vector.
 */
extern "C" void TIMER5_COMPB_vect(void) __attribute__ ((signal));

/**
 * @brief Extern C timer1 compare C vector.
 */
extern "C" void TIMER1_COMPC_vect(void) __attribute__ ((signal));

/**
 * @brief Extern C timer3 compare C vector.
 */
extern "C" void TIMER3_COMPC_vect(void) __attribute__ ((signal));

/**
 * @brief Extern C timer4 compare C vector.
 */
extern "C" void TIMER4_COMPC_vect(void) __attribute__ ((signal));

/**
 * @brief Extern C timer5 compare C vector.
 */
extern "C" void TIMER5_COMPC_vect(void) __attribute__ ((signal));

/**
 * @brief Timer16 class definition. This class inherits from the abstract Timer
 *				class for polymorphism.
 */
class Timer16 : public Timer
{
	public:

		/** Typedefs **************************************************************/
		/**
		 * @brief A pointer typedef for Timer16 instance.
		 *				TODO::use smart pointers.
		 */
		typedef Timer16 * Ptr;

		/**
		 * @brief A const pointer typedef for Timer16 instance.
		 *				TODO::use smart pointers.
		 */
		typedef Timer16 * const CPtr;

		/** Constructors/destructor/overloading ***********************************/
		/**
		 * @brief
		 */
		Timer16(void);

		/**
		 * @brief
		 */
		explicit Timer16(const t_alias);

		/**
		 * @brief
		 */
		Timer16(const volatile uint8_t * const&, const volatile uint8_t * const&,
			const volatile uint8_t * const&, const volatile uint8_t * const&,
			const volatile uint8_t * const&, const volatile uint8_t * const&,
			const volatile uint8_t * const&, const volatile uint8_t * const&,
			const volatile uint8_t * const&);

		/**
		 * @brief Explicitly forbid a copy-constructor. We don't want two instances
		 *				interfacing the same hardware addresses.
		 *
		 * The "= delete" argument since c++11. Otherwise set this funtion private.
		 */
		Timer16(const Timer16&) = delete;

		/**
		 * @brief
		 */
		virtual ~Timer16(void);

		/**
		 * @brief NOTE::removed private.
		 */
		//Timer16& operator=(const Timer16&);

		/** Timer setting functions ***********************************************/
		/**
		 * @brief
		 */
		int8_t setAlias(const t_alias) override;

		/**
		 * @brief Method for setting the timer mode to NORMAL or CTC.
		 * 				This method cannot set mode to any PWM.
		 * @param mode The timer operation mode, only NORMAL or CTC.
		 * @param interrupt The interrupt mode, default is NONE.
		 * @param compare The compare value.
		 * @return
		 */
		int8_t initialize(const t_mode, const t_interrupt) override;

		/**
		 * @brief Method for setting the timer mode to PWM.
		 *				This method cannot set mode to NORMAL or CTC.
		 * @param mode The timer operation mode, only PWM.
		 * @param interrupt The interrupt mode, default is NONE.
		 * @param inverted TODO
		 * @return
		 */
		int8_t initialize(const t_mode, const t_channel, const t_inverted) override;

		/**
		 * @brief Method for setting the timer prescaler value.
		 *
		 * Value | Timer 1 | Timer 3 | Timer 4 | Timer 5
		 * 	   1 |    x    |    x		 |    x    |    x
		 * 	 	 8 |    x    |    x		 |    x    |    x
		 *    64 |    x    |    x		 |    x    |    x
		 *   256 |    x    |    x    |    x    |    x
		 *  1024 |    x    |    x    |    x    |    x
		 *
		 * @param The prescaler value.
		 * @return
		 */
		int8_t setPrescaler(const uint16_t);

		/** Timer running functions ***********************************************/
		/**
		 * @brief
		 */
		void set(const uint16_t);

		/**
		 * @brief
		 */
		void reset(void) override;

		/**
		 * @brief
		 */
		void hardReset(void) override;

		/**
		 * @brief
		 */
		void setCompareValueA(const uint16_t);

		/**
		 * @brief
		 * @
		 */
		int8_t setDutyCycleA(double);

		/**
		 * @brief
		 */
		void setCompareValueB(const uint16_t);

		/**
		 * @brief
		 */
		int8_t setDutyCycleB(double);

		/**
		 * @brief
		 */
		void setCompareValueC(const uint16_t);


		/**
		 * @brief
		 */
		int8_t setDutyCycleC(double);

		/**
		 * @brief
		 */
		uint16_t getCount(void);

		/**
		 * @brief
		 */
		uint32_t getOverflows(void);

		/**
		 * @brief
		 */
		uint32_t getNonResetCount(void);

		/** Interrupt functionality overrides *************************************/
		/**
		 * @brief
		 */
		void enable(void) override;

		/**
		 * @brief
		 */
		void disable(void) override;

		/**
		 * @brief
		 */
		void clear(void) override;

	private:

		/** Constructors/destructor/overloading ***********************************/
		/**
		 * @brief The equals operator is used locally. We don't want to make two
		 * 				instances at global scale interfacing with the same hardware
		 *				registers. However, this function is used locally for the ISR
		 *				mapping.
		 * @param The Timer16 instance to copy.
		 * @return The copied Timer16 instance.
		 */
		Timer16& operator=(const Timer16&);

		/** Interrupt functionality overrides *************************************/
		/**
		 * @brief
		 */
		void interruptServiceRoutine(void) override;

		/** Variables *************************************************************/
		/**
		 * @brief
		 */
		uint16_t prescale_;

		/**
		 * @brief TODO::move to abstract base class.
		 */
		uint32_t overflows_;

		/** Registers *************************************************************/
		/**
		 * @brief Timer count low byte.
		 */
		volatile uint8_t * tcntxl_;

		/**
		 * @brief Timer count high byte.
		 */
		volatile uint8_t * tcntxh_;

		/**
		 * @brief Prescaler setting byte A.
		 */
		volatile uint8_t * tccrxa_;

		/**
		 * @brief Prescaler setting byte B.
		 */
		volatile uint8_t * tccrxb_;

		/**
		 * @brief Timer Interrupt Mask register.
		 */
		volatile uint8_t * timskx_;

		/**
		 * @brief
		 */
		volatile uint8_t * ocrxal_;

		/**
		 * @brief
		 */
		volatile uint8_t * ocrxah_;

		/**
		 * @brief
		 */
		volatile uint8_t * ocrxbl_;

		/**
		 * @brief
		 */
		volatile uint8_t * ocrxbh_;

		/**
		 * @brief
		 */
		volatile uint8_t * ocrxcl_;

		/**
		 * @brief
		 */
		volatile uint8_t * ocrxch_;

		/**
		 * @brief Sets the register pointers to the appropriate values for timer1.
		 */
		void setRegistersT1(void);

		/**
		 * @brief Sets the register pointers to the appropriate values for timer3.
		 */
		void setRegistersT3(void);

		/**
		 * @brief Sets the register pointers to the appropriate values for timer4.
		 */
		void setRegistersT4(void);

		/**
		 * @brief Sets the register pointers to the appropriate values for timer5.
		 */
		void setRegistersT5(void);

		/** Timer mode functionalities ********************************************/
		/**
		 * @brief Sets the timer mode. It uses the private mode functions below.
		 * @param The desired timer mode.
		 * @return Is the operation successful ? [0] YES : [-1] NO.
		 */
		int8_t setMode(const t_mode) override;

		/**
		 * @brief Sets the timer mode to normal.
		 */
		void setMode2Normal(void);

		/**
		 * @brief Sets the timer mode to Clear Timer on Compare (CTC).
		 */
		void setMode2Ctc(void);

		/**
		 * @brief Sets the timer mode to Fast PWM.
		 */
		void setMode2FastPwm(void);

		/**
		 * @brief Sets the timer mode to Phase Correct PWM.
		 */
		void setMode2PhaseCorrectPwm(void);

		/**
		 * @brief Sets the timer mode to Frequency Correct PWM.
		 */
		void setMode2FrequencyCorrectPwm(void);

		/**
		 * @brief Sets the timer interrupt mode. This is only possible if the timer
		 * 				is set to the corresponding modes {NORMAL, CTC}.
		 * @param The desired timer interrupt mode.
		 * @return Is the operation successful ? [0] YES : [-1] NO.
		 */
		int8_t setInterruptMode(const t_interrupt) override;

		/**
		 * @brief Sets the timer PWM-channel. This is only possible if the timer is
		 * 				set to the corresponding modes {PWM_F, PWM_PC}.
		 * @param The desired timer PWM-channel.
		 * @return Is the operation successful ? [0] YES : [-1] NO.
		 */
		int8_t setPwmChannel(const t_channel, const t_inverted) override;

		/** Interrupt functions ***************************************************/
		/**
		 * @brief Static array of Timer16 instances. For each interrupt vector
	 	 * 				memory is allocated. This list is defined at global scope to Links
		 *				it with the appropriate ISR.
		 */
		static Timer16 * __T16__[17];

		/**
		 * @brief
		 */
		friend void TIMER1_OVF_vect(void);

		/**
		 * @brief
		 */
		friend void TIMER3_OVF_vect(void);

		/**
		 * @brief
		 */
		friend void TIMER4_OVF_vect(void);

		/**
		 * @brief
		 */
		friend void TIMER5_OVF_vect(void);

		/**
		 * @brief
		 */
		friend void TIMER1_COMPA_vect(void);

		/**
		 * @brief
		 */
		friend void TIMER3_COMPA_vect(void);

		/**
		 * @brief
		 */
		friend void TIMER4_COMPA_vect(void);

		/**
		 * @brief
		 */
		friend void TIMER5_COMPA_vect(void);

		/**
		 * @brief
		 */
		friend void TIMER1_COMPB_vect(void);

		/**
		 * @brief
		 */
		friend void TIMER3_COMPB_vect(void);

		/**
		 * @brief
		 */
		friend void TIMER4_COMPB_vect(void);

		/**
		 * @brief
		 */
		friend void TIMER5_COMPB_vect(void);

		/**
		 * @brief
		 */
		friend void TIMER1_COMPC_vect(void);

		/**
		 * @brief
		 */
		friend void TIMER3_COMPC_vect(void);

		/**
		 * @brief
		 */
		friend void TIMER4_COMPC_vect(void);

		/**
		 * @brief
		 */
		friend void TIMER5_COMPC_vect(void);

}; //End Timer16 class.

}; //End AVR namespace.

#endif //End avr_TIMER16_H wrapper.
