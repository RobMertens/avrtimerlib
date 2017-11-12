/**
 * file vmath.hpp
 *
 * @brief This file belongs to the qc_library project. It provides functions
 *        for Vector and rotation math.
 *
 * @author 	Rob Mertens
 * @date		11/11/2017
 */

#ifndef avr_TIMER8_H
#define avr_TIMER8_H

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
 * @brief Extern C timer0 overflow vector.
 */
extern "C" void TIMER0_OVF_vect(void) __attribute__((signal));

/**
 * @brief Extern C timer2 overflow vector.
 */
extern "C" void TIMER2_OVF_vect(void) __attribute__((signal));

/**
 * @brief Extern C timer0 compare A vector.
 */
extern "C" void TIMER0_COMPA_vect(void) __attribute__((signal));

/**
 * @brief Extern C timer2 compare A vector.
 */
extern "C" void TIMER2_COMPA_vect(void) __attribute__((signal));

/**
 * @brief Extern C timer0 compare B vector.
 */
extern "C" void TIMER0_COMPB_vect(void) __attribute__((signal));

/**
 * @brief Extern C timer2 compare B vector.
 */
extern "C" void TIMER2_COMPB_vect(void) __attribute__((signal));

/**
 * @brief Timer8 class definition. This class inherits from the abstract Timer
 *				class for polymorphism.
 */
class Timer8 : public Timer
{

	public:

		/** Typedefs **************************************************************/
		/**
		 * @brief A pointer typedef for Timer8 instance.
		 *				TODO::use smart pointers.
		 */
		typedef Timer8 * Ptr;

		/**
		 * @brief A constant pointer typedef for Timer8 instance.
		 *				TODO::use smart pointers.
		 */
		typedef Timer8 * const CPtr;

		/** Constructors/destructor/overloading ***********************************/
		/**
		 * @brief
		 */
		Timer8(void);

		/**
		 * @brief
		 * @param
		 */
		explicit Timer8(const t_alias);

		/**
		 * @brief
		 * @param
		 */
		Timer8(const volatile uint8_t * const&, const volatile uint8_t * const&,
			const volatile uint8_t * const&, const volatile uint8_t * const&,
			const volatile uint8_t * const&, const volatile uint8_t * const&);

		/**
		 * @brief Explicitly forbid a copy-constructor. We don't want two instances
		 *				interfacing the same hardware addresses.
		 *
		 * The "= delete" argument since c++11. Otherwise set this funtion private.
		 */
		Timer8(const Timer8&) = delete;

		/**
		 * @brief
		 */
		virtual ~Timer8(void);

		/**
		 * @brief NOTE::removed private.
		 */
		//Timer8& operator=(const Timer8&);

		/** Timer setting functions ***********************************************/
		/**
		 * @brief
		 */
		int8_t setAlias(const t_alias) override;

		/**
		 * @brief
		 */
		int8_t initialize(const t_mode, const t_interrupt) override;

		/**
		 * @brief
		 */
		int8_t initialize(const t_mode, const t_channel,
			const t_inverted) override;

		/**
		 * @brief
		 */
		int8_t setPrescaler(const uint16_t);

		/** Timer runtime functions ***********************************************/
		/**
		 * @brief
		 */
		void set(const uint8_t);

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
		void setCompareValueA(const uint8_t);

		/**
		 * @brief
		 * @param[out]
		 */
		int8_t setDutyCycleA(double);

		/**
		 * @brief
		 */
		void setCompareValueB(const uint8_t);

		/**
		 * @brief
		 * @param[out]
		 */
		int8_t setDutyCycleB(double);

		/**
		 * @brief
		 */
		uint8_t getCount(void);

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
		 * @param The Timer8 instance to copy.
		 * @return The copied Timer8 instance.
		 */
		Timer8& operator=(const Timer8&);

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
		 * @brief
		 */
		uint32_t overflows_;

		/** Registers *************************************************************/
		/**
		 * @brief
		 */
		volatile uint8_t * tcntx_;			// TIMER COUNT

		/**
		 * @brief
		 */
		volatile uint8_t * tccrxa_;			// PRESCALER

		/**
		 * @brief
		 */
		volatile uint8_t * tccrxb_;			// PRESCALER

		/**
		 * @brief
		 */
		volatile uint8_t * timskx_;			// Timer Interrupt Mask register.

		/**
		 * @brief
		 */
		volatile uint8_t * ocrxa_;

		/**
		 * @brief
		 */
		volatile uint8_t * ocrxb_;

		/**
		 * @brief Sets the register pointers to the appropriate values for timer0.
		 */
		void setRegistersT0(void);

		/**
		 * @brief Sets the register pointers to the appropriate values for timer2.
		 */
		void setRegistersT2(void);

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
		 * @brief Static array of Timer8 instances. For each interrupt vector
	 	 * 				memory is allocated. This list is defined at global scope to Links
		 *				it with the appropriate ISR.
		 */
		static Timer8::Ptr __T8__[7];

		/**
		 * @brief Friend timer0 overflow vector. This vector is standard for each
		 * 				arduino and thus standard friend.
		 */
		friend void TIMER0_OVF_vect(void);

		/**
		 * @brief Friend timer2 overflow vector.
		 */
		friend void TIMER2_OVF_vect(void);

		/**
		 * @brief Friend timer0 compare A vector.
		 */
		friend void TIMER0_COMPA_vect(void);

		/**
		 * @brief Friend timer2 compare A vector.
		 */
		friend void TIMER2_COMPA_vect(void);

		/**
		 * @brief Friend timer0 compare B vector.
		 */
		friend void TIMER0_COMPB_vect(void);

		/**
		 * @brief Friend timer2 compare B vector.
		 */
		friend void TIMER2_COMPB_vect(void);

}; //End Timer8 class.

}; //End AVR namespace.

#endif
