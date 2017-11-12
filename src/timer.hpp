/**
 * file timer.hpp
 *
 * @brief This file belongs to the avr_timer_lib project.
 *
 * @author 	Rob Mertens
 * @date		27/10/2017
 */

#ifndef avr_TIMER_HPP
#define avr_TIMER_HPP

//Include standard headers.
#include <stdint.h>

//Include library headers.
#include "interrupt.hpp"
/**
 * @brief The project namespace.
 */
namespace avr
{

	/** Settings for the timer ************************************************/
	/**
	 * @brief Available timers on ATMEGA2560.
	 *
	 * T0 (8-bit),
	 * T1 (16-bit),
	 * T2 (8-bit),
	 * T3 (16-bit),
	 * T4 (16-bit),
	 * T5 (16-bit),
	 * TX (for non-atmega2560 avr-timers).
	 */
	enum class t_alias : int8_t
	{
		NONE		= -1,
		T0			= 0,
		T1			= 1,
		T2			= 2,
		T3			= 3,
		T4			= 4,
		T5			= 5,
		TX			= 6
	};

	/**
	 * @brief Available timer modes.
	 *				TODO::move this to Timer8/16 since the possibilities differ.
	 *
	 * NONE 	: Non-operating mode.
	 * NORMAL : Normal mode.
	 * CTC		: Clear on Compare mode.
	 * PWM_F	: Phase Width Modulation Fast mode.
	 * PWM_PC	: Phase Width Modulation Phase Correct mode.
	 * PWM_FC	: Phase Width Modulation Frequency Correct mode.
	 */
	enum class t_mode : uint8_t
	{
		NONE		= 0,
		NORMAL	= 1,
		CTC			= 2,
		PWM_F		= 3,
		PWM_PC	= 4,
		PWM_FC	= 5
	};

	/**
	 * @brief Interrupts only in NORMAL or CTC.
	 *				TODO::move this to Timer8/16 since the possibilities differ.
	 *
	 * NONE 	: No interrupts.
	 * OVF 		: Overflow flag.
	 * COMPA 	: Compare register A flag.
	 * COMPB 	: Compare register B flag.
	 * COMPC 	: Compare register C flag.
	 * //CAPT 		: Capture flag not (yet) supported.
	 */
	enum class t_interrupt : uint8_t
	{
		NONE		= 0,
		OVF			= 1,
		COMPA		= 2,
		COMPB		= 3,
		COMPC		= 4
		//CAPT  	= 5
	};

	/**
	 * @brief PWM channels.
	 *				TODO::move this to Timer8/16 since the possibilities differ.
	 *
	 * The TOP channels are used to set a specific TOP value in OCRA. Note that
	 * you lose resolution.
	 */
	enum class t_channel : uint8_t
	{
		NONE		= 0,
		A				= 1,
		B				= 2,
		B_TOP		= 3,
		C				= 4,
		C_TOP		= 5,
		AB			= 6,
		AC			= 7,
		BC			= 8,
		BC_TOP	= 9,
		ABC			= 10
	};

	/**
	 * @brief PWM inverted.
	 */
	enum class t_inverted : uint8_t
	{
		NONE		= 0,
		NORMAL	= 1,
		INV			= 2
	};

/**
 * @brief Abstract timer class. Links both Timer8 and Timer16 to one type.
 */
class Timer : public Interrupt::Handler
{

	public:

		/** Typedefs ****************************************************************/
		/**
		 * @brief A pointer typedef for Timer instance. This is an important feature
		 *				since they will be used to point to Timer8 and Timer16 instances.
		 */
		typedef Timer * Ptr;

		/**
		 * @brief A constant pointer typedef for Timer instance. This is an important
		 *				feature since they will be used to point to Timer8 and Timer16
		 *				instances.
		 */
		typedef Timer * const CPtr;

		/** Constructors/destructor/overloading *************************************/
		/**
		 * @brief Constructor for an arbitrary timer, an Abstract Base Class (ABC).
		 * 				This constructor serves as base instance for a Timer8 and Timer16
		 *				class.
		 * @param The timer alias.
		 */
		Timer(void) {}

		/**
		 * @brief Constructor for an arbitrary timer, an Abstract Base Class (ABC).
		 * 				This constructor serves as base instance for a Timer8 and Timer16
		 *				class.
		 * @param The timer alias.
		 */
		explicit Timer(const t_alias alias) : alias_(alias) {}

		/**
		 * @brief Destructor for the arbitrary timer, an Abstract Base Class (ABC).
		 */
		virtual ~Timer(void) {}

		/** Timer runtime function overrides ****************************************/
		/**
		 * @brief
		 */
		//virtual void set(const uint8_t) = 0;

		/**
		 * @brief
		 */
		virtual void reset(void) = 0;

		/**
		 * @brief
		 */
		virtual void hardReset(void) = 0;

		/** Timer setting functions ***********************************************/
		/**
		 * @brief
		 */
		virtual int8_t setAlias(const t_alias) = 0;

		/**
		 * @brief
		 */
		virtual int8_t initialize(const t_mode, const t_interrupt) = 0;

		/**
		 * @brief
		 */
		virtual int8_t initialize(const t_mode, const t_channel,
			const t_inverted) = 0;

		/**
		 * @brief Method for obtaining the actual timer alias.
		 * @return The current timer alias.
		 */
		t_alias	getAlias(void) { return alias_; }

		/**
		 * @brief Method for obtaining the actual timer mode.
		 * @return The current timer mode.
		 */
		t_mode getMode(void) { return mode_; }

		/**
		 * @brief Method for obtaining the actual timer interrupt mode.
		 * @return The current timer interrupt mode.
		 */
		t_interrupt	getInterruptMode(void) { return interrupt_; }

		/**
		 * @brief Method for obtaining the actual timer PWM-channel.
		 * @return The current timer PWM-channel.
		 */
		t_channel	getChannel(void) { return channel_; }

		/**
		 * @brief Method for obtaining the actual timer PWM-signal inversion.
		 * @return Is the signal inverted or not.
		 */
		t_inverted getInverted(void) { return inverted_; }

	protected:

		/** Variables ***************************************************************/
		/**
		 * @brief PWM inverted.
		 */
		t_alias alias_;

		/**
		 * @brief
		 */
		t_mode mode_;

		/**
		 * @brief
		 */
		t_interrupt interrupt_;

		/**
		 * @brief
		 */
		t_channel channel_;

		/**
		 * @brief
		 */
		t_inverted inverted_;

	private:

		/**
		 * @brief Virtual function for setting the timer mode.
		 */
		virtual int8_t setMode(const t_mode) = 0;

		/**
		 * @brief Virtual function for setting the interrupt mode.
		 */
		virtual int8_t setInterruptMode(const t_interrupt) = 0;

		/**
		 * @brief Virtual function for setting the PWM channel in PWM mode.
		 * @param The PWM-channel.
		 * @param Invert the PWN-signal.
		 */
		virtual int8_t setPwmChannel(const t_channel, const t_inverted) = 0;

}; //End abstract Timer class.

}; //End avr namespace.

#endif //End avr_TIMER_HPP wrapper.
