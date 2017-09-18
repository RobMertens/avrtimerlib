#ifndef _T_SETTINGS_H_
#define _T_SETTINGS_H_

#include <stdint.h>

namespace t_settings
{

/*******************************************************************************
 * Available timers on ATMEGA2560.
 *
 * T0 (8-bit),
 * T1 (16-bit),
 * T2 (8-bit),
 * T3 (16-bit),
 * T4 (16-bit),
 * T5 (16-bit),
 * TX (for non-atmega2560 timers).
 ******************************************************************************/
enum class alias : int8_t
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

/*******************************************************************************
 * Available timer modes:
 *
 * NONE 	: Non-operating mode.
 * NORMAL : Normal mode.
 * CTC		: Clear on Compare mode.
 * PWM_F	: Phase Width Modulation Fast mode.
 * PWM_PC	: Phase Width Modulation Phase Correct mode.
 * PWM_FC	: Phase Width Modulation Frequency Correct mode.
 ******************************************************************************/
enum class mode : uint8_t
{
	NONE		= 0,
	NORMAL	= 1,
	CTC			= 2,
	PWM_F		= 3,
	PWM_PC	= 4,
	PWM_FC	= 5
};

/*******************************************************************************
 * Interrupts only in NORMAL or CTC.
 ******************************************************************************/
enum class interrupt : uint8_t
{
	NONE		= 0,
	OVF			= 1,
	COMPA		= 2,
	COMPB		= 3,
	COMPC		= 4
	//CAPT  	= 5
};

/*******************************************************************************
 * PWM channels.
 *
 * The TOP channels are used to set a specific TOP value in OCRA. Note that you
 * lose resolution.
 ******************************************************************************/
enum class channel : uint8_t
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

/*******************************************************************************
 * PWM inverted.
 ******************************************************************************/
enum class inverted : uint8_t
{
	NONE		= 0,
	NORMAL	= 1,
	INV			= 2
};

}; //End namespace settings.

#endif
