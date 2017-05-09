#ifndef _T_SETTINGS_H_
#define _T_SETTINGS_H_

#include <stdint.h>

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
enum class t_alias : int8_t
{
	NONE	= -1,
	T0	= 0,
	T1	= 1,
	T2	= 2,
	T3	= 3,
	T4	= 4,
	T5	= 5,
	TX	= 6
};

/*******************************************************************************
 * Available timer modes:
 * 
 * NONE 	: Non-operating mode.
 * NORMAL 	: Normal mode.
 * CTC		: Clear on Compare mode.
 * PWM		: Phase Width Modulation mode.
 ******************************************************************************/
enum class t_mode : uint8_t
{
	NONE	= 0,
	NORMAL	= 1,
	CTC	= 2,
	PWM_F	= 3,
	PWM_PC	= 4,
	PWM_FC	= 5
};

/*******************************************************************************
 * Interrupts only in NORMAL or CTC.
 ******************************************************************************/
enum class t_interrupt : uint8_t
{
	NONE	= 0,
	OVF	= 1,
	COMPA	= 2,
	COMPB	= 3,
	COMPC	= 4
	//CAPT  = 5
};

/*******************************************************************************
 * PWM channels.
 ******************************************************************************/
enum class t_channel : uint8_t
{
	NONE	= 0,
	A	= 1,
	B	= 2,
	B_TOP	= 3,
	C	= 4,
	C_TOP	= 5,
	AB	= 6,
	AC	= 7,
	BC	= 8,
	BC_TOP	= 9,
	ABC	= 10
};

/*******************************************************************************
 * PWM inverted.
 ******************************************************************************/
enum class t_inverted : uint8_t
{
	NONE	= 0,
	NORMAL	= 1,
	INV	= 2
};

#endif
