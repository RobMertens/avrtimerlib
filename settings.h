#ifndef _SETTINGS_H_
#define _SETTINGS_H_

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
	//CAPT  = 5;
	//COMPAB	= 6
};

/*******************************************************************************
 * PWM channels.
 ******************************************************************************/
enum class t_channel : uint8_t
{
	NONE	= 0,
	A	= 1,
	B	= 2,
	C	= 3,
	AB	= 4,
	BC	= 5,
	AC	= 6,
	ABC	= 7
};

#endif
