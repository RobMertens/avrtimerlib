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
	PWM_PC	= 4
};

/*******************************************************************************
 * Interrupts only in NORMAL or CTC.
 ******************************************************************************/
enum class t_interrupt : uint8_t
{
	NONE	= 0,
	OVF	= 1,
	COMPA	= 2,
	COMPB	= 3
	//COMPAB= 4
	//COMPC = 4;
	//CAPT  = 5;
};

/*******************************************************************************
 * PWM channels.
 ******************************************************************************/
enum class t_channel : uint8_t
{
	NONE	= 0,
	A	= 1,
	B	= 2,
	AB	= 3
};

#endif
