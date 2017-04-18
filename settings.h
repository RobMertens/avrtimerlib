#ifndef _SETTINGS_H_
#define _SETTINGS_H_

enum class t_mode : short
{
	NORMAL = 0,
	CTC    = 1,
	PWM    = 2
};

enum class t_interrupt : short
{
	NONE  = 0,
	OVF   = 1,
	COMPA = 2, // T2
	COMPB = 3 // T2
	//COMPC = 5;
	//CAPT  = 6;
};

#endif
