## avr-timer-lib

Timer library for Arduino Mega (atmega2560).

A hardware timer (8/16-bit) can be set in a few lines of code as shown in the
example below:

```c++
// Create the timer instance.
// We use the Arduino MEGA hardware Timer0.
Timer8 timer(t_alias::T0);

// Enter the timer settings.
// We set the timer to count to Normal TOP value (0xFF).
// And set an interrupt when TOP is reached.
timer.initialize(t_mode::NORMAL, t_interrupt::OVF0);

// The timer prescaler value determines the actual time it takes to up de timer
// count value. For a prescaler of 1 this means that we use one clock pulse.
// t = 1/f_clock = 1/16MHz = 0.0625 $\mu$s.
// The time for a new interrupt to occur will be at:
// t_int = 0.0625*(2^8 - 1) = 15.938 $\mu$s.
timer.setPrescaler(1);

// Reset the count value and start counting.
timer.reset();
```

Pointer to Timer instances are supported as shown in the following example. The
factory pattern in added to obtain the correct pointers.

```c++
// Create a base class timer pointer to an 8-bit timer.
Timer::Ptr t0_ptr(new Timer8(t_alias::T0));

// Create a const base class timer pointer to an 16-bit timer.
// This is a const pointer and NOT pointer to const.
Timer::CPtr t1_ptr(new Timer16(t_alias::T1));

// Instead of determining bitness a Timer Factory is implemented.
// Create an array for all the hardware timers of the arduino MEGA.
Timer::Ptr timer[6];
timer[0] = t0_ptr;
timer[1] = t1_ptr;
timer[2] = TimerFactory::startBelt()->produce(t_alias::T2);
timer[3] = TimerFactory::startBelt()->produce(t_alias::T3);
timer[4] = TimerFactory::startBelt()->produce(t_alias::T4);
timer[5] = TimerFactory::startBelt()->produce(t_alias::T5);
```

All AVR timer modes are supported. These modes are shown is the next table.
For a detailed description of the principles I refer you to the AVR MEGA2560
datasheet. The first three modes are combined with *interrupt* functions. The
last three *PWM*-modes are combined with the corresponding PWM settings.

| Modes        																		| Code specifier 		| Bitness	|
| ----------------------------------------------- | ----------------- | ------- |
| None      																			| `t_mode::NONE`		| 8/16		|
| Normal mode counts to TOP 											| `t_mode::NORMAL` 	| 8/16		|
| Clear Timer on Compare (CTC) 										| `t_mode::CTC` 		| 8/16		|
| Phase Width Modulation (PWM) Fast  							| `t_mode::PWM_F` 	| 8/16		|
| Phase Width Modulation (PWM) Phase Correct 			| `t_mode::PWM_PC` 	| 8/16		|
| Phase Width Modulation (PWM) Frequency Correct 	| `t_mode::PWM_FC` 	| 16			|

The next example explains how to set a timer to an interrupt mode.
