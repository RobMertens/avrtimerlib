## avr-timer-lib

Timer library for Arduino Mega (atmega2560). The available timers on the atmega2560 are shown in the following table. The extern timer is not really a atmega2560 timer. Other AVR-timers and other arduino boards could be initiated with this timer. However, this is still in testing phase.

| Timer     		| Code specifier 		| Bitness	|
| ------------- | ----------------- | ------- |
| Timer 0    		| `t_alias::T0`			| 8				|
| Timer 1				| `t_alias::T1` 		| 16			|
| Timer 2				| `t_alias::T2` 		| 8				|
| Timer 3				| `t_alias::T3` 		| 16			|
| Timer 4				| `t_alias::T4` 		| 16			|
| Timer 5				| `t_alias::T5` 		| 16			|
| Timer extern	| `t_alias::TX` 		| 8/16		|

### Easy to use

A hardware timer (8/16-bit) can be set in a few lines of code as shown in the
example below:

```c++
// Create the timer instance.
// We use the Arduino MEGA hardware Timer0.
Timer8 t0(t_alias::T0);

// Enter the timer settings.
// We set the timer to count to Normal TOP value (0xFF).
// And set an interrupt when TOP is reached.
t0.initialize(t_mode::NORMAL, t_interrupt::OVF0);

// The timer prescaler value determines the actual time it takes to up de timer
// count value. For a prescaler of 1 this means that we use one clock pulse.
// t = 1/f_clock = 1/16MHz = 0.0625 $\mu$s.
// The time for a new interrupt to occur will be at:
// t_int = 0.0625*(2^8 - 1) = 15.938 $\mu$s.
t0.setPrescaler(1);

// Reset the count value and start counting.
t0.reset();
```

Pointer to Timer instances are supported as shown in the following example. The
factory pattern in added to obtain the correct pointers.

```c++
// Create a base class timer pointer to the 8-bit timer from the code block above.
Timer::Ptr t0_ptr = &t0;

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

### Ticks and time

Ok, now the math part appears. The concepts of tick and time are a must to initiate them properly. The Arduino MEGA runs on a clock frequency of 16 MHz which equals 16.000.000 Hz. The time it takes for one tick is equal to 0.0625e-8[!] us which is very very fast. An 8-bit timer uses one byte of information. The number of ticks to reach the maximum value is equal to 255 (2^8 - 1). This means that the timer reaches its TOP count value in 15.94 us.

//TODO::formula here

Every clock-tick the timer ups the count value. This means that the time corresponding to each tick equals:

//TODO::formula here

The time per timer tick depends on the oscillator on the Arduino and the prescale value. The timer tick value [\ms] is calculated as follows.



To calculate the total count time, we multiply it by the number of ticks depending on bitness and specified TOP.



AVR timers have two distict operation modes. Firstly the **interrupt** mode. In this mode the timer count to a specific value, either TOP or a manually set value. When the timer reaches this value an interrupt is fired. These timer modes are shown in the table below.

| Interrupt modes        													| Code specifier 		| Bitness	|
| ----------------------------------------------- | ----------------- | ------- |
| None      																			| `t_mode::NONE`		| 8/16		|
| Normal mode counts to TOP 											| `t_mode::NORMAL` 	| 8/16		|
| Clear Timer on Compare (CTC) 										| `t_mode::CTC` 		| 8/16		|

The next example explains how to set a timer to an interrupt mode.

```c++
// We use the timer2 instance from the code block above.
// The NORMAL specifier indicates that the timer counts to TOP.
// Since timer2 is an 8-bit timer, the maximum count value is 0xFE.
// The OVF0 specifier sets the interrupt flag on TOP.
timer[2]->initialize(t_mode::NORMAL, t_interrupt::OVF0);

// For the next example we want the timer to count to a desired value.
// Let's say that this value is 20000 clocks, e.g.: 0x4E20 (HEX).
// The CTC specifier clears the timer count on compare value.
// The COMPA specifier fires an interrupt on overflow.
timer[3]->initialize(t_mode::CTC, t_interrupt::COMPA);
timer[3]->setCompareValueA(0x4E20);

// The next line sets no interruptflag.
// NOTE::the InterruptServiceRoutine() ups the timer overflow counter.
// The nonResetCount() funtion does not work here.
timer[3]->initialize(t_mode::CTC, t_interrupt::NONE);
timer[3]->setCompareValueA(0x4E20);

// The following example shows how to NOT set a timer.
// We set the timer to clear on compare with 20000 and set an interrupt on TOP.
// This means that the timer won't reach it's
timer[3]->initialize(t_mode::CTC, t_interrupt::OVF);
timer[3]->setCompareValueA(0x4E20);
```

The second distinct operation modes are **Phase Width Modulation** (PWM). In these modes interrupts and corresponding vectors do not work. This library provides easy-to-use funtions to specify a timer as PWM-signal. The supported modes are listed below. The Fast PWM is considered as the most standard form of PWM with the recognizable saw-tooth curve. For the other modes I refer to the AVR-manual. An 8-bit timer has two channels A and B, and an 16-bit timer has three channels A,B and C. This means you can obtain either two or three PWM-signals from one timer!

| Phase Width Modulation (PWM) modes  						| Code specifier 		| Bitness	|
| ----------------------------------------------- | ----------------- | ------- |
| None      																			| `t_mode::NONE`		| 8/16		|
| Phase Width Modulation (PWM) Fast  							| `t_mode::PWM_F` 	| 8/16		|
| Phase Width Modulation (PWM) Phase Correct 			| `t_mode::PWM_PC` 	| 8/16		|
| Phase Width Modulation (PWM) Frequency Correct 	| `t_mode::PWM_FC` 	| 16			|

```c++
// Since timer4 is an 16-bit timer the maximum count value is 65535 (2^16 - 1).
// This means that the timer counts to 4095.94 us.
// Setting the duty cycle to 0.75% gives ON-time of 0.75*4095.9375 = 3071.95 us.
timer[4]->initialize(t_mode::PWM_F, t_channel::A, t_inverted::NORMAL);
timer[4]->setPrescaler(1);
timer[4]->setDutyCycleA(0.75);

// To invert the channel we set the INV specifier which gives us 3071.95 us OFF-time.
timer[4]->initialize(t_mode::PWM_F, t_channel::A, t_inverted::INV);
```

The period chosen in the latter example is a bit tedious. So let's set a timer with desired period. Let's say we want a PWM-signal with a period of 4 ms. The number of the timer TOP value can be found by the formulas above. For our example this means to set the top value to 63999 ticks with a prescaler of 1. This fits within the 16-bit timer maximum count value (65355). Which prescaler and TOP value must be set for an 8-bit timer? Try it yourself!

To set the 16-bit timer to a PWM-signal with period of 4 ms we look at the next code block example. The TOP specifier after the channel declaration sets the timer with desired TOP value. Note that you lose one PWM-channel since the TOP value is set in the OCRA register.

```c++
// Set timer4 to a PWM-signal with a period of 4000 us.
// The PWM-outputs are specified at channel B and C.
timer[4]->initialize(t_mode::PWM_F, t_channel::BC_TOP, t_inverted::NORMAL);
timer[4]->setPrescaler(1);
timer[4]->setCompareValueA(63999);
timer[4]->setDutyCycleB(0.75);
timer[4]->setDutyCycleC(0.67);
```

## Future extensions

* Arduino UNO support.
* The current interrupt service routine ups the overflow counter. However, a lot more ISR's could be usefull. In the future I'll provide some method to override the class member ISR() function.
