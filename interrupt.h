#ifndef _INTERRUPT_H_
#define _INTERRUPT_H_

/**
* interrupt.h 
*/
class interrupt { 
	public:
	    	class handler {
			public:
				//Constructors ***************************************************************

				//Setters ********************************************************************
				virtual void interruptServiceRoutine(void) = 0;
				virtual void enable() = 0;
				virtual void disable() = 0;
				virtual void clear() = 0;

				//Getters ********************************************************************
			private:
			
		};
	private:
};
#endif
