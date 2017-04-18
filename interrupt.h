#ifndef _INTERRUPT_H_
#define _INTERRUPT_H_

/**
* interrupt.h 
*/
class interrupt { 
	public:
	    	class handler {
	    		//Constructors ***************************************************************
		
	    		//Setters ********************************************************************
	    		virtual void interruptServiceRoutine(void) = 0;
	    		virtual void enable();
	    		virtual void disable();
	    		virtual void clear();
	    		
	    		//Getters ********************************************************************
		   
			};
	private:
};
#endif
