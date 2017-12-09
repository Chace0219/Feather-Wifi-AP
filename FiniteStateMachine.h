/*
||
|| @file 	FINITESTATEMACHINE.H
|| @version	1.0
|| @author	Jin zhouyun
|| @contact	2435575291@qq.com
|| State Machine Implementation
|| @description
||
*/

#ifndef FINITESTATEMACHINE_H
#define FINITESTATEMACHINE_H

#include <Arduino.h>

#define NO_ENTER (0)
#define NO_UPDATE (0)
#define NO_EXIT (0)

#define FSM FiniteStateMachine

//define the functionality of the states
class State {
	public:
		State( void (*updateFunction)() );
		State( void (*enterFunction)(), void (*updateFunction)(), void (*exitFunction)() );
		//State( byte newId, void (*enterFunction)(), void (*updateFunction)(), void (*exitFunction)() );
		
		//void getId();
		void enter();
		void update();
		void exit();
	private:
		//byte id;
		void (*userEnter)();
		void (*userUpdate)();
		void (*userExit)();
};

//define the finite state machine functionality
class FiniteStateMachine {
	public:
		FiniteStateMachine(State& current);
		
		FiniteStateMachine& update();
		FiniteStateMachine& transitionTo( State& state );
		FiniteStateMachine& immediateTransitionTo( State& state );
		
		State& getCurrentState();
		boolean isInState( State &state ) const;
		
		unsigned long timeInCurrentState();
		
	private:
		bool 	needToTriggerEnter;
		State* 	currentState;
		State* 	nextState;
		unsigned long stateChangeTime;
};

#endif

/*
|| @changelog
|| | 1.7 2010-03-08- Alexander Brevig : Fixed a bug, constructor ran update, thanks to René Pressé
|| | 1.6 2010-03-08- Alexander Brevig : Added timeInCurrentState() , requested by sendhb
|| | 1.5 2009-11-29- Alexander Brevig : Fixed a bug, introduced by the below fix, thanks to Jon Hylands again...
|| | 1.4 2009-11-29- Alexander Brevig : Fixed a bug, enter gets triggered on the first state. Big thanks to Jon Hylands who pointed this out.
|| | 1.3 2009-11-01 - Alexander Brevig : Added getCurrentState : &State
|| | 1.3 2009-11-01 - Alexander Brevig : Added isInState : boolean, requested by Henry Herman 
|| | 1.2 2009-05-18 - Alexander Brevig : enter and exit bug fix
|| | 1.1 2009-05-18 - Alexander Brevig : Added support for cascaded calls
|| | 1.0 2009-04-13 - Alexander Brevig : Initial Release
|| #
*/