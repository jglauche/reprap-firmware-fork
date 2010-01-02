// Yep, this is actually -*- c++ -*-

/*
  This is a fork of Repraps FiveD Gcode Firmware
  License: GPL
  
  Changes from official firmware:
    - added support for Arduino Mega
    - added support fo Arudino Duemilanove w/ ATMega328P
    - changed determination of using the extruder controller from MOTHERBOARD > 1 to USE_EXTRUDER_CONTROLLER 
    - fixed some bugs checking unsigned byte < 0 ...
    - updated interrupt function SIGNAL -> ISR
    - re-enabled 3D gcode support
    - added auto-shutdown on idle
    - added base heater command
*/


#include <ctype.h>
#include <HardwareSerial.h>
#include "WProgram.h"
#include "vectors.h"
#include "parameters.h"
#include "intercom.h"
#include "pins.h"
#include "extruder.h"
#include "cartesian_dda.h"

// Maintain a list of extruders...

#include "WProgram.h"
void setup();
void loop();
inline bool qFull();
inline bool qEmpty();
inline void dQMove();
inline void setUnits(bool u);
void setupTimerInterrupt();
void setTimerResolution(byte r);
inline void setTimer(long delay);
void manage_all_extruders();
void new_extruder(byte e);
inline void init_process_string();
bool get_and_do_command();
int parse_string(struct GcodeParser * gc, char instruction[ ], int size);
bool process_string(char instruction[], int size);
int scan_float(char *str, float *valp, unsigned int *seen, unsigned int flag);
int scan_int(char *str, int *valp, unsigned int *seen, unsigned int flag);
extruder* ex[EXTRUDER_COUNT];
byte extruder_in_use = 0;

// Text placed in this (terminated with 0) will be transmitted back to the host
// along with the next G Code acknowledgement.
char debugstring[10];

#if USE_EXTRUDER_CONTROLLER == false

// TODO: For some reason, if you declare the following two in the order ex0 ex1 then
// ex0 won't drive its stepper.  They seem fine this way round though.  But that's got
// to be a bug.

#if EXTRUDER_COUNT == 2            
static extruder ex1(EXTRUDER_1_MOTOR_DIR_PIN, EXTRUDER_1_MOTOR_SPEED_PIN , EXTRUDER_1_HEATER_PIN,
              EXTRUDER_1_FAN_PIN,  EXTRUDER_1_TEMPERATURE_PIN, EXTRUDER_1_VALVE_DIR_PIN,
              EXTRUDER_1_VALVE_ENABLE_PIN, EXTRUDER_1_STEP_ENABLE_PIN, EXTRUDER_1_LOW_HEAT, EXTRUDER_1_HIGH_HEAT);            
#endif
   static extruder ex0(EXTRUDER_0_MOTOR_DIR_PIN, EXTRUDER_0_MOTOR_SPEED_PIN , EXTRUDER_0_HEATER_PIN,
              EXTRUDER_0_FAN_PIN,  EXTRUDER_0_TEMPERATURE_PIN, EXTRUDER_0_VALVE_DIR_PIN,
              EXTRUDER_0_VALVE_ENABLE_PIN, EXTRUDER_0_STEP_ENABLE_PIN, EXTRUDER_0_LOW_HEAT, EXTRUDER_0_HIGH_HEAT);     
            
#else

#if EXTRUDER_COUNT == 2    
static extruder ex1(2);            
#endif

static extruder ex0(1);

intercom talker;

#endif

// Each entry in the buffer is an instance of cartesian_dda.

cartesian_dda* cdda[BUFFER_SIZE];

static cartesian_dda cdda0;
static cartesian_dda cdda1;
static cartesian_dda cdda2;
static cartesian_dda cdda3;

volatile byte head;
volatile byte tail;

// Where the machine is from the point of view of the command stream

FloatPoint where_i_am;

// Make sure each DDA knows which extruder to use

//inline void setExtruder()
//{
//   for(byte i = 0; i < BUFFER_SIZE; i++)
//    cdda[i]->set_extruder(ex[extruder_in_use]);
//}


// Our interrupt function

ISR(TIMER1_COMPA_vect) 
{
  disableTimerInterrupt();
  
  if(cdda[tail]->active())
    cdda[tail]->dda_step();
  else
    dQMove();
    
  enableTimerInterrupt();
}

void setup()
{
  disableTimerInterrupt();
  setupTimerInterrupt();
  debugstring[0] = 0;
  
  ex[0] = &ex0;
#if EXTRUDER_COUNT == 2  
  ex[1] = &ex1;
#endif  
  extruder_in_use = 0; 
  
  head = 0;
  tail = 0;
  
  cdda[0] = &cdda0;
  cdda[1] = &cdda1;  
  cdda[2] = &cdda2;  
  cdda[3] = &cdda3;
  
  //setExtruder();
  
  init_process_string();
  
  where_i_am.x = 0.0;
  where_i_am.y = 0.0;
  where_i_am.z = 0.0;
  where_i_am.e = 0.0;
  where_i_am.f = SLOW_XY_FEEDRATE;
  
  Serial.begin(19200);
  Serial.println("start");
  
  setTimer(DEFAULT_TICK);
  enableTimerInterrupt();

}

bool idling = true;
bool waitingForIdle = false;
signed long idleTimeout = 0;
void loop()
{
	manage_all_extruders();
        if (get_and_do_command())
        {
                if (idling && POWER_SUPPLY_PIN > 0)
                {
                        pinMode(POWER_SUPPLY_PIN, OUTPUT);
                        digitalWrite(POWER_SUPPLY_PIN, 0);  // Set the pin to GND to turn on power supply
                        idling = false;
                        waitingForIdle = true;
                }
        }
        else if (  (!idling) && (qEmpty()) && (ex[extruder_in_use]->get_target_temperature() < 1)  )
        {
                if (waitingForIdle)
                {
                        idleTimeout = (signed long)millis() + 10000;
                        waitingForIdle = false;
                }
                else
                {
                        if (idleTimeout - (signed long)millis() < 0)
                        {
                                pinMode(POWER_SUPPLY_PIN, INPUT);    // Set the pin to floating to turn off power supply
                                idling = true;
                        }
                }
        }
}

//******************************************************************************************

// The move buffer

inline bool qFull()
{
  if(tail == 0)
    return head == (BUFFER_SIZE - 1);
  else
    return head == (tail - 1);
}

inline bool qEmpty()
{
   return tail == head && !cdda[tail]->active();
}

inline void qMove(const FloatPoint& p)
{
  while(qFull()) delay(WAITING_DELAY);
  byte h = head; 
  h++;
  if(h >= BUFFER_SIZE)
    h = 0;
  cdda[h]->set_target(p);
  head = h;
}

inline void dQMove()
{
  if(qEmpty())
    return;
  byte t = tail;  
  t++;
  if(t >= BUFFER_SIZE)
    t = 0;
  cdda[t]->dda_start();
  tail = t; 
}

inline void setUnits(bool u)
{
   for(byte i = 0; i < BUFFER_SIZE; i++)
     cdda[i]->set_units(u); 
}


inline void setPosition(const FloatPoint& p)
{
  where_i_am = p;  
}


//******************************************************************************************

// Interrupt functions

void setupTimerInterrupt()
{
	//clear the registers
	TCCR1A = 0;
	TCCR1B = 0;
	TCCR1C = 0;
	TIMSK1 = 0;
	
	//waveform generation = 0100 = CTC
	TCCR1B &= ~(1<<WGM13);
	TCCR1B |=  (1<<WGM12);
	TCCR1A &= ~(1<<WGM11); 
	TCCR1A &= ~(1<<WGM10);

	//output mode = 00 (disconnected)
	TCCR1A &= ~(1<<COM1A1); 
	TCCR1A &= ~(1<<COM1A0);
	TCCR1A &= ~(1<<COM1B1); 
	TCCR1A &= ~(1<<COM1B0);

	//start off with a slow frequency.
	setTimerResolution(4);
	setTimerCeiling(65535);
}

void setTimerResolution(byte r)
{
	//here's how you figure out the tick size:
	// 1000000 / ((16000000 / prescaler))
	// 1000000 = microseconds in 1 second
	// 16000000 = cycles in 1 second
	// prescaler = your prescaler

	// no prescaler == 0.0625 usec tick
	if (r == 0)
	{
		// 001 = clk/1
		TCCR1B &= ~(1<<CS12);
		TCCR1B &= ~(1<<CS11);
		TCCR1B |=  (1<<CS10);
	}	
	// prescale of /8 == 0.5 usec tick
	else if (r == 1)
	{
		// 010 = clk/8
		TCCR1B &= ~(1<<CS12);
		TCCR1B |=  (1<<CS11);
		TCCR1B &= ~(1<<CS10);
	}
	// prescale of /64 == 4 usec tick
	else if (r == 2)
	{
		// 011 = clk/64
		TCCR1B &= ~(1<<CS12);
		TCCR1B |=  (1<<CS11);
		TCCR1B |=  (1<<CS10);
	}
	// prescale of /256 == 16 usec tick
	else if (r == 3)
	{
		// 100 = clk/256
		TCCR1B |=  (1<<CS12);
		TCCR1B &= ~(1<<CS11);
		TCCR1B &= ~(1<<CS10);
	}
	// prescale of /1024 == 64 usec tick
	else
	{
		// 101 = clk/1024
		TCCR1B |=  (1<<CS12);
		TCCR1B &= ~(1<<CS11);
		TCCR1B |=  (1<<CS10);
	}
}

unsigned int getTimerCeiling(const long& delay)
{
	// our slowest speed at our highest resolution ( (2^16-1) * 0.0625 usecs = 4095 usecs)
	if (delay <= 65535L)
		return (delay & 0xffff);
	// our slowest speed at our next highest resolution ( (2^16-1) * 0.5 usecs = 32767 usecs)
	else if (delay <= 524280L)
		return ((delay / 8) & 0xffff);
	// our slowest speed at our medium resolution ( (2^16-1) * 4 usecs = 262140 usecs)
	else if (delay <= 4194240L)
		return ((delay / 64) & 0xffff);
	// our slowest speed at our medium-low resolution ( (2^16-1) * 16 usecs = 1048560 usecs)
	else if (delay <= 16776960L)
		return ((delay / 256) & 0xffff);
	// our slowest speed at our lowest resolution ((2^16-1) * 64 usecs = 4194240 usecs)
	else if (delay <= 67107840L)
		return ((delay / 1024) & 0xffff);
	//its really slow... hopefully we can just get by with super slow.
	else
		return 65535;
}

byte getTimerResolution(const long& delay)
{
	// these also represent frequency: 1000000 / delay / 2 = frequency in hz.
	
	// our slowest speed at our highest resolution ( (2^16-1) * 0.0625 usecs = 4095 usecs (4 millisecond max))
	// range: 8Mhz max - 122hz min
	if (delay <= 65535L)
		return 0;
	// our slowest speed at our next highest resolution ( (2^16-1) * 0.5 usecs = 32767 usecs (32 millisecond max))
	// range:1Mhz max - 15.26hz min
	else if (delay <= 524280L)
		return 1;
	// our slowest speed at our medium resolution ( (2^16-1) * 4 usecs = 262140 usecs (0.26 seconds max))
	// range: 125Khz max - 1.9hz min
	else if (delay <= 4194240L)
		return 2;
	// our slowest speed at our medium-low resolution ( (2^16-1) * 16 usecs = 1048560 usecs (1.04 seconds max))
	// range: 31.25Khz max - 0.475hz min
	else if (delay <= 16776960L)
		return 3;
	// our slowest speed at our lowest resolution ((2^16-1) * 64 usecs = 4194240 usecs (4.19 seconds max))
	// range: 7.812Khz max - 0.119hz min
	else if (delay <= 67107840L)
		return 4;
	//its really slow... hopefully we can just get by with super slow.
	else
		return 4;
}


// Depending on how much work the interrupt function has to do, this is
// pretty accurate between 10 us and 0.1 s.  At fast speeds, the time
// taken in the interrupt function becomes significant, of course.

// Note - it is up to the user to call enableTimerInterrupt() after a call
// to this function.

inline void setTimer(long delay)
{
	// delay is the delay between steps in microsecond ticks.
	//
	// we break it into 5 different resolutions based on the delay. 
	// then we set the resolution based on the size of the delay.
	// we also then calculate the timer ceiling required. (ie what the counter counts to)
	// the result is the timer counts up to the appropriate time and then fires an interrupt.

        // Actual ticks are 0.0625 us, so multiply delay by 16
        
        delay <<= 4;
        
	setTimerCeiling(getTimerCeiling(delay));
	setTimerResolution(getTimerResolution(delay));
}

#include <stdio.h>
#include "parameters.h"
#include "pins.h"
#include "extruder.h"
#include "vectors.h"
#include "cartesian_dda.h"


// Initialise X, Y and Z.  The extruder is initialized
// separately.

cartesian_dda::cartesian_dda()
{
        live = false;
        nullmove = false;
        
  // Default is going forward
  
        x_direction = 1;
        y_direction = 1;
        z_direction = 1;
        e_direction = 1;
        f_direction = 1;
        
  // Default to the origin and not going anywhere
  
	target_position.x = 0.0;
	target_position.y = 0.0;
	target_position.z = 0.0;
	target_position.e = 0.0;
        target_position.f = SLOW_XY_FEEDRATE;

  // Set up the pin directions
  
	pinMode(X_STEP_PIN, OUTPUT);
	pinMode(X_DIR_PIN, OUTPUT);

	pinMode(Y_STEP_PIN, OUTPUT);
	pinMode(Y_DIR_PIN, OUTPUT);

	pinMode(Z_STEP_PIN, OUTPUT);
	pinMode(Z_DIR_PIN, OUTPUT);

#if MOTHERBOARD > 0
	pinMode(X_ENABLE_PIN, OUTPUT);
	pinMode(Y_ENABLE_PIN, OUTPUT);
	pinMode(Z_ENABLE_PIN, OUTPUT);
#endif

  //turn the motors off at the start.

	disable_steppers();

#if ENDSTOPS_MIN_ENABLED == 1
	pinMode(X_MIN_PIN, INPUT);
	pinMode(Y_MIN_PIN, INPUT);
	pinMode(Z_MIN_PIN, INPUT);
#endif

#if ENDSTOPS_MAX_ENABLED == 1
	pinMode(X_MAX_PIN, INPUT);
	pinMode(Y_MAX_PIN, INPUT);
	pinMode(Z_MAX_PIN, INPUT);
#endif
	
        // Default units are mm
        
        set_units(true);
}

// Switch between mm and inches

void cartesian_dda::set_units(bool using_mm)
{
    if(using_mm)
    {
      units.x = X_STEPS_PER_MM;
      units.y = Y_STEPS_PER_MM;
      units.z = Z_STEPS_PER_MM;
      units.e = E_STEPS_PER_MM;
      units.f = 1.0;
    } else
    {
      units.x = X_STEPS_PER_INCH;
      units.y = Y_STEPS_PER_INCH;
      units.z = Z_STEPS_PER_INCH;
      units.e = E_STEPS_PER_INCH;
      units.f = 1.0;
    }
}


void cartesian_dda::set_target(const FloatPoint& p)
{
        target_position = p;
        nullmove = false;
        
	//figure our deltas.

        delta_position = fabsv(target_position - where_i_am);
        
        // The feedrate values refer to distance in (X, Y, Z) space, so ignore e and f
        // values unless they're the only thing there.

        FloatPoint squares = delta_position*delta_position;
        distance = squares.x + squares.y + squares.z;
        // If we are 0, only thing changing is e
        if(distance <= 0.0)
          distance = squares.e;
        // If we are still 0, only thing changing is f
        if(distance <= 0.0)
          distance = squares.f;
        distance = sqrt(distance);          
                                                                                   			
	//set our steps current, target, and delta

        current_steps = to_steps(units, where_i_am);
	target_steps = to_steps(units, target_position);
	delta_steps = absv(target_steps - current_steps);

	// find the dominant axis.
        // NB we ignore the f values here, as it takes no time to take a step in time :-)

        total_steps = max(delta_steps.x, delta_steps.y);
        total_steps = max(total_steps, delta_steps.z);
        total_steps = max(total_steps, delta_steps.e);
  
        // If we're not going anywhere, flag the fact
        
        if(total_steps == 0)
        {
          nullmove = true;
          where_i_am = p;
          return;
        }    

#ifndef ACCELERATION_ON
        current_steps.f = round(target_position.f);
#endif

        delta_steps.f = abs(target_steps.f - current_steps.f);
        
        // Rescale the feedrate so it doesn't take lots of steps to do
        
        t_scale = 1;
        if(delta_steps.f > total_steps)
        {
            t_scale = delta_steps.f/total_steps;
            if(t_scale >= 3)
            {
              target_steps.f = target_steps.f/t_scale;
              current_steps.f = current_steps.f/t_scale;
              delta_steps.f = abs(target_steps.f - current_steps.f);
              if(delta_steps.f > total_steps)
                total_steps =  delta_steps.f;
            } else
            {
              t_scale = 1;
              total_steps =  delta_steps.f;
            }
        } 
        	
	//what is our direction?

	x_direction = (target_position.x >= where_i_am.x);
	y_direction = (target_position.y >= where_i_am.y);
	z_direction = (target_position.z >= where_i_am.z);
	e_direction = (target_position.e >= where_i_am.e);
	f_direction = (target_position.f >= where_i_am.f);

	dda_counter.x = -total_steps/2;
	dda_counter.y = dda_counter.x;
	dda_counter.z = dda_counter.x;
        dda_counter.e = dda_counter.x;
        dda_counter.f = dda_counter.x;
  
        where_i_am = p;
        
        return;        
}



void cartesian_dda::dda_step()
{  
  if(!live)
   return;
   
  do
  {
		x_can_step = can_step(X_MIN_PIN, X_MAX_PIN, current_steps.x, target_steps.x, x_direction);
		y_can_step = can_step(Y_MIN_PIN, Y_MAX_PIN, current_steps.y, target_steps.y, y_direction);
		z_can_step = can_step(Z_MIN_PIN, Z_MAX_PIN, current_steps.z, target_steps.z, z_direction);
                e_can_step = can_step(-1, -1, current_steps.e, target_steps.e, e_direction);
                f_can_step = can_step(-1, -1, current_steps.f, target_steps.f, f_direction);
                
                real_move = false;
                
		if (x_can_step)
		{
			dda_counter.x += delta_steps.x;
			
			if (dda_counter.x > 0)
			{
				do_x_step();
                                real_move = true;
				dda_counter.x -= total_steps;
				
				if (x_direction)
					current_steps.x++;
				else
					current_steps.x--;
			}
		}

		if (y_can_step)
		{
			dda_counter.y += delta_steps.y;
			
			if (dda_counter.y > 0)
			{
				do_y_step();
                                real_move = true;
				dda_counter.y -= total_steps;

				if (y_direction)
					current_steps.y++;
				else
					current_steps.y--;
			}
		}
		
		if (z_can_step)
		{
			dda_counter.z += delta_steps.z;
			
			if (dda_counter.z > 0)
			{
				do_z_step();
                                real_move = true;
				dda_counter.z -= total_steps;
				
				if (z_direction)
					current_steps.z++;
				else
					current_steps.z--;
			}
		}

		if (e_can_step)
		{
			dda_counter.e += delta_steps.e;
			
			if (dda_counter.e > 0)
			{
				do_e_step();
                                real_move = true;
				dda_counter.e -= total_steps;
				
				if (e_direction)
					current_steps.e++;
				else
					current_steps.e--;
			}
		}
		
		if (f_can_step)
		{
			dda_counter.f += delta_steps.f;
			
			if (dda_counter.f > 0)
			{
				dda_counter.f -= total_steps;
				if (f_direction)
					current_steps.f++;
				else
					current_steps.f--;
			}
		}

				
      // wait for next step.
      // Use milli- or micro-seconds, as appropriate
      // If the only thing that changed was f keep looping
  
                if(real_move)
                {
                  if(t_scale > 1)
                    timestep = t_scale*current_steps.f;
                  else
                    timestep = current_steps.f;
                  timestep = calculate_feedrate_delay((float) timestep);
                  setTimer(timestep);
                }
  } while (!real_move && f_can_step);

  live = (x_can_step || y_can_step || z_can_step  || e_can_step || f_can_step);


// Wrap up at the end of a line

  if(!live)
  {
      disable_steppers();
      setTimer(DEFAULT_TICK);
  }    
  
}

// Run the DDA

void cartesian_dda::dda_start()
{    
  // Set up the DDA
  //sprintf(debugstring, "%d %d", x_direction, nullmove);
  
  if(nullmove)
    return;
    
  	//set our direction pins as well
#if INVERT_X_DIR == 1
	digitalWrite(X_DIR_PIN, !x_direction);
#else
	digitalWrite(X_DIR_PIN, x_direction);
#endif

#if INVERT_Y_DIR == 1
	digitalWrite(Y_DIR_PIN, !y_direction);
#else
	digitalWrite(Y_DIR_PIN, y_direction);
#endif

#if INVERT_Z_DIR == 1
	digitalWrite(Z_DIR_PIN, !z_direction);
#else
	digitalWrite(Z_DIR_PIN, z_direction);
#endif
        if(e_direction)
          ex[extruder_in_use]->set_direction(EXTRUDER_FORWARD);
        else
          ex[extruder_in_use]->set_direction(EXTRUDER_REVERSE);
  
    //turn on steppers to start moving =)
    
	enable_steppers();
        
       // extcount = 0;

        setTimer(DEFAULT_TICK);
	live = true;
}


bool cartesian_dda::can_step(byte min_pin, byte max_pin, long current, long target, byte dir)
{

  //stop us if we're on target

	if (target == current){
		return false;
        } 

#if ENDSTOPS_MIN_ENABLED == 1

  //stop us if we're home and still going
  
	if(!dir && min_pin >= 0)
        {
          if (read_switch(min_pin))
		return false;
        }
#endif

#if ENDSTOPS_MAX_ENABLED == 1

  //stop us if we're at max and still going
  
	if(dir && max_pin >= 0)
        {
 	    if (read_switch(max_pin))
 		return false;
        }
#endif

  // All OK - we can step
  
	return true;
}




void cartesian_dda::enable_steppers()
{
#if MOTHERBOARD > 0
 if(delta_steps.x)
    digitalWrite(X_ENABLE_PIN, ENABLE_ON);
  if(delta_steps.y)    
    digitalWrite(Y_ENABLE_PIN, ENABLE_ON);
  if(delta_steps.z)
    digitalWrite(Z_ENABLE_PIN, ENABLE_ON);
  if(delta_steps.e)
    ex[extruder_in_use]->enableStep();
#endif  
}



void cartesian_dda::disable_steppers()
{
#if MOTHERBOARD > 0
	//disable our steppers
	digitalWrite(X_ENABLE_PIN, !ENABLE_ON);
	digitalWrite(Y_ENABLE_PIN, !ENABLE_ON);
	digitalWrite(Z_ENABLE_PIN, !ENABLE_ON);

        // Disabling the extrude stepper causes the backpressure to
        // turn the motor the wrong way.  Leave it on.
        
        //ex[extruder_in_use]->->disableStep();       
#endif
}

/*

void cartesian_dda::delayMicrosecondsInterruptible(unsigned int us)
{

#if F_CPU >= 16000000L
    // for the 16 MHz clock on most Arduino boards

	// for a one-microsecond delay, simply return.  the overhead
	// of the function call yields a delay of approximately 1 1/8 us.
	if (--us == 0)
		return;

	// the following loop takes a quarter of a microsecond (4 cycles)
	// per iteration, so execute it four times for each microsecond of
	// delay requested.
	us <<= 2;

	// account for the time taken in the preceeding commands.
	us -= 2;
#else
    // for the 8 MHz internal clock on the ATmega168

    // for a one- or two-microsecond delay, simply return.  the overhead of
    // the function calls takes more than two microseconds.  can't just
    // subtract two, since us is unsigned; we'd overflow.
	if (--us == 0)
		return;
	if (--us == 0)
		return;

	// the following loop takes half of a microsecond (4 cycles)
	// per iteration, so execute it twice for each microsecond of
	// delay requested.
	us <<= 1;
    
    // partially compensate for the time taken by the preceeding commands.
    // we can't subtract any more than this or we'd overflow w/ small delays.
    us--;
#endif


	// busy wait
	__asm__ __volatile__ (
		"1: sbiw %0,1" "\n\t" // 2 cycles
		"brne 1b" : "=w" (us) : "0" (us) // 2 cycles
	);
}
*/

#include "parameters.h"
#include "pins.h"
#include "ThermistorTable.h"
#include "extruder.h" 

// Keep all extruders up to temperature etc.

void manage_all_extruders()
{
    for(byte i = 0; i < EXTRUDER_COUNT; i++)
       ex[i]->manage();
}

// Select a new extruder

void new_extruder(byte e)
{
  if(e < 0)
    e = 0;
  if(e >= EXTRUDER_COUNT)
    e = EXTRUDER_COUNT - 1;
  
  if(e != extruder_in_use)
  {  
    extruder_in_use = e;
    //setExtruder();
  }
}

/***************************************************************************************************************************

If we have a new motherboard (V 1.x, x >= 1), the extruder is entirely controlled via the RS485, and all  the functions to do
it are simple inlines in extruder.h

Otherwise, we have to do the work ourselves...
*/

#if USE_EXTRUDER_CONTROLLER == false
extruder::extruder(byte md_pin, byte ms_pin, byte h_pin, byte f_pin, byte t_pin, byte vd_pin, byte ve_pin, signed int se_pin, byte low_heat, byte high_heat)
{
         motor_dir_pin = md_pin;
         motor_speed_pin = ms_pin;
         heater_pin = h_pin;
         fan_pin = f_pin;
         temp_pin = t_pin;
         valve_dir_pin = vd_pin;
         valve_en_pin = ve_pin;
         step_en_pin = se_pin;
         heater_low = low_heat;
         heater_high = high_heat;
         
	//setup our pins
	pinMode(motor_dir_pin, OUTPUT);
	pinMode(motor_speed_pin, OUTPUT);
	pinMode(heater_pin, OUTPUT);

	pinMode(temp_pin, INPUT);
	pinMode(valve_dir_pin, OUTPUT); 
        pinMode(valve_en_pin, OUTPUT);

	//initialize values
	digitalWrite(motor_dir_pin, EXTRUDER_FORWARD);
	
	analogWrite(heater_pin, 0);
	analogWrite(motor_speed_pin, 0);
	digitalWrite(valve_dir_pin, false);
	digitalWrite(valve_en_pin, 0);

// The step enable pin and the fan pin are the same...
// We can have one, or the other, but not both

        if(step_en_pin >= 0)
        {
          pinMode(step_en_pin, OUTPUT);
	  disableStep();
        } else
        {
	  pinMode(fan_pin, OUTPUT);
          analogWrite(fan_pin, 0);
        }

        //these our the default values for the extruder.
        e_speed = 0;
        target_celsius = 0;
        max_celsius = 0;
        heater_current = 0;
        valve_open = false;
        
//this is for doing encoder based extruder control
//        rpm = 0;
//        e_delay = 0;
//        error = 0;
//        last_extruder_error = 0;
//        error_delta = 0;
        e_direction = EXTRUDER_FORWARD;
        
        //default to cool
        set_target_temperature(target_celsius);
}


byte extruder::wait_till_hot()
{  
  count = 0;
  oldT = get_temperature();
  while (get_temperature() < target_celsius - HALF_DEAD_ZONE)
  {
	manage_all_extruders();
        count++;
        if(count > 20)
        {
            newT = get_temperature();
            if(newT > oldT)
               oldT = newT;
            else
                return 1;
            count = 0;
        }
	delay(1000);
  }
  return 0;
}

/*
byte extruder::wait_till_cool()
{  
  count = 0;
  oldT = get_temperature();
  while (get_temperature() > target_celsius + HALF_DEAD_ZONE)
  {
	manage_all_extruders();
        count++;
        if(count > 20)
        {
            newT = get_temperature();
            if(newT < oldT)
               oldT = newT;
            else
                return 1;
            count = 0;
        }
	delay(1000);
  }
  return 0;
}
*/



void extruder::valve_set(bool open, int dTime)
{
        wait_for_temperature();
	valve_open = open;
	digitalWrite(valve_dir_pin, open);
        digitalWrite(valve_en_pin, 1);
        delay(dTime);
        digitalWrite(valve_en_pin, 0);
}


void extruder::set_target_temperature(int temp)
{
	target_celsius = temp;
	max_celsius = (temp*11)/10;

        // If we've turned the heat off, we might as well disable the extrude stepper
       // if(target_celsius < 1)
        //  disableStep(); 
}

int extruder::get_target_temperature()
{
        return target_celsius;
}

/**
*  Samples the temperature and converts it to degrees celsius.
*  Returns degrees celsius.
*/
int extruder::get_temperature()
{
#ifdef USE_THERMISTOR
	int raw = sample_temperature();

	int celsius = 0;
	byte i;

	for (i=1; i<NUMTEMPS; i++)
	{
		if (temptable[i][0] > raw)
		{
			celsius  = temptable[i-1][1] + 
				(raw - temptable[i-1][0]) * 
				(temptable[i][1] - temptable[i-1][1]) /
				(temptable[i][0] - temptable[i-1][0]);

			break;
		}
	}

        // Overflow: Set to last value in the table
        if (i == NUMTEMPS) celsius = temptable[i-1][1];

        // Clamp
        if (celsius > 511) celsius = 511;
        else if (celsius < 0) celsius = 0; 
  
	return celsius;
#else
  return ( 5.0 * sample_temperature() * 100.0) / 1024.0;
#endif
}



/*
* This function gives us an averaged sample of the analog temperature pin.
*/
int extruder::sample_temperature()
{
	int raw = 0;
	
	//read in a certain number of samples
	for (byte i=0; i<TEMPERATURE_SAMPLES; i++)
		raw += analogRead(temp_pin);
		
	//average the samples
	raw = raw/TEMPERATURE_SAMPLES;

	//send it back.
	return raw;
}

/*!
  Manages extruder functions to keep temps, speeds etc
  at the set levels.  Should be called only by manage_all_extruders(),
  which should be called in all non-trivial loops.
  o If temp is too low, don't start the motor
  o Adjust the heater power to keep the temperature at the target
 */
void extruder::manage()
{
	//make sure we know what our temp is.
	int current_celsius = get_temperature();
        byte newheat = 0;
  
        //put the heater into high mode if we're not at our target.
        if (current_celsius < target_celsius)
                newheat = heater_high;
        //put the heater on low if we're at our target.
        else if (current_celsius < max_celsius)
                newheat = heater_low;
        
        // Only update heat if it changed
        if (heater_current != newheat) {
                heater_current = newheat;
                analogWrite(heater_pin, heater_current);
        }
}



void extruder::set_speed(float sp)
{
  // DC motor?
    if(step_en_pin < 0)
    {
      e_speed = (byte)sp;
      if(e_speed > 0)
          wait_for_temperature();
      analogWrite(motor_speed_pin, e_speed);
      return;
    }
      
    // No - stepper
  disableTimerInterrupt();
  
  if(sp <= 1.0e-4)
  {
    disableStep();
    e_speed = 0; // Just use this as a flag
    return;
  } else
  {
    wait_for_temperature();
    enableStep();
    e_speed = 1;
  }
    
  extrude_step_count = 0;
  
  float milliseconds_per_step = 60000.0/(E_STEPS_PER_MM*sp);
  long thousand_ticks_per_step = 4*(long)(milliseconds_per_step);
  setupTimerInterrupt();
  setTimer(thousand_ticks_per_step);
  enableTimerInterrupt();
}

#if 0
void extruder::interrupt()
{
    if(!e_speed)
      return;
    extrude_step_count++;
    if(extrude_step_count > 1000)
    {
      step();
      extrude_step_count = 0;
    }
}
#endif
#endif
/*
 * Class to handle internal communications in the machine via RS485
 *
 * Adrian Bowyer 3 July 2009
 *
 */
  
#if USE_EXTRUDER_CONTROLLER == true

#include "intercom.h"

intercom::intercom()
{
  pinMode(RX_ENABLE_PIN, OUTPUT);
  pinMode(TX_ENABLE_PIN, OUTPUT);
  digitalWrite(RX_ENABLE_PIN, 0); //always listen.
  Serial1.begin(38400);  
}

void intercom::sendPacket(byte address, char* string)
{
  digitalWrite(TX_ENABLE_PIN, 1);
  Serial1.print(address, HEX);
  Serial1.print(MASTER_ADDRESS);
  Serial1.println(string);
  digitalWrite(TX_ENABLE_PIN, 0);
  getPacket(myBuffer, IC_BUFFER);
  if(myBuffer[0] != MASTER_ADDRESS[0] ||  myBuffer[1] != MASTER_ADDRESS[1])
  {
     // Horrible error - what to do?
  } 
}

void intercom::sendPacketWithReply(byte address, char* string, char* reply)
{
  
}

void intercom::getPacket(char* string, int len)
{
  int i = -1;
  ok = true;
  do
  {
    while(!Serial1.available()) delay(1);
    i++;
    
    // Stop runaway buffer overflow
    
    if(i >= len)
      ok = false;
    else  
      string[i] = Serial1.read();
  } while(string[i] != '\n' && ok);
  string[i] = 0;
}

#endif

#include "parameters.h"
#include "pins.h"
#include "extruder.h"
#include "vectors.h"
#include "cartesian_dda.h"

/* bit-flags for commands and parameters */
#define GCODE_G	(1<<0)
#define GCODE_M	(1<<1)
#define GCODE_P	(1<<2)
#define GCODE_X	(1<<3)
#define GCODE_Y	(1<<4)
#define GCODE_Z	(1<<5)
#define GCODE_I	(1<<6)
#define GCODE_J	(1<<7)
#define GCODE_K	(1<<8)
#define GCODE_F	(1<<9)
#define GCODE_S	(1<<10)
#define GCODE_Q	(1<<11)
#define GCODE_R	(1<<12)
#define GCODE_E	(1<<13)
#define GCODE_T	(1<<14)


#define PARSE_INT(ch, str, len, val, seen, flag) \
	case ch: \
		len = scan_int(str, &val, &seen, flag); \
		break;

#define PARSE_FLOAT(ch, str, len, val, seen, flag) \
	case ch: \
		len = scan_float(str, &val, &seen, flag); \
		break;

/* gcode line parse results */
struct GcodeParser
{
    unsigned int seen;
    int G;
    int M;
    int T;
    float P;
    float X;
    float Y;
    float Z;
    float E;
    float I;
    float J;
    float F;
    float S;
    float R;
    float Q;
};


//our command string
char cmdbuffer[COMMAND_SIZE];
char c = '?';
byte serial_count = 0;
boolean comment = false;
FloatPoint fp;
FloatPoint sp;
        
// The following three inline functions are used for things like return to 0

inline void specialMoveX(const float& x, const float& feed)
{
  sp = where_i_am;
  sp.x = x;
  sp.f = feed;
  qMove(sp);
}

inline void specialMoveY(const float& y, const float& feed)
{
  sp = where_i_am;
  sp.y = y;
  sp.f = feed;
  qMove(sp);
}

inline void specialMoveZ(const float& z, const float& feed)
{
  sp = where_i_am;
  sp.z = z; 
  sp.f = feed;
  qMove(sp);
}

//our feedrate variables.
//float feedrate = SLOW_XY_FEEDRATE;

/* keep track of the last G code - this is the command mode to use
 * if there is no command in the current string 
 */
int last_gcode_g = -1;

boolean abs_mode = true; //0 = incremental; 1 = absolute

float extruder_speed = 0;

int scan_int(char *str, int *valp);
int scan_float(char *str, float *valp);

GcodeParser gc;	/* string parse result */


//init our string processing
inline void init_process_string()
{
	serial_count = 0;
        comment = false;
}

// Get a command and process it

bool get_and_do_command()
{
	//read in characters if we got them.
	if (Serial.available())
	{
		c = Serial.read();
                if(c == '\r')
                  c = '\n';
                // Throw away control chars except \n
                if(c >= ' ' || c == '\n')
                {

		  //newlines are ends of commands.
		  if (c != '\n')
		  {
			// Start of comment - ignore any bytes received from now on
			if (c == ';')
				comment = true;
				
			// If we're not in comment mode, add it to our array.
			if (!comment)
				cmdbuffer[serial_count++] = c;
		  }

                }
	}

        // Data runaway?
        if(serial_count >= COMMAND_SIZE)
          init_process_string();

	//if we've got a real command, do it
	if (serial_count && c == '\n')
	{
                // Terminate string
                cmdbuffer[serial_count] = 0;
                
		//process our command!
		bool busy = process_string(cmdbuffer, serial_count);

		//clear command.
		init_process_string();

                // Say we're ready for the next one
                
                if(debugstring[0] != 0)
                {
                  Serial.print("ok ");
                  Serial.println(debugstring);
                  debugstring[0] = 0;
                } else
                  Serial.println("ok");
                  
    return busy;
	}
	else
	{
	  return false;
	}
}



int parse_string(struct GcodeParser * gc, char instruction[ ], int size)
{
	int ind;
	int len;	/* length of parameter argument */

	gc->seen = 0;

	len=0;
	/* scan the string for commands and parameters, recording the arguments for each,
	 * and setting the seen flag for each that is seen
	 */
	for (ind=0; ind<size; ind += (1+len))
	{
		len = 0;
		switch (instruction[ind])
		{
			  PARSE_INT('G', &instruction[ind+1], len, gc->G, gc->seen, GCODE_G);
			  PARSE_INT('M', &instruction[ind+1], len, gc->M, gc->seen, GCODE_M);
			  PARSE_INT('T', &instruction[ind+1], len, gc->T, gc->seen, GCODE_T);
			PARSE_FLOAT('S', &instruction[ind+1], len, gc->S, gc->seen, GCODE_S);
			PARSE_FLOAT('P', &instruction[ind+1], len, gc->P, gc->seen, GCODE_P);
			PARSE_FLOAT('X', &instruction[ind+1], len, gc->X, gc->seen, GCODE_X);
			PARSE_FLOAT('Y', &instruction[ind+1], len, gc->Y, gc->seen, GCODE_Y);
			PARSE_FLOAT('Z', &instruction[ind+1], len, gc->Z, gc->seen, GCODE_Z);
			PARSE_FLOAT('I', &instruction[ind+1], len, gc->I, gc->seen, GCODE_I);
			PARSE_FLOAT('J', &instruction[ind+1], len, gc->J, gc->seen, GCODE_J);
			PARSE_FLOAT('F', &instruction[ind+1], len, gc->F, gc->seen, GCODE_F);
			PARSE_FLOAT('R', &instruction[ind+1], len, gc->R, gc->seen, GCODE_R);
			PARSE_FLOAT('Q', &instruction[ind+1], len, gc->Q, gc->seen, GCODE_Q);
			PARSE_FLOAT('E', &instruction[ind+1], len, gc->E, gc->seen, GCODE_E);
                        default:
			  break;
		}
	}
}


//Read the string and execute instructions
bool process_string(char instruction[], int size)
{
	//the character / means delete block... used for comments and stuff.
	if (instruction[0] == '/')	
		return false;

        float fr;
        
	fp.x = 0.0;
	fp.y = 0.0;
	fp.z = 0.0;
        fp.e = 0.0;
        fp.f = 0.0;

	//get all our parameters!
	parse_string(&gc, instruction, size);
	/* if no command was seen, but parameters were, then use the last G code as 
	 * the current command
	 */
	if ((!(gc.seen & (GCODE_G | GCODE_M | GCODE_T))) && 
	    ((gc.seen != 0) &&
		(last_gcode_g >= 0))
	)
	{
		/* yes - so use the previous command with the new parameters */
		gc.G = last_gcode_g;
		gc.seen |= GCODE_G;
	}
	//did we get a gcode?
	if (gc.seen & GCODE_G)
	{
		last_gcode_g = gc.G;	/* remember this for future instructions */
		fp = where_i_am;
		if (abs_mode)
		{
			if (gc.seen & GCODE_X)
				fp.x = gc.X;
			if (gc.seen & GCODE_Y)
				fp.y = gc.Y;
			if (gc.seen & GCODE_Z)
				fp.z = gc.Z;
			if (gc.seen & GCODE_E)
				fp.e = gc.E;
		}
		else
		{
			if (gc.seen & GCODE_X)
				fp.x += gc.X;
			if (gc.seen & GCODE_Y)
				fp.y += gc.Y;
			if (gc.seen & GCODE_Z)
				fp.z += gc.Z;
			if (gc.seen & GCODE_E)
				fp.e += gc.E;
		}

		// Get feedrate if supplied - feedrates are always absolute???
		if ( gc.seen & GCODE_F )
			fp.f = gc.F;
               
                // Process the buffered move commands first
                // If we get one, return immediately

		switch (gc.G)
                {
			//Rapid move
			case 0:
                                fr = fp.f;
                                fp.f = FAST_XY_FEEDRATE;
                                qMove(fp);
                                fp.f = fr;
                                return true;
                                
                        // Controlled move
			case 1:                          
#if USE_EXTRUDER_CONTROLLER == false
                                ex[extruder_in_use]->manage();  // jglauche: Forces temperature management to be called before a move. Doesn't seem to be called enough times otherwise. It's maybe fixing the symptoms, but works for me. 
#endif
                                qMove(fp);
                                return true;
                                
                        //go home.
			case 28:
                                where_i_am.f = SLOW_XY_FEEDRATE;
                                specialMoveX(where_i_am.x - 5, FAST_XY_FEEDRATE);
                                specialMoveX(where_i_am.x - 250, FAST_XY_FEEDRATE);
                                where_i_am.x = 0;
                                where_i_am.f = SLOW_XY_FEEDRATE;
                                specialMoveX(where_i_am.x + 1, SLOW_XY_FEEDRATE);
                                specialMoveX(where_i_am.x - 10, SLOW_XY_FEEDRATE);                                
                                where_i_am.x = 0;
                                
                                specialMoveY(where_i_am.y - 5, FAST_XY_FEEDRATE);
                                specialMoveY(where_i_am.y - 250, FAST_XY_FEEDRATE);
                                where_i_am.y = 0;
                                where_i_am.f = SLOW_XY_FEEDRATE;
                                specialMoveY(where_i_am.y + 1, SLOW_XY_FEEDRATE);
                                specialMoveY(where_i_am.y - 10, SLOW_XY_FEEDRATE);                                
                                where_i_am.y = 0; 
 
                                where_i_am.f = SLOW_Z_FEEDRATE;
                                specialMoveZ(where_i_am.z - 0.5, FAST_Z_FEEDRATE);
                                specialMoveZ(where_i_am.z - 250, FAST_Z_FEEDRATE);
                                where_i_am.z = 0;
                                where_i_am.f = SLOW_Z_FEEDRATE;
                                specialMoveZ(where_i_am.z + 1, SLOW_Z_FEEDRATE);
                                specialMoveZ(where_i_am.z - 2, SLOW_Z_FEEDRATE);                                
                                where_i_am.z = 0;
                                where_i_am.f = SLOW_XY_FEEDRATE;     // Most sensible feedrate to leave it in                    

				return true;


                  default:
                                break;
                }
                
		// Non-buffered G commands
                // Wait till the buffer q is empty first
                    
                  while(!qEmpty()) delay(WAITING_DELAY);
		  switch (gc.G)
		  {

  			 //Dwell
			case 4:
				delay((int)(gc.P + 0.5));  
				break;

			//Inches for Units
			case 20:
                                setUnits(false);
				break;

			//mm for Units
			case 21:
                                setUnits(true);
				break;

			//Absolute Positioning
			case 90: 
				abs_mode = true;
				break;

			//Incremental Positioning
			case 91: 
				abs_mode = false;
				break;

			//Set position as fp
			case 92: 
                                setPosition(fp);
				break;

			default:
				Serial.print("huh? G");
				Serial.println(gc.G, DEC);
		  }
	}



        
	//find us an m code.
	if (gc.seen & GCODE_M)
	{
            // Wait till the q is empty first
            while(!qEmpty()) delay(WAITING_DELAY);
            
		switch (gc.M)
		{
			//TODO: this is a bug because search_string returns 0.  gotta fix that.
			case 0:
				break;
				/*
				 case 0:
				 //todo: stop program
				 break;

				 case 1:
				 //todo: optional stop
				 break;

				 case 2:
				 //todo: program end
				 break;
				 */

// jglauche: re-activate M101-M103 and M108 codes to support both 3d & 5d code
//           no idea why mccoin commended them out

			//turn extruder on, forward
			case 101:
				ex[extruder_in_use]->set_direction(1);
				ex[extruder_in_use]->set_speed(extruder_speed);
				break;

			//turn extruder on, reverse
			case 102:
				ex[extruder_in_use]->set_direction(0);
				ex[extruder_in_use]->set_speed(extruder_speed);
				break;

			//turn extruder off
			case 103:
				ex[extruder_in_use]->set_speed(0);
				break;

			//custom code for temperature control
			case 104:
				if (gc.seen & GCODE_S)
				{
					ex[extruder_in_use]->set_target_temperature((int)gc.S);
				}
				break;

			//custom code for temperature reading
			case 105:
				Serial.print("T:");
				Serial.println(ex[extruder_in_use]->get_temperature());
				return false;

			//turn fan on
			case 106:
				ex[extruder_in_use]->set_cooler(255);
				break;

			//turn fan off
			case 107:
				ex[extruder_in_use]->set_cooler(0);
				break;

			//set max extruder speed, 0-255 PWM
			case 108:
				if (gc.seen & GCODE_S)
					extruder_speed = gc.S;
				break;

 			case 109: // Base plate heater on/off
 				if (gc.seen & GCODE_S)
 				  digitalWrite(BASE_HEATER_PIN, gc.S != 0);
 				break;

// The valve (real, or virtual...) is now the way to control any extruder (such as
// a pressurised paste extruder) that cannot move using E codes.

                        // Open the valve
                        case 126:
                                ex[extruder_in_use]->valve_set(true, (int)(gc.P + 0.5));
                                break;
                                
                        // Close the valve
                        case 127:
                                ex[extruder_in_use]->valve_set(false, (int)(gc.P + 0.5));
                                break;
                                                                

			default:
				Serial.print("[FIRMWARE WARNING] invalid M-Code received: M");
				Serial.println(gc.M, DEC);
		}

                

	}

// Tool (i.e. extruder) change?
                
        if (gc.seen & GCODE_T)
        {
            while(!qEmpty()) delay(WAITING_DELAY);
            new_extruder(gc.T);
            return false;
        }
        
        return true;
}

int scan_float(char *str, float *valp, unsigned int *seen, unsigned int flag)
{
	float res;
	int len;
	char *end;
     
	res = (float)strtod(str, &end);
      
	len = end - str;

	if (len > 0)
	{
		*valp = res;
		*seen |= flag;
	}
	else
		*valp = 0;
          
	return len;	/* length of number */
}

int scan_int(char *str, int *valp, unsigned int *seen, unsigned int flag)
{
	int res;
	int len;
	char *end;

	res = (int)strtol(str, &end, 10);
	len = end - str;

	if (len > 0)
	{
		*valp = res;
		*seen |= flag;
	}
	else
		*valp = 0;
          
	return len;	/* length of number */
}




int main(void)
{
	init();

	setup();
    
	for (;;)
		loop();
        
	return 0;
}

