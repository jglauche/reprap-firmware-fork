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
              EXTRUDER_1_VALVE_ENABLE_PIN, EXTRUDER_1_STEP_ENABLE_PIN);            
#endif

static extruder ex0(EXTRUDER_0_MOTOR_DIR_PIN, EXTRUDER_0_MOTOR_SPEED_PIN , EXTRUDER_0_HEATER_PIN,
            EXTRUDER_0_FAN_PIN,  EXTRUDER_0_TEMPERATURE_PIN, EXTRUDER_0_VALVE_DIR_PIN,
            EXTRUDER_0_VALVE_ENABLE_PIN, EXTRUDER_0_STEP_ENABLE_PIN);
            
            
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
                if (idling)
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

