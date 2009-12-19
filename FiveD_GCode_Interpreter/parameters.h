#ifndef PARAMETERS_H
#define PARAMETERS_H

// Here are the moterboard codes; set MOTHERBOARD to the right one
// (Arduino: 0 - no longer in use)
// Sanguino or RepRap Motherboard with direct drive extruders: 1
// RepRap Motherboard with RS485 extruders: 2
// Arduino Mega: 3
// Arudino Duemilanove w/ ATMega328P: 4

#define MOTHERBOARD 4

// Set 1s where you have endstops; 0s where you don't
#define ENDSTOPS_MIN_ENABLED 1
#define ENDSTOPS_MAX_ENABLED 0

//our command string length
#define COMMAND_SIZE 128

// The size of the movement buffer

#define BUFFER_SIZE 4

// Number of microseconds between timer interrupts when no movement
// is happening

#define DEFAULT_TICK (long)1000

// What delay() value to use when waiting for things to free up in milliseconds

#define WAITING_DELAY 1

#define INCHES_TO_MM 25.4

// define the parameters of our machine.
#define X_STEPS_PER_MM   9.8482399079
#define X_STEPS_PER_INCH (X_STEPS_PER_MM*INCHES_TO_MM)
#define X_MOTOR_STEPS    400
#define INVERT_X_DIR 0

#define Y_STEPS_PER_MM   9.8482399079
#define Y_STEPS_PER_INCH (Y_STEPS_PER_MM*INCHES_TO_MM)
#define Y_MOTOR_STEPS    400
#define INVERT_Y_DIR 1

#define Z_STEPS_PER_MM   160 // 200 steps per revoloution / 1.25 mm thread pitch
#define Z_STEPS_PER_INCH (Z_STEPS_PER_MM*INCHES_TO_MM)
#define Z_MOTOR_STEPS    200
#define INVERT_Z_DIR 0

// For when we have a stepper-driven extruder
// E_STEPS_PER_MM is the number of steps needed to 
// extrude 1mm out of the nozzle.

#define E_STEPS_PER_MM   0.7525056 // drive diameter = 4.7 mm, material in:material out = 36:1, 400 steps per revolution
#define E_STEPS_PER_INCH (E_STEPS_PER_MM*INCHES_TO_MM)
#define E_MOTOR_STEPS    400

//our maximum feedrates
#define FAST_XY_FEEDRATE 4000.0
#define FAST_Z_FEEDRATE  60.0

// Data for acceleration calculations
// Comment out the next line to turn accelerations off
//#define ACCELERATION_ON
#define SLOW_XY_FEEDRATE 1000.0 // Speed from which to start accelerating
#define SLOW_Z_FEEDRATE 20

// Set to 1 if enable pins are inverting
// For RepRap stepper boards version 1.x the enable pins are *not* inverting.
// For RepRap stepper boards version 2.x and above the enable pins are inverting.
#define INVERT_ENABLE_PINS 1

#if INVERT_ENABLE_PINS == 1
#define ENABLE_ON LOW
#else
#define ENABLE_ON HIGH
#endif

// Set to one if sensor outputs inverting (ie: 1 means open, 0 means closed)
// RepRap opto endstops are *not* inverting.
#define ENDSTOPS_INVERTING 0

// Inline interrupt control functions

inline void enableTimerInterrupt() 
{
   TIMSK1 |= (1<<OCIE1A);
}
	
inline void disableTimerInterrupt() 
{
     TIMSK1 &= ~(1<<OCIE1A);
}
        
inline void setTimerCeiling(unsigned int c) 
{
    OCR1A = c;
}

inline void resetTimer()
{
  TCNT2 = 0;
}

#endif
