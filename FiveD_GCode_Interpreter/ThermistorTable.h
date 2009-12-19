#ifndef THERMISTORTABLE_H_
#define THERMISTORTABLE_H_

#if USE_EXTRUDER_CONTROLLER == false

// Uncomment the next line if you are using a thermistor; leave it if you have a thermocouple
#define USE_THERMISTOR

// How many temperature samples to take for an average.  each sample takes about 100 usecs.
#define TEMPERATURE_SAMPLES 3

// How accurately do we maintain the temperature?
#define HALF_DEAD_ZONE 5

// Thermistor lookup table for RepRap Temperature Sensor Boards (http://make.rrrf.org/ts)
// Made with createTemperatureLookup.py (http://svn.reprap.org/trunk/reprap/firmware/Arduino/utilities/createTemperatureLookup.py)
// ./createTemperatureLookup.py --r0=97100 --t0=25.4 --r1=0 --r2=4650 --beta=4200 --max-adc=1022
// r0: 97100
// t0: 25.4
// r1: 0
// r2: 4650
// beta: 4200
// max adc: 1022
#define NUMTEMPS 60
short temptable[NUMTEMPS][2] = {
   {   2,  603}, // 0.01 V    0.01 mW    108.458 C/step    79.90417 C/(step^2)
   {  19,  321}, // 0.09 V    0.10 mW    4.524 C/step    0.30244 C/(step^2)
   {  36,  271}, // 0.18 V    0.18 mW    2.031 C/step    0.06953 C/(step^2)
   {  53,  244}, // 0.26 V    0.26 mW    1.267 C/step    0.02882 C/(step^2)
   {  71,  224}, // 0.35 V    0.35 mW    0.894 C/step    0.01486 C/(step^2)
   {  88,  211}, // 0.43 V    0.42 mW    0.695 C/step    0.00916 C/(step^2)
   { 105,  200}, // 0.51 V    0.49 mW    0.568 C/step    0.00615 C/(step^2)
   { 123,  191}, // 0.60 V    0.57 mW    0.475 C/step    0.00431 C/(step^2)
   { 140,  184}, // 0.68 V    0.63 mW    0.412 C/step    0.00322 C/(step^2)
   { 157,  177}, // 0.77 V    0.70 mW    0.364 C/step    0.00248 C/(step^2)
   { 174,  171}, // 0.85 V    0.76 mW    0.326 C/step    0.00197 C/(step^2)
   { 192,  166}, // 0.94 V    0.82 mW    0.294 C/step    0.00157 C/(step^2)
   { 209,  161}, // 1.02 V    0.87 mW    0.270 C/step    0.00130 C/(step^2)
   { 226,  156}, // 1.10 V    0.92 mW    0.250 C/step    0.00108 C/(step^2)
   { 244,  152}, // 1.19 V    0.98 mW    0.232 C/step    0.00091 C/(step^2)
   { 261,  148}, // 1.27 V    1.02 mW    0.218 C/step    0.00077 C/(step^2)
   { 278,  145}, // 1.36 V    1.06 mW    0.206 C/step    0.00067 C/(step^2)
   { 295,  141}, // 1.44 V    1.10 mW    0.195 C/step    0.00058 C/(step^2)
   { 313,  138}, // 1.53 V    1.14 mW    0.185 C/step    0.00050 C/(step^2)
   { 330,  135}, // 1.61 V    1.17 mW    0.178 C/step    0.00044 C/(step^2)
   { 347,  132}, // 1.69 V    1.20 mW    0.171 C/step    0.00038 C/(step^2)
   { 365,  129}, // 1.78 V    1.23 mW    0.164 C/step    0.00033 C/(step^2)
   { 382,  126}, // 1.87 V    1.26 mW    0.159 C/step    0.00029 C/(step^2)
   { 399,  123}, // 1.95 V    1.28 mW    0.154 C/step    0.00026 C/(step^2)
   { 416,  121}, // 2.03 V    1.30 mW    0.150 C/step    0.00023 C/(step^2)
   { 434,  118}, // 2.12 V    1.31 mW    0.146 C/step    0.00020 C/(step^2)
   { 451,  116}, // 2.20 V    1.33 mW    0.143 C/step    0.00017 C/(step^2)
   { 468,  113}, // 2.29 V    1.33 mW    0.140 C/step    0.00015 C/(step^2)
   { 486,  111}, // 2.37 V    1.34 mW    0.138 C/step    0.00013 C/(step^2)
   { 503,  108}, // 2.46 V    1.34 mW    0.136 C/step    0.00011 C/(step^2)
   { 520,  106}, // 2.54 V    1.34 mW    0.134 C/step    0.00009 C/(step^2)
   { 537,  104}, // 2.62 V    1.34 mW    0.133 C/step    0.00007 C/(step^2)
   { 555,  101}, // 2.71 V    1.33 mW    0.132 C/step    0.00005 C/(step^2)
   { 572,   99}, // 2.79 V    1.33 mW    0.131 C/step    0.00003 C/(step^2)
   { 589,   97}, // 2.88 V    1.31 mW    0.131 C/step    0.00001 C/(step^2)
   { 607,   95}, // 2.96 V    1.30 mW    0.131 C/step    -0.00001 C/(step^2)
   { 624,   92}, // 3.05 V    1.28 mW    0.131 C/step    -0.00002 C/(step^2)
   { 641,   90}, // 3.13 V    1.26 mW    0.131 C/step    -0.00004 C/(step^2)
   { 658,   88}, // 3.21 V    1.23 mW    0.132 C/step    -0.00006 C/(step^2)
   { 676,   86}, // 3.30 V    1.21 mW    0.134 C/step    -0.00009 C/(step^2)
   { 693,   83}, // 3.38 V    1.18 mW    0.135 C/step    -0.00011 C/(step^2)
   { 710,   81}, // 3.47 V    1.14 mW    0.137 C/step    -0.00014 C/(step^2)
   { 728,   78}, // 3.55 V    1.10 mW    0.140 C/step    -0.00017 C/(step^2)
   { 745,   76}, // 3.64 V    1.07 mW    0.143 C/step    -0.00020 C/(step^2)
   { 762,   74}, // 3.72 V    1.02 mW    0.147 C/step    -0.00024 C/(step^2)
   { 779,   71}, // 3.80 V    0.98 mW    0.152 C/step    -0.00029 C/(step^2)
   { 797,   68}, // 3.89 V    0.93 mW    0.158 C/step    -0.00035 C/(step^2)
   { 814,   65}, // 3.97 V    0.88 mW    0.164 C/step    -0.00042 C/(step^2)
   { 831,   63}, // 4.06 V    0.82 mW    0.172 C/step    -0.00051 C/(step^2)
   { 849,   59}, // 4.15 V    0.76 mW    0.182 C/step    -0.00063 C/(step^2)
   { 866,   56}, // 4.23 V    0.70 mW    0.194 C/step    -0.00078 C/(step^2)
   { 883,   53}, // 4.31 V    0.64 mW    0.209 C/step    -0.00098 C/(step^2)
   { 900,   49}, // 4.39 V    0.57 mW    0.228 C/step    -0.00126 C/(step^2)
   { 918,   45}, // 4.48 V    0.50 mW    0.254 C/step    -0.00172 C/(step^2)
   { 935,   40}, // 4.57 V    0.43 mW    0.289 C/step    -0.00240 C/(step^2)
   { 952,   35}, // 4.65 V    0.35 mW    0.339 C/step    -0.00361 C/(step^2)
   { 970,   28}, // 4.74 V    0.27 mW    0.424 C/step    -0.00622 C/(step^2)
   { 987,   20}, // 4.82 V    0.19 mW    0.575 C/step    -0.01271 C/(step^2)
   {1004,    7}, // 4.90 V    0.10 mW    0.960 C/step    -0.04049 C/(step^2)
   {1022,  -29}  // 4.99 V    0.01 mW    7.633 C/step    -3.53721 C/(step^2)
};


#endif
#endif
