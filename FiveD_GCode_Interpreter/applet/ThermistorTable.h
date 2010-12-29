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
// Table derived from table in data sheet http://www.epcos.com/inf/50/db/ntc_06/GESensors__B57560__G560.pdf
// r2: 4650
// max adc: 1022
//
short temptable[][2] = {
{	23	,	300	},
{	25	,	295	},
{	27	,	290	},
{	28	,	285	},
{	31	,	280	},
{	33	,	275	},
{	35	,	270	},
{	38	,	265	},
{	41	,	260	},
{	44	,	255	},
{	48	,	250	},
{	52	,	245	},
{	56	,	240	},
{	61	,	235	},
{	66	,	230	},
{	71	,	225	},
{	78	,	220	},
{	84	,	215	},
{	92	,	210	},
{	100	,	205	},
{	109	,	200	},
{	120	,	195	},
{	131	,	190	},
{	143	,	185	},
{	156	,	180	},
{	171	,	175	},
{	187	,	170	},
{	205	,	165	},
{	224	,	160	},
{	245	,	155	},
{	268	,	150	},
{	293	,	145	},
{	320	,	140	},
{	348	,	135	},
{	379	,	130	},
{	411	,	125	},
{	445	,	120	},
{	480	,	115	},
{	516	,	110	},
{	553	,	105	},
{	591	,	100	},
{	628	,	95	},
{	665	,	90	},
{	702	,	85	},
{	737	,	80	},
{	770	,	75	},
{	801	,	70	},
{	830	,	65	},
{	857	,	60	},
{	881	,	55	},
{	903	,	50	},
{	922	,	45	},
{	939	,	40	},
{	954	,	35	},
{	966	,	30	},
{	977	,	25	},
{	985	,	20	},
{	993	,	15	},
{	999	,	10	},
{	1004	,	5	},
{	1008	,	0	},

};
#define NUMTEMPS (sizeof(temptable) / sizeof(temptable[0]))

#endif
#endif
