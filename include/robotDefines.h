/******************************************************************************/
/**
* \file robotDefines.h
* \brief  Defines used in the simulation
*/
/******************************************************************************/

#ifndef _ROBOTDEFINES_H
#define _ROBOTDEFINES_H

#include <math.h>

//! Macro that transforms the value in seconds to nanoseconds
/*#define SEC2NANO(val)	(val * 1000000000.0)*/
#define SEC2NANO(val)	(val * 1e9)

//! Simulation timers
#define INIT_TIME			0.0
#define FINAL_TIME			10.0
#define TOTAL_TIME	 ( FINAL_TIME - INIT_TIME ) 

//! 1m tick
#define TICK 1000000000

//! Step times of each task (all in miliseconds)
#define TIMECTRL	0.05 
#define TIMESIM		0.03
#define TIMELIN		0.04
#define TIMEGEN		0.120
#define TIMEMODX	0.05
#define TIMEMODY	0.05

//! Dummy packet. Used to finish threads
#define DUMMYPACK	0xffff

//! Step times
#define STEPTIMECALC		0.05
#define STEPTIMESIM			0.03
#define STEPTIMELIN			0.04
#define STEPTIMEGENERATION	0.120
#define STEPTIMEREFMODELX	0.05
#define STEPTIMEREFMODELY	0.05

//! Step times in nanoseconds
#define STEPTIMECALCNANO	SEC2NANO(STEPTIMECALC)
#define STEPTIMELINNANO		SEC2NANO(STEPTIMELIN)
#define STEPTIMESIMNANO		SEC2NANO(STEPTIMESIM)
#define STEPTIMEGENERNANO	SEC2NANO(STEPTIMEGENERATION)
#define STEPTIMEREFMODELXNANO	SEC2NANO(STEPTIMEREFMODELX)
#define STEPTIMEREFMODELYNANO	SEC2NANO(STEPTIMEREFMODELY)

//! Priorities
#define SIMPRIORITY	1
#define CTRLPRIORITY	(SIMPRIORITY + 1)
#define GENPRIORITY		(SIMPRIORITY + 2)
#define REFMODXPRIORITY	(SIMPRIORITY + 3)
#define REFMODYPRIORITY	(SIMPRIORITY + 4)
#define LINPRIORITY		(SIMPRIORITY + 5)

//! Tasks names
#define GENTASK		"GEN"
#define REFMODX		"REFX"
#define REFMODY		"REFY"
#define CONTROLTSK	"CTRL"
#define LINEARTSK	"LIN"
#define SIMTSK		"SIM"

//! below this value I consider that it equals to zero
#define	CALCERROR	0.00001

//! Dimensions
#define REF_DIMENSION	2
#define X_DIMENSION		3
#define Y_DIMENSION		2
#define XY_DIMENSION	(X_DIMENSION + Y_DIMENSION)
#define U_DIMENSION		2
#define V_DIMENSION		2
#define XY_DIM_PACKET	3

//!ym vector
#define YM_DIMENSION	2
#define XM_POSITION		0
#define YM_POSITION		1

//! ref vector
#define XREF_POSITION	0
#define YREF_POSITION	1


//! ALPHA DEFINES
#define ALPHA_DIMENSION	2
#define ALPHA_1	0
#define ALPHA_2	1

//! Distance from the robot's front
#define DIST	0.6

//! Radius variable
#define RADIUS	(0.5 * DIST)

//! Maximum quantity of data
#define MAX_DATA_VALUE	(int)(ceil( ( (double)( (TOTAL_TIME) / (STEPTIMESIM) ) ) ) + 1 )

//! Macro that returns the number of elements from a vector
#define NMEMB(x)   ((sizeof(x))/(sizeof(*x)))

//! Monitor defines
#define MONITOR_GET_REFERENCE_X	0
#define MONITOR_SET_REFERENCE_X	1

#define MONITOR_GET_REFERENCE_Y 2
#define MONITOR_SET_REFERENCE_Y 3

#define MONITOR_GET_YMX			4
#define MONITOR_SET_YMX			5

#define MONITOR_GET_YMY			6
#define MONITOR_SET_YMY			7

#define MONITOR_GET_V			8
#define MONITOR_SET_V			9

#define MONITOR_GET_SIM_SHARED	10
#define MONITOR_SET_SIM_SHARED	11

#define MONITOR_GET_Y			12
#define MONITOR_SET_Y			13


#endif //! _ROBOTDEFINES_H

