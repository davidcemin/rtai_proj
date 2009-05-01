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
#define SEC2NANO(val)	(val * 1000000000.0)

//! Initial Time of the simulation
#define INIT_TIME			0.0

//! Final time of the simulation
#define FINAL_TIME			1.0

//! Total time of simulation
#define TOTAL_TIME	 ( FINAL_TIME - INIT_TIME ) 

//! 50ms of calculation time
#define STEPTIMECALC	0.05

//! Calculation time in nanoseconds
#define STEPTIMECALCNANO	SEC2NANO(STEPTIMECALC)

//! 30ms of simulation time
#define STEPTIMESIM		0.03

//! Simulation time in nanoseconds
#define STEPTIMESIMNANO		SEC2NANO(STEPTIMESIM)

//! Simulation task Priority
#define SIMPRIORITY		1

//! Calculation task priority
#define CALCPRIORITY	(SIMPRIORITY + 1)

//! below this value I consider that it equals to zero
#define	CALCERROR	0.00001

//! XY Dimension used
#define XY_DIMENSION	3

//! U Dimension used
#define U_DIMENSION		2

//! Distance from the robot's front
#define DIST	0.6

//! Maximum quantity of data
#define MAX_DATA_VALUE	(int)(ceil( ( (double)( (TOTAL_TIME) / (STEPTIMESIM) ) ) ) )

#endif //! _ROBOTDEFINES_H

