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

//! Simulation timers
#define INIT_TIME			0.0
#define FINAL_TIME			20.0
#define TOTAL_TIME	 ( FINAL_TIME - INIT_TIME ) 

//! Step times
#define STEPTIMECALC	0.05
#define STEPTIMESIM		0.03
#define STEPTIMEGENERATION	0.120
#define STEPTIMEREFMODELX	0.05
#define STEPTIMEREFMODELY	0.05

//! Step times in nanoseconds
#define STEPTIMECALCNANO	SEC2NANO(STEPTIMECALC)
#define STEPTIMESIMNANO		SEC2NANO(STEPTIMESIM)
#define STEPTIMEGENERNANO	SEC2NANO(STEPTIMEGENERATION)
#define STEPTIMEREFMODELXNANO	SEC2NANO(STEPTIMEREFMODELX)
#define STEPTIMEREFMODELYNANO	SEC2NANO(STEPTIMEREFMODELY)

//! Priorities
#define SIMPRIORITY		1
#define CALCPRIORITY	(SIMPRIORITY + 1)
#define REFMODXPRIORITY	(SIMPRIORITY + 2)
#define REFMODYPRIORITY	(SIMPRIORITY + 3)

//! below this value I consider that it equals to zero
#define	CALCERROR	0.00001

//! Dimensions
#define XY_DIMENSION	3
#define X_DIMENSION		3
#define Y_DIMENSION		2
#define U_DIMENSION		2

//! Distance from the robot's front
#define DIST	0.6

//! Radius variable
#define RADIUS	(0.5 * DIST)

//! Maximum quantity of data
#define MAX_DATA_VALUE	(int)(ceil( ( (double)( (TOTAL_TIME) / (STEPTIMESIM) ) ) ) + 1 )

//! Monitor defines
#define MONITOR_GET	0
#define MONITOR_SET	1

#endif //! _ROBOTDEFINES_H

