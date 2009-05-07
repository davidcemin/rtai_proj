/******************************************************************************/
/**
 * \file monitorSim.h
 * \brief This file has the Simulation monitor prototypes, structs and defines
 *
 */
/******************************************************************************/

#ifndef _MONITORSIM_H
#define _MONITORSIM_H

#include "libRobot.h"

/******************************************************************************/

/**
 * \brief  
 */
extern int monitorSimSet(st_robotMainArrays *robot, st_robotShared *shared);

/******************************************************************************/

/**
 * \brief  
 */
extern int monitorSimGet(st_robotMainArrays *robot, st_robotShared *shared);

/******************************************************************************/

#endif //! _MONITORSIM_H
