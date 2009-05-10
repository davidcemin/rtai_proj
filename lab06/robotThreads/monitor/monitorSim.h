/******************************************************************************/
/**
 * \file monitorSim.h
 * \brief This file has the Simulation monitor prototypes, structs and defines
 *
 */
/******************************************************************************/

#ifndef _MONITORSIM_H
#define _MONITORSIM_H

/*robot includes*/
#include "libRobot.h"

/******************************************************************************/

/**
 * \brief This function copy a value into the shared
 * \param robot Pointer to st_robotMainArrays structure
 * \param shared pointer to shared memory
 * \return 0
 */
extern int monitorSimSet(st_robotMainArrays *robot, st_robotShared *shared);

/******************************************************************************/

/**
 * \brief  This function gets a copy of the shared memory
 * \param  sharedCp pointer to shared copy
 * \param  shared pointer to shared memory
 */
extern int monitorSimGet(st_robotShared *sharedCp, st_robotShared *shared);

/******************************************************************************/

#endif //! _MONITORSIM_H
