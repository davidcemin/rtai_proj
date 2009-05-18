/******************************************************************************/
/**
 * \file monitorGen.h
 * \brief This file has the Generation monitor prototypes
 *
 */
/******************************************************************************/

#ifndef _MONITORGEN_H
#define _MONITORGEN_H

/*robot includes*/
#include "libRobot.h"

/******************************************************************************/

/**
 * \brief Generation thread monitor
 * \param shared pointer to shared memory
 * \param local pointer to local memory
 * \param type Get or Set.
 * \return 0 ok, -1 error;
 */
extern int monitorGeneration(st_robotGenerationShared *shared, st_robotGeneration *local, int type);

/******************************************************************************/

#endif /*_MONITORGEN_H */

