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
#include "robotStructs.h"

/******************************************************************************/

/**
 * \brief This function copy the local structure into shared
 * \param shared pointer to shared memory
 * \param local pointer to local memory
 * \return 0;
 */
extern inline int monitorGenSetX(st_robotControlShared *shared, st_robotControl *local);

/******************************************************************************/

/**
 * \brief  Function that get the structure from shared
 * \param  local Pointer to st_robotGeneration structure
 * \param  shared Pointer to st_robotGenerationShared structure
 * \return 0
 */
extern inline int monitorGenGetX(st_robotControl *local, st_robotControlShared *shared);

/******************************************************************************/

/**
 * \brief This function copy the local structure into shared
 * \param shared pointer to shared memory
 * \param local pointer to local memory
 * \return 0;
 */
extern inline int monitorGenSetY(st_robotControlShared *shared, st_robotControl *local);

/******************************************************************************/

/**
 * \brief  Function that get the structure from shared
 * \param  local Pointer to st_robotGeneration structure
 * \param  shared Pointer to st_robotGenerationShared structure
 * \return 0
 */
extern inline int monitorGenGetY(st_robotControl *local, st_robotControlShared *shared);

/******************************************************************************/

#endif /*_MONITORGEN_H */

