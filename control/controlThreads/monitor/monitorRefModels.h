/******************************************************************************/
/**
 * \file monitorRefModels.h
 * \brief This file has the reference models monitor prototypes
 *
 */
/******************************************************************************/

#ifndef _MONITORREFMODELS_H
#define _MONITORREFMODELS_H

/*robot includes*/
#include "robotStructs.h"

/******************************************************************************/

/**
 * \brief This function copy the local structure into shared
 * \param shared pointer to shared memory
 * \param local pointer to local memory
 * \return 0;
 */
extern inline int monitorRefXSet(st_robotControlShared *shared, st_robotControl *local);

/******************************************************************************/

/**
 * \brief This function copy the local structure into shared
 * \param shared pointer to shared memory
 * \param local pointer to local memory
 * \return 0;
 */
extern inline int monitorRefYSet(st_robotControlShared *shared, st_robotControl *local);

/******************************************************************************/

/**
 * \brief This function copy the shared into local structure
 * \param local pointer to local memory
 * \param shared pointer to shared memory
 * \return 0;
 */
extern inline int monitorRefXGet(st_robotControl *local, st_robotControlShared *shared);

/******************************************************************************/

/**
 * \brief This function copy the shared into local structure
 * \param local pointer to local memory
 * \param shared pointer to shared memory
 * \return 0;
 */
extern inline int monitorRefYGet(st_robotControl *local, st_robotControlShared *shared);

/******************************************************************************/

#endif


