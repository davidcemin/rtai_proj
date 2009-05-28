/******************************************************************************/
/**
 * \file monitorControl.h
 * \brief This file has the Control monitor prototypes, structs and defines
 *
 */
/******************************************************************************/

#ifndef _MONITORCONTROL_H
#define _MONITORCONTROL_H

/*robot includes*/
#include "robotStructs.h"

/******************************************************************************/

/**
 * \brief Sets v into shared memory
 * \param shared Pointer to shared structure
 * \param local Pointer to local structure
 * \return 0
 */
extern inline int monitorLinSet(st_robotControlShared *shared, st_robotControl *local);

/******************************************************************************/

/**
 * \brief  Function that copies v from shared memory
 * \param  shared pointer to shared memory
 * \param  local Pointer to local memory
 * \return 0
 */
extern inline int monitorLinGet(st_robotControl *local, st_robotControlShared *shared);

/******************************************************************************/

/**
 * \brief  
 */
extern inline int monitorLinGetX(st_robotControl *local, st_robotControlShared *shared);

/******************************************************************************/
/**
 * \brief  
 */
extern inline int monitorLinSetX(st_robotControlShared *shared, st_robotControl *local);
/******************************************************************************/

/**
 * \brief  
 */
extern inline int monitorLinGetU(st_robotControl *local, st_robotControlShared *shared);

/******************************************************************************/
/**
 * \brief  
 */
extern inline int monitorLinSetU(st_robotControlShared *shared, st_robotControl *local);
/******************************************************************************/


#endif
