/******************************************************************************/
/**
 * \file monitorDisp.h
 * \brief This file has the Control monitor prototypes, structs and defines
 *
 */
/******************************************************************************/

#ifndef _MONITORDISP_H
#define _MONITORDISP_H

/*robot includes*/
#include "robotStructs.h"

/******************************************************************************/

/**
 * \brief  
 */
extern inline int monitorGetAlpha(st_robotControlShared *shared, st_robotControl *local);

/******************************************************************************/

/**
 * \brief  
 */
extern inline int monitorSetAlpha(st_robotControl *local, st_robotControlShared *shared);

/******************************************************************************/

#endif
