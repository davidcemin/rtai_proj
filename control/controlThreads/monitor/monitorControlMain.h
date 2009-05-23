/******************************************************************************/
/**
 * \file monitorControlMain.h
 * \brief This file has the Main monitor prototypes
 *
 */
/******************************************************************************/

#ifndef _MONITORCONTROLMAIN_H
#define _MONITORCONTROLMAIN_H

#include "robotStructs.h"

/******************************************************************************/

/**
 * \brief Monitor main control function. This function calls all others in order to be transparent 
 * to other functions.
 * \param shared Pointer to shared structure
 * \param local Pointer to local structure
 * \param type Monitor used
 * \return void
 */
extern inline void monitorControlMain(st_robotControlShared *shared, st_robotControl *local, int type);

/******************************************************************************/


#endif /*_MONITORCONTROLMAIN_H*/






