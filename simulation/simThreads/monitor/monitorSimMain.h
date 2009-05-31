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
 * \brief Initialization of robot shared memory
 * \param void
 * \return void
 */
extern void robotSimSharedInit(void);

/******************************************************************************/

/**
 * \brief  Clean of robot shared memory
 * \param  void
 * \return void
 */
extern void robotSimSharedFinish(void);

/******************************************************************************/

/**
 * \brief  Simulation monitor
 * \param  packet Pointer to local structure
 * \param  type Type used
 */
extern inline void monitorSimMain(st_robotSimulPacket *packet, int type);

/******************************************************************************/


#endif /*_MONITORCONTROLMAIN_H*/


