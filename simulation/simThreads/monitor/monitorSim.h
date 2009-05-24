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
#include "robotStructs.h"

/******************************************************************************/

/**
 * \brief  
 */
extern inline int monitorSimSet(st_robotSimulShared *shared, st_robotSimulPacket *packet);

/******************************************************************************/

/**
 * \brief  
 */
extern inline int monitorSimGet(st_robotSimulPacket *packet, st_robotSimulShared *shared);

/******************************************************************************/

#endif //! _MONITORSIM_H
