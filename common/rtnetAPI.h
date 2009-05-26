/******************************************************************************/
/**
* \file   rtnetAPI.h
* \brief  API between rtnet and robot functions prototypes
*/
/******************************************************************************/

#ifndef _RTNETAPI_H
#define _RTNETAPI_H

#include "robotStructs.h"

/******************************************************************************/

/**
 * \brief  Initializes rtnet packet
 * \param  rtnet Pointer to st_rtnetRobot structure
 * \return void
 */
extern inline void rtnetPacketInit(st_rtnetRobot *rtnet);

/*****************************************************************************/

/**
 * \brief  Wait for a valid task handler
 * \param  rtnet Pointer to st_rtnetRobot structure
 * \param  taskHandler Pointer to task's handler
 * \param  taskName Task's name
 * \return void
 */
extern inline void rtnetTaskWait(st_rtnetRobot *rtnet, RT_TASK *taskHanlder, char *taskName);

/*****************************************************************************/

/**
 * \brief  Receives a packet from other task
 * \param  rtnet Pointer to st_rtnetRobot structure
 * \param  task Pointer to task's handler that is expected to receive a packet
 * \param  msg Pointer to received message
 * \return Messsage length if there is a packet, 0 otherwise.
 */
extern inline long robotGetPacket(st_rtnetRobot *rtnet, RT_TASK *task, void *msg);

/*****************************************************************************/

/**
 * \brief  Sends a packet to other task
 * \param  rtnet Pointer to st_rtnetStructure
 * \param  task Pointer to task's handler
 * \param  msg  Message to be sent
 * \return void
 */
extern inline void robotSendPacket(st_rtnetRobot *rtnet, RT_TASK *task, void *msg);

/*****************************************************************************/

/**
 * \brief  Release port and node from a task
 * \param  rtnet Pointer to st_rtnetRobot structure
 * \return void
 */
extern inline void rtnetPacketFinish(st_rtnetRobot *rtnet);

/*****************************************************************************/

#endif /* _RTNETAPI_H */
