/******************************************************************************/
/**
* \file   rtnetAPI.h
* \brief  API between rtnet and robot functions prototypes
*/
/******************************************************************************/

#ifndef _RTNETAPI_H
#define _RTNETAPI_H

#include "libRobot.h"

/******************************************************************************/

/**
 * \brief  
 */
extern inline void robotGetPacket(st_rtnetReceive *recv, void *msg);

/*****************************************************************************/

/**
 * \brief  
 */
extern inline void robotSendPacket(st_rtnetSend *send, void *msg);

/*****************************************************************************/

/**
 * \brief  
 */
extern inline void rtnetSendPacketInit(st_rtnetSend *rtnet, char *sendtask);

/*****************************************************************************/

/**
 * \brief  
 */
extern inline void rtnetRecvPacketInit(st_rtnetReceive *rtnet, char *recvtask);

/*****************************************************************************/

/**
 * \brief  
 */
extern inline void rtnetSendPacketFinish(st_rtnetSend *rtnet);

/*****************************************************************************/

/**
 * \brief  
 */
extern inline void rtnetRecvPacketFinish(st_rtnetReceive *rtnet);

#endif /* _RTNETAPI_H */
