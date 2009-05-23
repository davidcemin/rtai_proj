/*****************************************************************************/
/**
 * \file rtnetAPI.c
 * \brief API between rtnet and robot functions
 */
/*****************************************************************************/

/*libc Includes*/
#include <arpa/inet.h>

/*robot includes*/
#include "robotStructs.h"
#include "rtnetAPI.h"

/*rtai includes*/
#include <rtai_netrpc.h>
#include <rtai_lxrt.h>

/*****************************************************************************/


inline void robotGetPacket(st_rtnetReceive *recv, void *msg)
{
	RT_receivex_if(recv->recvnode, recv->recvport, recv->recvfrom, msg, sizeof(msg), &recv->len);
}

/*****************************************************************************/

inline void robotSendPacket(st_rtnetSend *send, void *msg)
{
	RT_sendx(send->sendnode, -send->sendport, send->sendto, msg, sizeof(msg) );
}

/*****************************************************************************/

inline void rtnetSendPacketInit(st_rtnetSend *rtnet, char *sendtask)
{
	rtnet->sendto = NULL;

	inet_aton("127.0.0.1", &rtnet->addr.sin_addr);
	rtnet->sendnode = rtnet->addr.sin_addr.s_addr;
	rtnet->sendport = rt_request_soft_port(rtnet->sendnode);
	rtnet->sendto = RT_get_adr(rtnet->sendnode, rtnet->sendport, sendtask);
}

/*****************************************************************************/

inline void rtnetRecvPacketInit(st_rtnetReceive *rtnet, char *recvtask)
{
	rtnet->recvfrom = NULL;
	
	inet_aton("127.0.0.1", &rtnet->addr.sin_addr);
	rtnet->recvnode = rtnet->addr.sin_addr.s_addr;
	rtnet->recvport = rt_request_soft_port(rtnet->recvnode);
	rtnet->recvfrom = RT_get_adr(rtnet->recvnode, rtnet->recvport, recvtask);
}

/*****************************************************************************/

inline void rtnetSendPacketFinish(st_rtnetSend *rtnet)
{
	RT_release_port(rtnet->sendnode, rtnet->sendport);
}

/*****************************************************************************/

inline void rtnetRecvPacketFinish(st_rtnetReceive *rtnet)
{
	RT_release_port(rtnet->recvnode, rtnet->recvport);
}


