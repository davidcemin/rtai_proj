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

inline void rtnetSendPacketInit(st_rtnetSend *rtnet, char *sendtask)
{
	rtnet->sendto = NULL;

	printf("send packet init\n\r");
	inet_aton("127.0.0.1", &rtnet->addr.sin_addr);
	rtnet->sendnode = rtnet->addr.sin_addr.s_addr;
	//rtnet->sendnode = 0; /*forces to be local*/
	
	while ( (rtnet->sendport = rt_request_soft_port(rtnet->sendnode)) < 0) {	
		switch(rtnet->sendport) {
			case -EINVAL:
				printf("Can't find send port\n");
				usleep(100000);
				break;
			case -ENODEV:
				printf("Cant find node\n\r");
				usleep(100000);
				break;
			case -ETIMEDOUT:
				printf("Timeout receiving port");
				rtnet->sendport = 0; /*dummy*/
				break;
			default:
				printf("I should not be here!\n\r");
				break;
		}
	}
	printf("SEND port: %d\n", rtnet->sendport);

	rtnet->sendto = RT_get_adr(rtnet->sendnode, rtnet->sendport, sendtask);
	//while ( (rtnet->sendto = RT_get_adr(rtnet->sendnode, rtnet->sendport, sendtask)) == NULL) {
	//		usleep(100000);
	//		printf("Cant find send task\n");
	//}
}

/*****************************************************************************/

inline void rtnetRecvPacketInit(st_rtnetReceive *rtnet, char *recvtask)
{
	printf("recv init\n\r");
	inet_aton("127.0.0.1", &rtnet->addr.sin_addr);
	rtnet->recvnode = rtnet->addr.sin_addr.s_addr;
	//rtnet->recvnode = 0; /*Forces to be local */

	printf("a:0x%x\n\r", rtnet->recvnode);
	while ( (rtnet->recvport = rt_request_soft_port(rtnet->recvnode)) < 0) {
		switch(rtnet->recvport) {
			case -EINVAL:
				printf("Can't find receive port\n");
				usleep(100000);
				break;
			case -ENODEV:
				printf("Cant find node\n\r");
				usleep(100000);
				break;
			case -ETIMEDOUT:
				printf("Timeout receiving port");
				break;
			default:
				printf("I should not be here!\n\r");
				break;
		}
	}
	printf("RECV port: 0x%x\n", rtnet->recvport);
	
	rtnet->recvfrom = RT_get_adr(rtnet->recvnode, rtnet->recvport, recvtask);
	//while ( (rtnet->recvfrom = RT_get_adr(rtnet->recvnode, rtnet->recvport, recvtask)) == NULL) {
	//	usleep(100000);        
	//	printf("Can't find receive task\n");        
	//}
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

/*****************************************************************************/

inline int robotGetPacket(st_rtnetReceive *recv, void *msg)
{
	int ret = 0;
	if (RT_receivex_if(recv->recvnode, recv->recvport, recv->recvfrom, msg, sizeof(msg), &recv->len) == NULL)
		ret = -1;
	return ret;
}

/*****************************************************************************/

inline void robotSendPacket(st_rtnetSend *send, void *msg)
{
	RT_sendx(send->sendnode, -send->sendport, send->sendto, msg, sizeof(msg) );
}

/*****************************************************************************/
