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

inline void rtnetPacketInit(st_rtnetRobot *rtnet)
{
	printf("rtnet packet init\n\r");
	inet_aton("127.0.0.1", &rtnet->addr.sin_addr);
	rtnet->node = rtnet->addr.sin_addr.s_addr;
	printf("0x%lx\n\r", rtnet->node);
	
	rtnet->port = rt_request_port(rtnet->node);
	while ( (rtnet->port = rt_request_port(rtnet->node)) <= 0) {	
		switch(rtnet->port) {
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
				break;
			default:
				printf("I should not be here!\n\r");
				break;
		}
	}
	printf("RTNET port: 0x%x\n", rtnet->port);
}

/*****************************************************************************/

inline void rtnetTaskWait(st_rtnetRobot *rtnet, RT_TASK *taskHandler, char *taskname)
{
	while ( (taskHandler = (RT_TASK *)RT_get_adr(rtnet->node, rtnet->port, taskname)) == NULL) {
			usleep(100000);
			printf("Cant find task %s\n", taskname);
	}
}

/*****************************************************************************/

inline long robotGetPacket(st_rtnetRobot *rtnet, RT_TASK *task, void *msg)
{
//	long len = 0;
//
//	void *msgtmp = malloc(sizeof(msg));
//	
//	RT_receivex_if(rtnet->node, -rtnet->port, task, msgtmp, sizeof(msg), &len);
//
//	if( len != sizeof(msg) ) {
//		printf("len: %ld msg: %d\n\r", len, sizeof(msg));
//		free(msgtmp);
//		return -1;
//	}
//	else
//		memcpy(msg, msgtmp, sizeof(msg));
//
//	free(msgtmp);
//	return len;
	printf("%d\n\r", sizeof(msg));
	return 0;
}

/*****************************************************************************/

inline void robotSendPacket(st_rtnetRobot *rtnet, RT_TASK *task, void *msg)
{
	printf("node: %ld; port: %d size: %d\n\r", rtnet->node, -rtnet->port, sizeof(msg));
	RT_sendx_if(rtnet->node, -rtnet->port, task, msg, sizeof(msg));
}

/*****************************************************************************/

inline void rtnetPacketFinish(st_rtnetRobot *rtnet)
{
	RT_release_port(rtnet->node, rtnet->port);
}

/*****************************************************************************/

