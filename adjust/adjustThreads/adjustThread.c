/*****************************************************************************/
/**
 * \file adjustThread.c
 * \brief These functions are fully based on kbrd functions from showroom directory
 */
/*****************************************************************************/

/*libc Includes*/
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <arpa/inet.h>
#include <semaphore.h>
#include <string.h> 
#include <termio.h>
#include <sys/time.h> 

/*robot includes*/
#include "adjustThread.h"
#include "libRobot.h"
#include "robotStructs.h"
#include "rtnetAPI.h"
#include "rtaiCtrl.h"

/*rtai includes*/
#include <rtai_netrpc.h>
#include <rtai_lxrt.h>

/******************************************************************************/
static void menu(double alpha1, double alpha2)
{
	printf("\n");
	printf("  _________________________________________________\n");
	printf(" |                ALPHA VALUES                     |\n");
	printf(" |     ALPHA1 = y: alpha1+=0.2  h: alpha1-=0.2     |\n");
	printf(" |     ALPHA2 = u: alpha2+=0.2  j: alpha2-=0.2     |\n");
	printf(" |     q: End adjust thread                        |\n");
	printf(" |     ALPHA1: %f ALPHA2: %f           |\n", alpha1, alpha2);
	printf("  -------------------------------------------------\n");
	printf("\n");
}

static char get_key(void)
{
	static struct termio my_kb, orig_kb;
	static int first = 1;
	int ch;
	if (first) {
		first = 0;
		ioctl(0, TCGETA, &orig_kb);
		my_kb = orig_kb;
		my_kb.c_lflag &= ~(ECHO | ISIG | ICANON);
		my_kb.c_cc[4] = 1;
	}
	ioctl(0, TCSETAF, &my_kb);
	ch = getchar();
	ioctl(0, TCSETAF, &orig_kb);
	return ch;
}
/******************************************************************************/
void *adjustThread(char *ip)
{
	RT_TASK *adjusttsk = NULL;
	double alpha[ALPHA_DIMENSION];
	char ch;

	/* init rt task */
	rt_allow_nonroot_hrt();

	/*registering realtime task*/
	if((adjusttsk = rt_thread_init(nam2num(ADJTSK), ADJPRIORITY, 0, SCHED_FIFO, 0xFF)) == 0) {
		printf("adjust task init error\n\r");
		pthread_exit(NULL);
	}

	st_rtnetRobot rtnet;
	unsigned long ctrlnode = 0;
	int ctrlport=0;
	rtnet.ip = ip;

	/*init rtnet*/
	rtnetPacketInit(&rtnet);
	ctrlport = rtnet.port;
	ctrlnode = rtnet.node;
	
	RT_TASK *ctrltsk=NULL;
	
	while((ctrltsk=(RT_TASK *)RT_get_adr(ctrlnode,ctrlport,LINEARTSK))==NULL) {
		usleep(100000);
		printf("Waiting %s to be up...\n\r", LINEARTSK);
	}

    printf("INIT \n");

	alpha[ALPHA_1] = ALPHA_1_INIT;
	alpha[ALPHA_2] = ALPHA_2_INIT;
	menu(alpha[ALPHA_1], alpha[ALPHA_2]);
	do {
	        ch = get_key();
	        printf("%c ", toupper(ch));
//	        if (ch == 'p' || ch == 'P') {
			menu(alpha[ALPHA_1], alpha[ALPHA_2]);
//		}
		if (ch != 'q' && ch == 'y') {
			alpha[ALPHA_1] += 0.2;
			RT_sendx(ctrlnode, -ctrlport, ctrltsk, alpha, sizeof(alpha));
			menu(alpha[ALPHA_1], alpha[ALPHA_2]);
		}
		if (ch != 'q' && ch == 'h') {
			alpha[ALPHA_1] -= 0.2;
			RT_sendx(ctrlnode, -ctrlport, ctrltsk, alpha, sizeof(alpha));
			menu(alpha[ALPHA_1], alpha[ALPHA_2]);
		}
		if (ch != 'q' && ch == 'u') {
			alpha[ALPHA_2] += 0.2;
			RT_sendx(ctrlnode, -ctrlport, ctrltsk, alpha, sizeof(alpha));
			menu(alpha[ALPHA_1], alpha[ALPHA_2]);
		}
		if (ch != 'q' && ch == 'j') {
			alpha[ALPHA_2] -= 0.2;
			RT_sendx(ctrlnode, -ctrlport, ctrltsk, alpha, sizeof(alpha));
			menu(alpha[ALPHA_1], alpha[ALPHA_2]);
		}
	} while (ch != 'q');

	RT_release_port(rtnet.node, rtnet.port);
	munlockall();
	rt_task_delete(adjusttsk);
	pthread_exit (NULL);
}

