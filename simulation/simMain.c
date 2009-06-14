/******************************************************************************/
/**
 * \file simulRobot.c
 * \brief  Main file used to generate the robot simulation
 */
/******************************************************************************/

/*includes libc*/
#include <stdio.h> 

/*includes robot*/
#include "simThreads.h"

/*rtai includes*/
#include <rtai_lxrt.h>
/******************************************************************************/

/**
 * \brief  Main function
 * \param  argc Number of parameters in the command line
 * \param  argv array of parameters
 */
int main(int argc, char *argv[]) 
{
	if (argc != 2){
		fprintf(stderr, "Usage: ./plant <ip remote>\n\r");
		return -1;
	}

	char *ip = argv[1];
	printf("%s\n\r", ip);
	
	rt_allow_nonroot_hrt();
	rt_set_oneshot_mode();
	start_rt_timer(0);

	robotSimThreadsMain(ip);
	
	return 0;
}

