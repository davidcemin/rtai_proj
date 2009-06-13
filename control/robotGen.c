/******************************************************************************/
/**
 * \file simulRobot.c
 * \brief  Main file used to generate the robot simulation
 */
/******************************************************************************/

/*includes libc*/
#include <stdio.h> 

/*includes robot*/
#include "robotThreads.h"

/*rtai*/
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
		fprintf(stderr, "Usage: ./control <ip remote>\n\r");
		return -1;
	}

	rt_allow_nonroot_hrt();
	rt_set_oneshot_mode();
	start_rt_timer(0);
	
	char *ip = argv[1];
	
	robotControlThreadsMain(ip);
	return 0;
}

