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

/******************************************************************************/

/**
 * \brief  Main function
 * \param  argc Number of parameters in the command line
 * \param  argv array of parameters
 */
int main(int argc, char *argv[]) 
{
	if (argc != 1){
		fprintf(stderr, "Usage: ./robotSim\n\r");
		return -1;
	}
	
	printf("control init\n\r");
	robotControlThreadsMain();

	return 0;
}

