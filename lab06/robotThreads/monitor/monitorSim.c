/******************************************************************************/
/**
 * \file monitorSim.c
 * \brief This file has the Simulation monitor functions
 *
 */
/******************************************************************************/

#include "monitorSim.h"
#include "robotCalcUtils.h" 



int monitorSim_append(int item, st_monitorBuffer *b)
{
	pthread_mutex_lock(&b->mutex);

	while(b->count==SIZE)
		pthread_cond_wait(&b->notfull, &b->mutex);

	b->buf[b->first]=item;
	b->count++;
	b->first=(b->first+1) % SIZE;
	pthread_mutex_unlock(&b->mutex);
	pthread_cond_signal(&b->notempty);
	return 0;
}


int monitorSim_take(int *item, st_monitorBuffer *b)
{
	pthread_mutex_lock(&b->mutex);

	while(b->count==0)
		pthread_cond_wait(&b->notempty, &b->mutex);

	*item = b->buf[b->last];
	b->last=(b->last+1) % SIZE;
	b->count--;
	pthread_mutex_unlock(&b->mutex);
	pthread_cond_signal(&b->notfull);
	return 0;
}

int monitorSim_run(st_robotShared *shared, st_robotMainArrays *robot)
{
	/* New X value */
	robotNewX(robot);

	/* Get u values from shared */
	getUFromShared(robot, shared);

	/* Calculates x' from x and u*/
	robotDxSim(robot);

	/* Calculates y from x and the inputs */
	robotCalcYFromX(robot);

	/* Copy y values into shared memory */
	cpYIntoShared(robot, shared);

}


