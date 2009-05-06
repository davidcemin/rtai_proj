/******************************************************************************/
/**
 * \file monitorSim.c
 * \brief This file has the Simulation monitor functions
 *
 */
/******************************************************************************/

#include "monitorSim.h"
#include "robotCalcUtils.h" 



int monitorSimSet(st_robotShared *shared)
{
	pthread_mutex_lock(shared->mutexShared);

	while(b->count==SIZE)
		pthread_cond_wait(&b->notfull, &b->mutex);

	b->buf[b->first]=item;
	b->count++;
	b->first=(b->first+1) % SIZE;
	pthread_mutex_unlock(&b->mutex);
	pthread_cond_signal(&b->notempty);
	return 0;
}


int monitorSimGet(int *item, st_monitorBuffer *b)
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


