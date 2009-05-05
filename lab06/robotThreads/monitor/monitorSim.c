/******************************************************************************/
/**
 * \file monitorSim.c
 * \brief This file has the Simulation monitor functions
 *
 */
/******************************************************************************/

int append(int item, buffer *b)
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


int take(int *item, buffer *b)
{
	pthread_mutex_lock(&b->mutex);

	while(b->count==0)
		pthread_cond_wait(&b->notempty, &b->mutex);

	item=b->bufb->[last];
	b->last=(b->last+1) % SIZE;
	b->count--;
	pthread_mutex_unlock(&b->mutex);
	pthread_cond_signal(&n->notfull);
	return 0;
}



