/*****************************************************************************/
/**
 * \file robotThreads.h
 * \brief Threads used on simulation
 */
/*****************************************************************************/

#ifndef _ROBOTTHREADS_H
#define _ROBOTTHREADS_H

#include <pthread.h>
/*****************************************************************************/

typedef struct {
    pthread_mutex_t mutex;
    pthread_cond_t notfull;
    pthread_cond_t notempty;
    int count, first, last;
    int buf[10];
} st_monitorBuffer;


/**
 * \brief Threads main function. This function initializes all the other threads.
 * \return void
 */
void robotThreadsMain(void);
/*****************************************************************************/

#endif
