/******************************************************************************/
/**
 * \file monitorSim.h
 * \brief This file has the Simulation monitor prototypes, structs and defines
 *
 */
/******************************************************************************/

#ifndef _MONITORSIM_H
#define _MONITORSIM_H

#include <pthreads.h>
#define SIZE 10

typedef struct {
    pthread_mutex_t mutex;
    pthread_cond_t notfull;
    pthread_cond_t notempty;
    int count, first, last;
    int buf[SIZE];
} buffer;


#endif //! _MONITORSIM_H
