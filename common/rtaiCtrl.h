/*****************************************************************************/
/**
 * \file rtaiCtrl.h
 * \brief rtai init and clean
 */
/*****************************************************************************/

#ifndef _RTAICTRL_H
#define _RTAICTRL_H

/*robot includes*/
#include "robotStructs.h"

/*rtai includes*/
#include <rtai_lxrt.h>

/*****************************************************************************/

/**
 * \brief  
 */
extern int rtai_init_task(RT_TASK *task, int tick, char *name);

	
/*****************************************************************************/
/**
 * \brief  It creates a rtai task
 * \param  task Pointer to task 
 * \param  taskName Task's name
 * \param  priority Task's priority
 * \param  stepTick Step timer
 * \return -1 error, 0 ok, 1 started_timer.
 */
extern inline int taskCreateRtai(RT_TASK *task, char *task_name, char priority, double stepTick);

/*****************************************************************************/

/**
 * \brief  Make a rtai task hard real time
 * \param  task Pointer to the task
 * \param  stepTick Clock tick
 * \param  task_name Name of the current task
 * \return void
 */
extern inline void mkTaksRealTime(RT_TASK *task, double stepTick, char *task_name);

/*****************************************************************************/
/**
 * \brief  It destroys a rtai task
 * \param  task pointer to task to be destroyed
 * \param  started_timer flag to tell if the timer was initialized
 * \return void
 */
extern inline void taskFinishRtai(RT_TASK *task, int started_timer);

#endif /*_RTAI_CTRL_H */

