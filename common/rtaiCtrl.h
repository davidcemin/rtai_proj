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
 * \brief  It creates a rtai task
 * \param  task Pointer to task 
 * \param  taskName Task's name
 * \param  priority Task's priority
 * \param  stepTick Step timer
 * \return -1 error, 0 ok.
 */
inline int taskCreateRtai(RT_TASK *task, char *task_name, char priority, double stepTick, int msgSize);

/*****************************************************************************/

/**
 * \brief  It destroys a rtai task
 * \param  task pointer to task to be destroyed
 * \return void
 */
inline void taskFinishRtai(RT_TASK *task);

#endif /*_RTAI_CTRL_H */

