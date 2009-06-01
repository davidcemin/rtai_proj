/*****************************************************************************/
/**
 * \file rtaiCtrl.c
 * \brief rtai init and clean
 */
/*****************************************************************************/


/*robot includes*/
#include "robotStructs.h"

/*rtai includes*/
#include <rtai_lxrt.h>

/*****************************************************************************/

int rtai_init_task(RT_TASK *task, int tick, char *name)
{
	int started_timer = 0;
	struct sched_param sched;
	rt_allow_nonroot_hrt(); 
	sched.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1; 
	sched_setscheduler(0, SCHED_FIFO, &sched);
 
	mlockall(MCL_CURRENT | MCL_FUTURE);
	
	/*registering realtime task*/
	if(!(task = rt_task_init( nam2num(name), 0, 0, 0))){
		fprintf(stderr, "Error in task init\n\r");
		return started_timer;
	}

	if(!rt_is_hard_timer_running())	{
		rt_set_oneshot_mode();
		start_rt_timer(tick);
		started_timer=1;
	}
	return started_timer;
}

/*****************************************************************************/

/**
 * \brief  It creates a rtai task
 * \param  task Pointer to task 
 * \param  taskName Task's name
 * \param  priority Task's priority
 * \param  stepTick Step timer
 * \return -1 error, 0 ok.
 */
inline int taskCreateRtai(RT_TASK *task, char *task_name, char priority, double stepTick) 
{
	int msgSize = 0;
	int stkSize = 0; /*default values*/
	unsigned long taskName = nam2num(task_name);
	struct sched_param sched;
	int started_timer=0;
	int period = (int) nano2count((RTIME)stepTick);

	/*set root permissions to user space*/
	rt_allow_nonroot_hrt();

	sched.sched_priority = sched_get_priority_max(SCHED_FIFO) - 1;
	sched_setscheduler(0,SCHED_FIFO,&sched);

	/*It Prevents the memory to be paged*/
    mlockall(MCL_CURRENT | MCL_FUTURE);

	//if(!(task = rt_thread_init(taskName, priority, 0, SCHED_FIFO, 0xFF))){
	if(!(task = rt_task_init_schmod(taskName, priority, stkSize, msgSize, SCHED_FIFO, 0xff) ) ) {	
		fprintf(stderr, "Cannot Init Task: ");
		return started_timer;
	}
	
	printf("INIT: %s; prior: %d\n\r", task_name, priority);

	//if(!rt_is_hard_timer_running())	{
		rt_set_oneshot_mode();
		start_rt_timer(period);
		started_timer=1;
	//}

	return started_timer;
}
	
/*****************************************************************************/

inline void mkTaksRealTime(RT_TASK *task, double stepTick, char *task_name)
{
	int period; 

	/*make it hard real time*/	
    rt_make_hard_real_time();

	/*set the period according to our desired tick*/
	period = (int)nano2count((RTIME)stepTick);

	/*start the task*/
	//rt_task_make_periodic(task, rt_get_time()+period, period);
	rt_task_make_periodic(task, rt_get_time(), period);
	printf("tsk %s running in real time!\n\r", task_name);
}

/*****************************************************************************/

/**
 * \brief  It destroys a rtai task
 * \param  task pointer to task to be destroyed
 * \return void
 */
inline void taskFinishRtai(RT_TASK *task, int started_timer)
{
	rt_make_soft_real_time();
	rt_task_delete(task);
	munlockall();
}

/*****************************************************************************/







