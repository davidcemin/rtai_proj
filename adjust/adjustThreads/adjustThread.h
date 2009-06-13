/*****************************************************************************/
/**
 * \file robotSimulation.h
 * \brief Threads used on simulation
 */
/*****************************************************************************/

#ifndef _ROBOTADJ_H
#define _ROBOTADJ_H

/**
 * \brief  Thread to simulate the robot 
 * \param  ptr Pointer to shared data
 * \return void
 */
void *adjustThread(char *ip);

#endif /*_ROBOTADJ_H*/

