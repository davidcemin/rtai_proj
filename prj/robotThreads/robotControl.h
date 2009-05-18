/*****************************************************************************/
/**
 * \file robotControl.h
 * \brief Control thread prototypes
 */
/*****************************************************************************/

#ifndef _ROBOTCONTROL_H
#define _ROBOTCONTROL_H

/*****************************************************************************/

/**
* \brief Thread used to generate u and sample yf
* \param ptr Pointer to shared data
* \return void
*/
void *robotControl(void *ptr);

#endif /*_ROBOTCONTROL_H*/

