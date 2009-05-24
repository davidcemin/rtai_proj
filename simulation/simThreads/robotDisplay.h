/*****************************************************************************/
/**
 * \file robotDisplay.h
 * \brief Display thread prototypes
 */
/*****************************************************************************/

#ifndef _ROBOTDISPLAY_H
#define _ROBOTDISPLAY_H

/*****************************************************************************/

/**
 * \brief  Thread used to show data on the screen. It is not a RTAI thread.
 * \param  ptr Pointer to shared memory
 * \return void
 */
extern void *robotThreadDisplay(void *ptr);

/*****************************************************************************/

#endif /* _ROBOTDISPLAY_H */
