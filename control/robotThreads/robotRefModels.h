/*****************************************************************************/
/**
 * \file robotRefModels.h.h
 * \brief Reference model threads prototypes
 */
/*****************************************************************************/

#ifndef _ROBOTREFMODELS_H
#define _ROBOTREFMODELS_H

/*****************************************************************************/

/**
 * \brief Reference model x direction thread.
 * \param ptr pointer to shared memory
 * \return void
 */
extern void *robotRefModSimX(void *ptr);

/*****************************************************************************/

/**
 * \brief Reference model y direction thread.
 * \param ptr pointer to shared memory
 * \return void
 */
extern void *robotRefModSimY(void *ptr);

/*****************************************************************************/

#endif /*_ROBOTREFMODELS_H */






