/*****************************************************************************/
/**
 * \file robotGeneration.h
 * \brief Generation thread prototypes
 */
/*****************************************************************************/

#ifndef _ROBOTGENERATION_H
#define _ROBOTGENERATION_H

/*****************************************************************************/

/**
 * \brief Generation thread. It is responsible to generate the references
 * \param ptr pointer to shared memory
 * \return void
 */
extern void *robotGeneration(void *ptr);

/*****************************************************************************/

#endif /*_ROBOTGENERATION_H */
