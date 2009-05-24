/******************************************************************************/
/**
* \file  robotStructs.h
* \brief StructsUsed both in control and robot.
*/
/******************************************************************************/

#ifndef _ROBOTSTRUCTS_H
#define _ROBOTSTRUCTS_H

/*linux includes*/
#include <pthread.h>
#include <arpa/inet.h>
#include <semaphore.h>

/*rtai includes*/
#include <rtai_sem.h>

/*robot includes*/
#include "robotDefines.h"


/************************* control structures ****************************/

//! Structure with reference models
typedef struct {
	double timeInstant[MAX_DATA_VALUE];	//! Current time
	double ref[MAX_DATA_VALUE];			//! x and y reference
	double ym[MAX_DATA_VALUE];			//! ymx and ymy
	double dRef[MAX_DATA_VALUE];		//! ref'
	int alpha;							//! alpha used
	int kIndex;							//! current index
} st_robotRefMod;

//! Structure used in reference generation
typedef struct {
	double xref;
	double yref;
} st_robotGeneration_t;

//! Control structure
typedef struct {
	double ym[YM_DIMENSION];
	double dym[YM_DIMENSION];
} st_robotControl_t;

//! Linearization structure
typedef struct {
	double v[V_DIMENSION];
} st_robotLin_t;

//! Main control shared structure
typedef struct {
	st_robotGeneration_t generation_t;
	st_robotControl_t control_t;
	st_robotLin_t lin_t;
	unsigned char alpha[ALPHA_DIMENSION];
} st_robotControl;	

//! Monitor shared structure
typedef struct {
	st_robotControl control;
	pthread_mutex_t mutexControl;
	pthread_mutex_t	mutexGen;
	pthread_mutex_t mutexLin;
} st_robotControlShared;

//! Linearization packet structure
typedef struct {
	double v[V_DIMENSION];
	double u[U_DIMENSION];
	double x[X_DIMENSION];
} st_robotLinPacket;

/************************* simulation structures ****************************/

//! Main structure with arrays
typedef struct {
	int kIndex;
	double timeInstant[MAX_DATA_VALUE];
	double dxVal[X_DIMENSION][MAX_DATA_VALUE];
	double xVal[X_DIMENSION][MAX_DATA_VALUE];
	double yVal[Y_DIMENSION][MAX_DATA_VALUE];
	double uVal[U_DIMENSION][MAX_DATA_VALUE];
} st_robotMainArrays;

//! Simulation structure base
typedef struct {
	double x[X_DIMENSION];
	double y[Y_DIMENSION];
	double u[U_DIMENSION];
} st_robotSimulPacket;

//! Simulation structure shared
typedef struct {
	double t;
	st_robotSimulPacket simul_t;
	pthread_mutex_t mutexSim;
} st_robotSimulShared;

/**************************** rtnet structures *******************************/

//! rtnet receive packet information
typedef struct {
	RT_TASK *recvfrom;
	unsigned long recvnode;
	unsigned long recvport;
	struct sockaddr_in addr;
	long len;
} st_rtnetReceive;

//! rtnet send packet information
typedef struct {
	RT_TASK *sendto;
	unsigned int sendnode;
	unsigned int sendport;
	struct sockaddr_in addr;
} st_rtnetSend;

/******************************************************************************/

#endif /* _ROBOTSTRUCTS_H */
