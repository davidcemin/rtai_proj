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

/**************************** rtnet structures *******************************/

//! rtnet main structure
typedef struct {
	unsigned long node;
	int port;
	struct sockaddr_in addr;
	char *ip;
} st_rtnetRobot;


/************************* control structures ****************************/

//! Structure with reference models
typedef struct {
	double timeInstant[MAX_DATA_VALUE];	
	double ref[MAX_DATA_VALUE];			
	double ym[MAX_DATA_VALUE];			
	double dRef[MAX_DATA_VALUE];		
	int kIndex;							
} st_robotRefMod;

typedef struct {
	double ym;
	double ymLast;
	double dym;
	double ts;
	double time[MAX_DATA_VALUE];
	double tc[MAX_DATA_VALUE];
	//double period[MAX_DATA_VALUE];
	int k;
} st_referenceModel_t;

//! Structure used in reference generation
typedef struct {
	double ref[REF_DIMENSION];
	double time[MAX_DATA_VALUE];
	double tc[MAX_DATA_VALUE];
	//double period[MAX_DATA_VALUE];
	int k;
} st_robotGeneration_t;

//! Control structure
typedef struct {
	double ym[YM_DIMENSION];
	double dym[YM_DIMENSION];
	double time[MAX_DATA_VALUE];
	//double period[MAX_DATA_VALUE];
	double tc[MAX_DATA_VALUE];
	int k;
} st_robotControl_t;

//! Linearization structure
typedef struct {
	double v[V_DIMENSION];
	double y[Y_DIMENSION];
	double x[X_DIMENSION];
	double alpha[ALPHA_DIMENSION][MAX_DATA_VALUE];
	double time[MAX_DATA_VALUE];
	double tc[MAX_DATA_VALUE];
	//double period[MAX_DATA_VALUE];
	int k;
} st_robotLin_t;

//! Main control shared structure
typedef struct {
	st_robotGeneration_t generation_t;
	st_referenceModel_t refmod_t;
	st_robotControl_t control_t;
	st_robotLin_t lin_t;
	double alpha[ALPHA_DIMENSION];
} st_robotControl;	

//! Control structure with mutexes
typedef struct {
	pthread_mutex_t mutexRefX;
	pthread_mutex_t mutexRefY;
	pthread_mutex_t mutexYmx;
	pthread_mutex_t mutexYmy;
	pthread_mutex_t mutexV;
	pthread_mutex_t mutexY;
	pthread_mutex_t mutexAlpha;
} st_controlMutex;

//! Control structure with semaphores
typedef struct {
	SEM *sm_control;
	SEM *sm_gen;
	SEM *sm_lin;
	SEM *sm_refx;
	SEM *sm_refy;
	sem_t sm_disp;
} st_controlSem;

//! Monitor shared structure
typedef struct {
	st_robotControl control;	//! Control structure
	st_controlMutex mutex;		//! mutex structure
} st_robotControlShared;

//! Linearization packet structure
typedef struct {
	double v[V_DIMENSION];
	double u[U_DIMENSION];
	double x[X_DIMENSION];
} st_robotLinPacket;

typedef struct {
	int tick;	//! tick of the tasks
	RTIME time;
	st_controlSem sem;
	char *ip;
} st_robotControlStack;


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

//! Main structure with arrays
typedef struct {
	int k;
	double time[MAX_DATA_VALUE];              //!< Current time
	double tc[MAX_DATA_VALUE];                //!< Execution data
	double period[MAX_DATA_VALUE];            //!< Period data
	double dx[X_DIMENSION][MAX_DATA_VALUE];   //!< x' data
	double x[X_DIMENSION][MAX_DATA_VALUE];    //!< x data
	double y[Y_DIMENSION][MAX_DATA_VALUE];    //!< y data
	double u[U_DIMENSION][MAX_DATA_VALUE];    //!< u data
} st_robotMain;

//! Simulation structure base
typedef struct {
	double x[X_DIMENSION];	//!< current x value
	double xl[X_DIMENSION];	//!< last x value
	double xd[X_DIMENSION]; //!< current x' value
	double y[Y_DIMENSION];	//!< current y value
	double u[U_DIMENSION];	//!< current u value
} st_robotSimulPacket;

//! Simulation structure shared
typedef struct {
	double t;
	st_robotSimulPacket simul_t;
	pthread_mutex_t mutexSim;
} st_robotSimulShared;

//! Simulation stack packet
typedef struct {
	int tick;	//! tick of the tasks
	char *ip;
	RTIME time;
	sem_t sm_disp;
} st_robotSimulStack;

/******************************************************************************/

#endif /* _ROBOTSTRUCTS_H */
