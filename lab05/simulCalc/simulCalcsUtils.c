/******************************************************************************/
/**
 * \file simulCalcsUtils.c
 * \brief Functions used in the simulation 
 */
/******************************************************************************/

#include <sys/time.h> 
#include <stdlib.h> 
#include <stdio.h>

#include "libRobot.h"
#include "simulCalcsUtils.h"

/******************************************************************************/

double getTimeMilisec(void)
{
	double ret;
	struct timeval tv;

	gettimeofday(&tv, NULL);

	/*tv_sec has the number of seconds elapsed and tv_usec the number of microseconds in a second*/
	ret = ( (tv.tv_sec * 1000) + (tv.tv_usec / 1000 ) );
	
	return ret;
}
/******************************************************************************/

void robotInit(st_robotMainArrays *robotInit)
{
	int i, j;
	robotInit->kIndex = 0;
	robotInit->timeInstant[0] = 0;

	for (i = 0; i < 3; i++) {
		for(j = 0; j < MAX_DATA_VALUE; j++) {
			robotInit->xVal[i][j] = 0;
			robotInit->yVal[i][j] = 0;
			robotInit->dxVal[i][j] = 0;
		}
	}
	for (i = 0; i < 2; i++)
		for(j = 0; j < MAX_DATA_VALUE; j++)
			robotInit->uVal[i][j] = 0;
}
/******************************************************************************/

void robotInputCalc(st_robotShared *robot, double t)
{	
	if (t < 0.0) {
		robot->u[0] = 0;
		robot->u[1] = 0;
	}
	else if(t < 10.0)  {
		robot->u[0] = 1; 
		robot->u[1] = 0.2 * M_PI;
	}
	else { 
		/*t >=10 */
		robot->u[0] = 1;
		robot->u[1] = -0.2 * M_PI;
	}
}
/******************************************************************************/

/**
 * \brief  Calculate the period as specified
 * \param  period pointer to x array
 * \param  ret pointer to period's array
 * \param  nmemb number of members in x
 * \return void
 */
static int dataPeriod(double *period, double *ret, int nmemb)
{
	int i;
	FILE *fd;

	/* Opens a file to write */
	if ( (fd = fopen("period.dat","w")) == NULL) { 
		fprintf(stderr, "Error! Cannot open a file to write!\n\r");
		return -1;
	}

	ret[0] = 0.0;

	for (i = 1; i < nmemb; i++) {
		ret[i] = 1000 * (period[i] - period[i-1]);
		fprintf(fd, "%f\t%d\n", ret[i], i);
	}
	fclose(fd);
	return 0;
}
/******************************************************************************/

/**
 * \brief  Calculates a vectors mean
 * \param  val Pointer to the vector
 * \param  n nmemb of val
 * \param  mean Pointer to mean result
 * \return void
 */
static void dataMean(double *val, int n, double *mean)
{	
	double ret;
	int i;
	double nmemb = n - 1;
	ret = 0.0;
	*mean = 0.0;

	for (i = 0; i < n; i++)
		ret += val[i];
	
	ret = ret / nmemb;
	*mean = ret;
}

/******************************************************************************/

/**
 * \brief Calculates max value of a vector
 * \param period pointer to period vector
 * \param nmemb number of members in period
 * \param max pointer to max value
 * \return void
 */
static void maxValue(double *period, int nmemb, double *max)
{
	int i;
	
	*max = fabs(period[1]);

	for (i = 1; i < nmemb; i++) 
		if (fabs(period[i]) > *max) 
			*max = fabs(period[i]);
}
/******************************************************************************/

/**
 * \brief Calculates min value of a vector
 * \param period pointer to period vector
 * \param nmemb number of members in period
 * \param min pointer to min value
 * \return void
 */
static void minValue(double *period, int nmemb, double *min)
{
	int i;
	
	*min = fabs(period[1]);

	for (i = 1; i < nmemb; i++) 
		if (fabs(period[i]) < *min) 
			*min = fabs(period[i]);
}
/******************************************************************************/

/**
 * \brief  Calculates the variance and standard deviation
 * \param  period pointer to period array
 * \param  nmemb number of members
 * \param  mean Mean of the period values
 * \param  variance pointer to the result
 * \param  stddev pointer to standard deviation
 * \return void
 */
static void variance_stddev(double *period, int nmemb, double mean, double *variance, double *stddev)
{
	int i;
	double sum = 0.0;
	double calc = 0.0;

	for (i = 0; i < nmemb; i++){
		calc = period[i] - mean;
		if(calc < CALCERROR)
			calc = 0;
		sum += pow(calc, 2);
	}
	*variance = (double)(sum / (nmemb));
	*stddev = (double)sqrt(*variance);
}

/******************************************************************************/

/**
 * \brief  Calculates the period jitter
 * \param  period Pointer to period array
 * \param  nmemb Number of members in array
 * \param  jitter pointer to jitter array
 * \return -1 Error, 0 ok
 */
static int dataJitter(double *period, int nmemb, double *jitter)
{
	int i;
	FILE *fd;

	/* Opens a file to write */
	if ( (fd = fopen("jitter.dat","w")) == NULL) { 
		fprintf(stderr, "Error! Cannot open a file to write!\n\r");
		return -1;
	}
	for (i = 1; i < nmemb; i++) {
		jitter[i] = period[i] - (double)(STEPTIMESIM * 1000) ;
		fprintf(fd, "%f\t%d\n", jitter[i], i);
	}
	return 0;
}

/******************************************************************************/

int robotCalcData(st_robotMainArrays *robot)
{
	double nmemb = robot->kIndex;
	double *period;
	double *jitter;
	FILE *fd;
	
	double mean = 0.0;
	double maxT = 0.0;
	double minT = 0.0;
	double var  = 0.0;
	double dev  = 0.0;

	/* Allocate memory to period array */
	if ( (period = (double*) malloc(robot->kIndex * sizeof(double)) ) == NULL ) { 
		fprintf(stderr, "Not possible to allocate memory to main!\n\r");
		return -1;
	}

	/* Allocate memory to jitter array */
	if ( (jitter = (double*) malloc(robot->kIndex * sizeof(double)) ) == NULL ) { 
		fprintf(stderr, "Not possible to allocate memory to main!\n\r");
		return -1;
	}
	/* Opens a file to write */
	if ( (fd = fopen("results.dat","w")) == NULL) { 
		fprintf(stderr, "Error! Cannot open a file to write!\n\r");
		return -1;
	}


	if (dataPeriod(robot->timeInstant, period, nmemb) < 0) {
		free(period);
		free(jitter);
		return -1;
	}
	
	dataMean(period, nmemb, &mean);
	maxValue(period, nmemb, &maxT);
	minValue(period, nmemb, &minT);
	variance_stddev(period, nmemb, mean, &var, &dev);
	
	fprintf(fd, "Period:\n");

	fprintf(fd, "Mean: %f\n", mean);
	fprintf(fd, "MaxT: %f\n", maxT);
	fprintf(fd, "MinT: %f\n", minT);
	fprintf(fd, "Var:  %f\n", var);
	fprintf(fd, "Dev:  %f\n", dev);

	if(dataJitter(period, nmemb, jitter) < 0) {
		free(period);
		free(period);
		return -1;
	}

	mean = 0.0;
	maxT = 0.0;
	minT = 0.0;
	var  = 0.0;
	dev  = 0.0;

	dataMean(jitter, nmemb, &mean);
	maxValue(jitter, nmemb, &maxT);
	minValue(jitter, nmemb, &minT);
	variance_stddev(jitter, nmemb, mean, &var, &dev);
	
	fprintf(fd, "\n");
	fprintf(fd, "Jitter:\n");
	
	fprintf(fd, "Mean: %f\n", mean);
	fprintf(fd, "MaxT: %f\n", maxT);
	fprintf(fd, "MinT: %f\n", minT);
	fprintf(fd, "Var:  %f\n", var);
	fprintf(fd, "Dev:  %f\n", dev);

	fclose(fd);
	free(period);
	free(jitter);
	return 0;
}

