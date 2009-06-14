/******************************************************************************/
/**
 * \file simulCalcsUtils.c
 * \brief Functions used in the simulation 
 */
/******************************************************************************/

#include <sys/time.h> 
#include <stdlib.h> 
#include <stdio.h>
#include <string.h>

#include "robotStructs.h"
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

/**
 * \brief  
 */
static void robotSaveToFileSim(st_robotMain *data, char *name)
{
	FILE *fd;
	int k;

	/* Opens a file to write */
	if ( (fd = fopen(name, "w+")) == NULL) { 
		fprintf(stderr, "Error! Cannot open a file to write!\n\r");
	}
	
	fprintf(stdout, "Writing %s\n", name);

	for(k = 2; k < data->k; k++)
		fprintf(fd, "%f\t%f\t%f\t%f\t%f\t%f\t%f\n",
				data->time[k], 
				data->y[0][k], 
				data->y[1][k], 
				data->x[2][k], 
				data->u[0][k], 
				data->u[1][k],
				data->tc[k]);
	fclose(fd);
}

/******************************************************************************/

/**
 * \brief  
 */
static void robotSaveToFileGen(st_robotGeneration_t *data, char *name)
{
	FILE *fd;
	int k;

	/* Opens a file to write */
	if ( (fd = fopen(name, "w+")) == NULL) { 
		fprintf(stderr, "Error! Cannot open a file to write!\n\r");
	}
	
	fprintf(stdout, "Writing %s\n", name);

	for(k = 1; k < data->k; k++)
		fprintf(fd, "%f\t%f\t%f\t%f\t%d\n", data->time[k], data->ref[XREF_POSITION][k], data->ref[YREF_POSITION][k], data->tc[k], k);
	fclose(fd);
}

/******************************************************************************/

/**
 * \brief  
 */
static void robotSaveToFileRefx(st_referenceModel_t *data, char *name)
{
	FILE *fd;
	int k;

	/* Opens a file to write */
	if ( (fd = fopen(name, "w+")) == NULL) { 
		fprintf(stderr, "Error! Cannot open a file to write!\n\r");
	}
	
	fprintf(stdout, "Writing %s\n", name);

	for(k = 1; k < data->k; k++)
		fprintf(fd, "%f\t%f\t%d\n", data->time[k], data->tc[k], k);
	fclose(fd);
}

/******************************************************************************/

/**
 * \brief  
 */
static void robotSaveToFileRefy(st_referenceModel_t *data, char *name)
{
	FILE *fd;
	int k;

	/* Opens a file to write */
	if ( (fd = fopen(name, "w+")) == NULL) { 
		fprintf(stderr, "Error! Cannot open a file to write!\n\r");
	}
	
	fprintf(stdout, "Writing %s\n", name);

	for(k = 1; k < data->k; k++)
		fprintf(fd, "%f\t%f\t%d\n", data->time[k], data->tc[k], k);
	fclose(fd);
}

/******************************************************************************/

/**
 * \brief  
 */
static void robotSaveToFileCtrl(st_robotControl_t *data, char *name)
{
	FILE *fd;
	int k;

	/* Opens a file to write */
	if ( (fd = fopen(name, "w+")) == NULL) { 
		fprintf(stderr, "Error! Cannot open a file to write!\n\r");
	}
	
	fprintf(stdout, "Writing %s\n", name);

	for(k = 1; k < data->k; k++)
		fprintf(fd, "%f\t%f\t%d\n", data->time[k], data->tc[k], k);
	fclose(fd);
}

/******************************************************************************/

/**
 * \brief  
 */
static void robotSaveToFileLin(st_robotLin_t *data, char *name)
{
	FILE *fd;
	int k;

	/* Opens a file to write */
	if ( (fd = fopen(name, "w+")) == NULL) { 
		fprintf(stderr, "Error! Cannot open a file to write!\n\r");
	}
	
	fprintf(stdout, "Writing %s\n", name);

	for(k = 1; k < data->k; k++)
		fprintf(fd, "%f\t%f\t%f\t%f\t%d\n", data->time[k], data->tc[k], data->alpha[ALPHA_1][k], data->alpha[ALPHA_2][k], k);
	fclose(fd);
}

/******************************************************************************/

/**
 * \brief  
 */
inline void saveToFileGeneric(int type, void *ptr)
{
	switch (type) {
		case FILE_GEN:
			robotSaveToFileGen((st_robotGeneration_t*)ptr, "gen.dat");
			break;

		case FILE_REFX:
			robotSaveToFileRefx((st_referenceModel_t*)ptr, "refx.dat");
			break;

		case FILE_REFY:
			robotSaveToFileRefy((st_referenceModel_t*)ptr, "refy.dat");
			break;

		case FILE_CTRL:
			robotSaveToFileCtrl((st_robotControl_t*)ptr, "ctrl.dat");
			break;
		
		case FILE_LIN:
			robotSaveToFileLin((st_robotLin_t*)ptr, "lin.dat");
			break;
		
		case FILE_SIM:
			robotSaveToFileSim((st_robotMain*)ptr, "sim.dat");
			break;

		default:
			printf("File write error: I should not be here!\n\r");
			break;
	}
}

/*****************************************************************************/

/**
 * \brief  Calculate the period as specified
 * \param  period pointer to x array
 * \param  ret pointer to period's array
 * \param  nmemb number of members in x
 * \param  filename name of file to concatenate
 * \return void
 */
static int dataPeriod(double *period, int nmemb, char *filename)
{
	int i;
	FILE *fd2;
	char s[30] = "period_";

	printf("%s\n\r", filename);
	strcat(s, filename);
	printf("%s\n\r", s);

	/* Opens a file to write */
	if ( (fd2 = fopen(s,"w+")) == NULL) { 
		fprintf(stderr, "Error! Cannot open a file to write!\n\r");
		return -1;
	}

	fprintf(stdout, "Writing %s\n", s);
	for (i = 2; i < nmemb; i++) {
		fprintf(fd2, "%f\t%d\n", period[i], i -2);
	}
	fclose(fd2);
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
		ret += (val[i]);
	
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
	
	*min = fabs(period[2]);

	for (i = 2; i < nmemb; i++) 
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

	for (i = 2; i < nmemb; i++){
		calc = period[i] - mean;
		if(calc < CALCERROR)
			calc = 0;
		sum += pow(calc, 2);
	}
	*variance = (double)(sum / (nmemb-2));
	*stddev = (double)sqrt(*variance);
}

/******************************************************************************/

/**
 * \brief  Calculates the period jitter
 * \param  period Pointer to period array
 * \param  nmemb Number of members in array
 * \param  jitter pointer to jitter array
 * \param  filename name of file to write to
 * \return -1 Error, 0 ok
 */
static int dataJitter(double *period, double mean, int nmemb, double *jitter, char *filename)
{
	int i;
	FILE *fd1;	
	char s[30] = "period_";

	strcat(s, filename);

	/* Opens a file to write */
	if ( (fd1 = fopen(s,"w+")) == NULL) { 
		fprintf(stderr, "Error! Cannot open a file to write!\n\r");
		return -1;
	}
	fprintf(stdout, "Writing %s\n", s);
	for (i = 2; i < nmemb; i++) {
		jitter[i] = period[i] - mean;
		fprintf(fd1, "%f\t%d\n", jitter[i], i-2);
	}
	fclose(fd1);
	return 0;
}

/******************************************************************************/

int robotCalcData(double *tc, int nmemb, char *filename)
{
	double jitter[MAX_DATA_VALUE];
	FILE *fd;
	
	double mean = 0.0;
	double maxT = 0.0;
	double minT = 0.0;
	double var  = 0.0;
	double dev  = 0.0;

	/* Opens a file to write */
	if ( (fd = fopen(filename,"w+")) == NULL) { 
		fprintf(stderr, "Error! Cannot open a file to write!\n\r");
		return -1;
	}

	if (dataPeriod(tc, nmemb, filename) < 0) {
		return -1;
	}
	
	dataMean(tc, nmemb, &mean);
	maxValue(tc, nmemb, &maxT);
	minValue(tc, nmemb, &minT);
	variance_stddev(tc, nmemb, mean, &var, &dev);
	
	fprintf(fd, "Period:\n");

	fprintf(fd, "Mean: %f\n", mean);
	fprintf(fd, "MaxT: %f\n", maxT);
	fprintf(fd, "MinT: %f\n", minT);
	fprintf(fd, "Var:  %f\n", var);
	fprintf(fd, "Dev:  %f\n", dev);

	if(dataJitter(tc, mean, nmemb, jitter, filename) < 0) {
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
	return 0;
}

