#include <stdio.h>
#include <malloc.h>
// strtod() needs the stdlib.h, but apparently the prototype
// of it is wrongly defined in string.h
// https://cboard.cprogramming.com/c-programming/173070-strtod-standard-library-not-working.html
#include <string.h>
#include <stdlib.h>
#include "pidcontrol.h"

#ifdef PID_FIXPOINT
/* Define sample variance threshold depending on the 
 * integer precision 
 * standard deviation should be better than error of sqrt(2) in second last decimal point
 * ==> var_thresh = sqrt(2)^2 * 10^(-2(IntegerPrecision-1)) */
	#if PID_INTEGER_PRECISION == 0
		#define DEFAULT_SAMPLE_VAR_THRESH 2e2
	#elif PID_INTEGER_PRECISION == 1
		#define DEFAULT_SAMPLE_VAR_THRESH 2e0
	#elif PID_INTEGER_PRECISION == 2
		#define DEFAULT_SAMPLE_VAR_THRESH 2e-2
	#elif PID_INTEGER_PRECISION == 3
		#define DEFAULT_SAMPLE_VAR_THRESH 2e-4
	#elif PID_INTEGER_PRECISION == 4
		#define DEFAULT_SAMPLE_VAR_THRESH 2e-6
	#elif PID_INTEGER_PRECISION == 5
		#define DEFAULT_SAMPLE_VAR_THRESH 2e-8
	#elif PID_INTEGER_PRECISION == 6
		#define DEFAULT_SAMPLE_VAR_THRESH 2e-10
	#elif PID_INTEGER_PRECISION == 7
		#define DEFAULT_SAMPLE_VAR_THRESH 2e-12
	#elif PID_INTEGER_PRECISION == 8
		#define DEFAULT_SAMPLE_VAR_THRESH 2e-14
	#elif PID_INTEGER_PRECISION == 9
		#define DEFAULT_SAMPLE_VAR_THRESH 2e-16
	#elif PID_INTEGER_PRECISION == 10
		#define DEFAULT_SAMPLE_VAR_THRESH 2e-18
	#elif PID_INTEGER_PRECISION == 11
		#define DEFAULT_SAMPLE_VAR_THRESH 2e-20
	#elif PID_INTEGER_PRECISION == 12
		#define DEFAULT_SAMPLE_VAR_THRESH 2e-22
	#elif PID_INTEGER_PRECISION == 13
		#define DEFAULT_SAMPLE_VAR_THRESH 2e-24
	#elif PID_INTEGER_PRECISION == 14
		#define DEFAULT_SAMPLE_VAR_THRESH 2e-26
	#elif PID_INTEGER_PRECISION == 15
		#define DEFAULT_SAMPLE_VAR_THRESH 2e-28
	#elif PID_INTEGER_PRECISION == 16
		#define DEFAULT_SAMPLE_VAR_THRESH 2e-30
	#endif  
#else
/* 2e-13 was chosen arbitrarily */
#define DEFAULT_SAMPLE_VAR_THRESH 2e-13
#endif

int main(int argc, char* argv[])
{
	int    DataSets = 0;
	int    i = 0;
	double* tSim;
	double* eSim;
	double* yPSim;
	double* yPISim;
	double* yPIDSim;
	PIDValue* yPLib;
	PIDValue* yPILib;
	PIDValue* yPIDLib;
	PIDValue*  eLib;
	PIDValue Kp_Read, Ki_Read, Kd_Read, Tf_Read, TSample_Read, Kr_Read, Tn_Read, Tv_Read;
	PIDValue *err;
	
#ifndef PID_FIXPOINT
	puts("Library got compiled for using FLOATING POINT.");
#else
	puts("Library got compiled for using FIXPOINT.");
	printf("Fixpoint precision = %d, fixpoint factor = %d\n", PID_INTEGER_PRECISION, PID_FIXPOINT_FACTOR);
	double d; //just a helper variable for fixpoint calculation
#endif
	
	
	/* Read parameters from command line 
	 * Which controller should be tested?
	 * Threshold for sample variance?
	 * Command Line Options: pidtest [Testmode] [variance_thresh]
	 * TestMode = 1 (P), 2(PI), 3(PID)
	 * variance_thresh e.g. "1.423095e-13"
	 * if variance_thresh is not provided, then it will be  2e-13
	 * */
	 char usage_string[] = "Usage ./pidtest [TestMode] [variance_thres] \n "
	 "TestMode = 1,2 or 3 \n1 ==> Test P-Controller \n2 ==> Test PI-Controller\n"
	 "3 ==> Test PID-Controller \n"
	 "variance_thresh e.g. \"1.423095e-13\". If not provided then it will be 2e-13\n\n";
	 
	if (argc < 2 || argc > 3) {
		 puts(usage_string);
		 return 1;
	}
	//read test mode
	int test_mode = strtol(argv[1], NULL, 10);
	if (test_mode < 1 || test_mode > 3) {
		puts(usage_string);
		return 1;
	}
	else
		printf("Test Mode is %d\n", test_mode);
	//read threshold, if provided
	double sample_var_thresh = 0;
	if (argc == 3) {
		sample_var_thresh = strtod(argv[2], NULL);
		if (sample_var_thresh == 0.0) {
			printf("%f\n", sample_var_thresh);
			puts(usage_string);
			return 1;
		}
	}
	else
		sample_var_thresh = (double)DEFAULT_SAMPLE_VAR_THRESH;
		
	printf("Threshold for sample variance is %e\n", sample_var_thresh);
	i = 0;
	

#ifndef PID_FIXPOINT
	/* For the floating point test we assume the following parameters:
	   Kp = 2, Ki = 0.5*1/s, Kd = 2s, TSample = 0.5s, Tf = 2 s
     */
	PIDValue Kp      = 2.0;
	PIDValue Ki      = 0.5;
	PIDValue Kd      = 2;
	PIDValue Tf      = 2;
	PIDValue TSample = 0.5;
#else
	/* When using fixpoint arithmetic we have to consider the following 
	   for the floating point parameters given above:
	   Sample Time = 0.5s -> smallest possible integer representation: 5
	   Hence, Tf = 20 and Ki = 0.05*1/s and Kd = 20s.
	   When having 4 decimals this means we have to pass the following parameter
	   Kp = 20000 (2.0000), Ki = 50 (0.0050), Kd = 200000 (20.0000), 
	   Tf = 20, Ta = 5
    */
	PIDValue Kp      = 2*PID_FIXPOINT_FACTOR;
	PIDValue Ki      = (PIDValue)(0.05*PID_FIXPOINT_FACTOR);
	PIDValue Kd      = 20*PID_FIXPOINT_FACTOR;
	PIDValue Tf      = 20;
	PIDValue TSample = 5;
#endif

	char	line[256];
	FILE*			datafile;
	int	choice = 0;

	/* Open the file with the reference test-data */
	datafile = fopen("PIDControlTestData.txt", "r");
	if (datafile == NULL) 
	{
		printf("Couldn't open datafile for reading\n");
		return 1;
	}

	/* Determine the number of datasets (=rows) provided by the test-data */
	memset(line, 0, sizeof(line));
	while (	fgets(line, sizeof(line), datafile) != NULL ) DataSets++;

	/* Allocate corresponding memory for the data to import */
	tSim    = malloc(DataSets*sizeof(double));
	eSim    = malloc(DataSets*sizeof(double));
	eLib    = malloc(DataSets*sizeof(PIDValue));
	yPSim   = malloc(DataSets*sizeof(double));
	yPISim  = malloc(DataSets*sizeof(double));
	yPIDSim = malloc(DataSets*sizeof(double));
	yPLib   = malloc(DataSets*sizeof(PIDValue));
	yPILib  = malloc(DataSets*sizeof(PIDValue));
	yPIDLib = malloc(DataSets*sizeof(PIDValue));
	err = malloc(DataSets*sizeof(PIDValue));

	/* Reset the file pointer to the beginning of the file and import the data */
	fseek(datafile, 0, SEEK_SET);
	while (	fgets(line, sizeof(line), datafile) != NULL )
	{
		sscanf(line, "%lf\t%lf\t%lf\t%lf\t%lf\n", &tSim[i], &eSim[i], &yPSim[i], &yPISim[i], &yPIDSim[i]);
		i++;
	}
	fclose(datafile);

	/* Run the controllers */
	pid_Init();
	pid_ParaSet_K(0, Kp, 0, 0, 0, TSample);
	pid_ParaSet_K(1, Kp, Ki, 0, 0, TSample);
	pid_ParaSet_K(2, Kp, Ki, Kd, Tf, TSample);
	for (i=0; i < DataSets; i++)
	{
#ifdef PID_FIXPOINT
		eLib[i] = (PIDValue)(eSim[i]*PID_FIXPOINT_FACTOR);
#else
		eLib[i] = (PIDValue)eSim[i];
#endif
		pid_Step(0, eLib[i], &yPLib[i]);
		pid_Step(1, eLib[i], &yPILib[i]);
		pid_Step(2, eLib[i], &yPIDLib[i]);
	}

	printf("Size of int8_t: %lu Byte\n", sizeof(int8_t));
	printf("Size of uint8_t: %lu Byte\n", sizeof(uint8_t));
	printf("Size of int16_t: %lu Byte\n", sizeof(int16_t));
	printf("Size of uint16_t: %lu Byte\n", sizeof(uint16_t));
	printf("Size of int32_t: %lu Byte\n", sizeof(int32_t));
	printf("Size of uint32_t: %lu Byte\n", sizeof(uint32_t));
	printf("Size of int64_t: %lu Byte\n", sizeof(int64_t));
	printf("Size of uint64_t: %lu Byte\n", sizeof(uint64_t));
	printf("Size of float: %lu Byte\n", sizeof(float));
	printf("Size of double: %lu Byte\n\n", sizeof(double));

	// use the input from the command line to specify
	// which controller is supposed to be tested
	choice = test_mode; 
	switch (choice) {
		case 1:
			printf("Testing P-Controller\n");
			break;
		case 2:
			printf("Testing PI-Controller\n");
			break;
		case 3:
			printf("Testing PID-Controller\n");
	};

	
#ifdef PID_FIXPOINT
	printf("Data in integers\n");
	printf("\nt \t\t eLib \t\t Sim \t\t Lib \t\t Err \n");
	switch (choice)
	{
	case 1:
		for (i=0; i < DataSets; i++) {
		printf("%.8f\t %d\t\t %d\t\t %d\t\t %d\n", tSim[i], eLib[i], (int)(yPSim[i] * PID_FIXPOINT_FACTOR), yPLib[i], yPLib[i] - (int)(yPSim[i] * PID_FIXPOINT_FACTOR));
		err[i] = yPLib[i] - (int)(yPSim[i] * PID_FIXPOINT_FACTOR);
		}
		pid_ParaGet_T(0, &Kr_Read, NULL, NULL, &Tf_Read, &TSample_Read );
		pid_ParaGet_K(0, &Kp_Read, NULL, NULL, NULL, NULL );
		printf("Kr = %d (should be: %d)\n", Kr_Read, Kp);
		printf("Kp = %d (should be: %d)\n", Kp_Read, Kp);
		printf("Kr = %d (should be: %d)\n", Kr_Read, Kp);
		printf("TSample = %d (should be: %d)\n", TSample_Read, TSample);
		break;
	case 2:
		for (i=0; i < DataSets; i++) {
			printf("%.8f\t %d\t\t %d\t\t %d\t\t %d\n", tSim[i], eLib[i], (int)(yPISim[i] * PID_FIXPOINT_FACTOR), yPILib[i], yPILib[i] - (int)(yPISim[i] * PID_FIXPOINT_FACTOR));
			err[i] = yPILib[i] - (int)(yPISim[i] * PID_FIXPOINT_FACTOR);
		}
		pid_ParaGet_T(1, &Kr_Read, &Tn_Read, NULL, &Tf_Read, &TSample_Read );
		pid_ParaGet_K(1, &Kp_Read, &Ki_Read, NULL, NULL, NULL );
		printf("Kr = %d (should be: %d)\n", Kr_Read, Kp);
		printf("Kp = %d (should be: %d)\n", Kp_Read, Kp);
		printf("Ki = %d (should be: %d)\n", Ki_Read, Ki);
		printf("Kr = %d (should be: %d)\n", Kr_Read, Kp);
		printf("Tn = %d (should be: %d)\n", Tn_Read, Kp/Ki);
		printf("TSample = %d (should be: %d)\n", TSample_Read, TSample);
		break;
	case 3:
		for (i=0; i < DataSets; i++) {
			printf("%.8f\t %d\t\t %d\t\t %d\t\t %d\n", tSim[i], eLib[i], (int)(yPIDSim[i] * PID_FIXPOINT_FACTOR), yPIDLib[i], yPIDLib[i] - (int)(yPIDSim[i] * PID_FIXPOINT_FACTOR));
			err[i] = yPIDLib[i] - (int)(yPIDSim[i] * PID_FIXPOINT_FACTOR);
		}
		pid_ParaGet_T(2, &Kr_Read, &Tn_Read, &Tv_Read, &Tf_Read, &TSample_Read );
		pid_ParaGet_K(2, &Kp_Read, &Ki_Read, &Kd_Read, NULL, NULL );
		printf("Kr = %d (should be: %d)\n", Kr_Read, Kp);
		printf("Kp = %d (should be: %d)\n", Kp_Read, Kp);
		printf("Ki = %d (should be: %d)\n", Ki_Read, Ki);
		printf("Kd = %d (should be: %d)\n", Kd_Read, Kd);
		printf("Kr = %d (should be: %d)\n", Kr_Read, Kp);
		printf("Tn = %d (should be: %d)\n", Tn_Read, Kp/Ki);
		printf("Tv = %d (should be: %d)\n", Tv_Read, Kd/Kp);
		printf("Tf = %d (should be: %d)\n", Tf_Read, Tf);
		printf("TSample = %d (should be: %d)\n", TSample_Read, TSample);
		break;
	}
#else
	switch (choice)
	{
	case 1:
		for (i=0; i < DataSets; i++) {
			printf("%.8f\t %.8f\t %.8f\t %.8f\t %.8f\n", tSim[i], eSim[i], yPSim[i], yPLib[i], yPLib[i] - yPSim[i]);
			err[i] = yPLib[i] - yPSim[i];
		}
		pid_ParaGet_T(0, &Kr_Read, NULL, NULL, &Tf_Read, &TSample_Read );
		pid_ParaGet_K(0, &Kp_Read, NULL, NULL, NULL, NULL );
		printf("Kr = %.2f (should be: %.2f)\n", Kr_Read, Kp);
		printf("Kp = %.2f (should be: %.2f)\n", Kp_Read, Kp);
		printf("Kr = %.2f (should be: %.2f)\n", Kr_Read, Kp);
		printf("TSample = %.2f (should be: %.2f)\n", TSample_Read, TSample);
		break;
		
	case 2:
		for (i=0; i < DataSets; i++) {
			printf("%.8f\t %.8f\t %.8f\t %.8f\t %.8f\n", tSim[i], eSim[i], yPISim[i], yPILib[i], yPILib[i] - yPISim[i]);
			err[i] = yPILib[i] - yPISim[i];
		}
		pid_ParaGet_T(1, &Kr_Read, &Tn_Read, NULL, &Tf_Read, &TSample_Read );
		pid_ParaGet_K(1, &Kp_Read, &Ki_Read, NULL, NULL, NULL );
		printf("Kr = %.2f (should be: %.2f)\n", Kr_Read, Kp);
		printf("Kp = %.2f (should be: %.2f)\n", Kp_Read, Kp);
		printf("Ki = %.2f (should be: %.2f)\n", Ki_Read, Ki);
		printf("Kr = %.2f (should be: %.2f)\n", Kr_Read, Kp);
		printf("Tn = %.2f (should be: %.2f)\n", Tn_Read, Kp/Ki);
		printf("TSample = %.2f (should be: %.2f)\n", TSample_Read, TSample);

		break;
	case 3:
		for (i=0; i < DataSets; i++) {
			printf("%.8f\t %.8f\t %.8f\t %.8f\t %.8f\n", tSim[i], eSim[i], yPIDSim[i], yPIDLib[i], yPIDLib[i] - yPIDSim[i]);
			err[i] = yPIDLib[i] - yPIDSim[i];
		}
		pid_ParaGet_T(2, &Kr_Read, &Tn_Read, &Tv_Read, &Tf_Read, &TSample_Read );
		pid_ParaGet_K(2, &Kp_Read, &Ki_Read, &Kd_Read, NULL, NULL );
		printf("Kr = %.2f (should be: %.2f)\n", Kr_Read, Kp);
		printf("Kp = %.2f (should be: %.2f)\n", Kp_Read, Kp);
		printf("Ki = %.2f (should be: %.2f)\n", Ki_Read, Ki);
		printf("Kd = %.2f (should be: %.2f)\n", Kd_Read, Kd);
		printf("Kr = %.2f (should be: %.2f)\n", Kr_Read, Kp);
		printf("Tn = %.2f (should be: %.2f)\n", Tn_Read, Kp/Ki);
		printf("Tv = %.2f (should be: %.2f)\n", Tv_Read, Kd/Kp);
		printf("Tf = %.2f (should be: %.2f)\n", Tf_Read, Tf);
		printf("TSample = %.2f (should be: %.2f)\n", TSample_Read, TSample);
		break;
	}
#endif

	/* Test if the squared error sum is less than a chosen threshold 
	 * We can definitely use double/float values because this test is 
	 * for travis, not for a microcontroller */
	double squared_err_sum = (PIDValue)0.0;
	double sample_variance;
	for (i=0; i < DataSets; i++) {
#ifdef PID_FIXPOINT
	d = ((double)err[i]) / PID_FIXPOINT_FACTOR;
	squared_err_sum +=  d * d;
#else
	squared_err_sum += err[i] * err[i];
#endif
	}
	sample_variance = squared_err_sum / DataSets;
	
	/* Free memory */
	free(tSim);
	free(eSim);
	free(yPSim);
	free(yPISim);
	free(yPIDSim);
	free(yPLib);
	free(yPILib);
	free(yPIDLib);
	free(eLib);
	free(err);
	
	printf("squared error sum = %e\n", squared_err_sum);
	printf("sample variance = squared error sum / number of samples = %e\n", sample_variance);
	printf("threshold = %e", (double)sample_var_thresh);
	if (sample_variance <= sample_var_thresh) {
		puts("==> Test successful!\n");
		return 0;
	}
	else {
		puts("==> Test failed!\n");
		return 1;
	}
}

