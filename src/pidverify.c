#include <stdio.h>
#include <malloc.h>
#include <string.h>
#include "pidcontrol.h"

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
	PIDValue  eLib;
	PIDValue Kp_Read, Ki_Read, Kd_Read, Tf_Read, TSample_Read, Kr_Read, Tn_Read, Tv_Read;

#ifndef PID_FIXPOINT
	/* For the floating point test we assume the following parameters:
	   Kp = 2, Ki = 0.5*1/s, Kd = 2s, TSample = 0.5s, Tf = 2 s
     */
	PIDValue Kp      = 2.0;
	PIDValue Ki      = 0.5;
	PIDValue Kd      = 2;
	PIDValue Tf      = 2;
	PIDValue TSample = 0.5;
	puts("Compiled withOUT PID_FIXPOINT enabled");
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
	PIDValue Ki      = (PIDValue)(0.05*PID_FIXPOINT_FACTOR); //????? diese stelle sieht gefährlich aus
	PIDValue Kd      = 20*PID_FIXPOINT_FACTOR;
	PIDValue Tf      = 20;
	PIDValue TSample = 5;
	
	puts("Compiled with PID_FIXPOINT enabled");
#endif

	char	line[256];

	FILE*			datafile;
	int				cnt;

	int	choice = 0;

	/* Open the file with the reference test-data */
	datafile = fopen("PIDControlTestData.txt", "r");
	if (datafile == NULL) 
	{
		printf("Couldn't open datafile for reading\n");
		return 0;
	}

	/* Determine the number of datasets (=rows) provided by the test-data */
	memset(line, 0, sizeof(line));
	while (	fgets(line, sizeof(line), datafile) != NULL ) DataSets++;

	/* Allocate corresponding memory for the data to import */
	tSim    = malloc(DataSets*sizeof(double));
	eSim    = malloc(DataSets*sizeof(double));
	yPSim   = malloc(DataSets*sizeof(double));
	yPISim  = malloc(DataSets*sizeof(double));
	yPIDSim = malloc(DataSets*sizeof(double));
	yPLib   = malloc(DataSets*sizeof(double));
	yPILib  = malloc(DataSets*sizeof(double));
	yPIDLib = malloc(DataSets*sizeof(double));

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
		eLib = (PIDValue)(eSim[i]*PID_FIXPOINT_FACTOR);
		printf("%d \t", eLib);
#else
		eLib = (PIDValue)eSim[i];
#endif
		pid_Step(0, eLib, &yPLib[i]);
		pid_Step(1, eLib, &yPILib[i]);
		pid_Step(2, eLib, &yPIDLib[i]);
	}

	printf("Size of int8_t: %d Byte\n", sizeof(int8_t));
	printf("Size of uint8_t: %d Byte\n", sizeof(uint8_t));
	printf("Size of int16_t: %d Byte\n", sizeof(int16_t));
	printf("Size of uint16_t: %d Byte\n", sizeof(uint16_t));
	printf("Size of int32_t: %d Byte\n", sizeof(int32_t));
	printf("Size of uint32_t: %d Byte\n", sizeof(uint32_t));
	printf("Size of int64_t: %d Byte\n", sizeof(int64_t));
	printf("Size of uint64_t: %d Byte\n", sizeof(uint64_t));
	printf("Size of float: %d Byte\n", sizeof(float));
	printf("Size of double: %d Byte\n\n", sizeof(double));

	printf("Which controller would you like to compare?\nEnter one of the following nummbers:\n");
	printf("1 -> P-controller\n");
	printf("2 -> PI-controller\n");
	printf("3 -> PID-controller\n");
	printf("\nYour choice: ");
	cnt = scanf("%d", &choice);

	if (cnt <= 0)
	{
		printf("Nothing read (scanf return-code: %d)", cnt);
		return -1;
	}

	printf("\n\nt \t\t e \t\t Sim \t\t Lib \t\t Err \n");
#ifdef PID_FIXPOINT
	switch (choice)
	{
	case 1:
		for (i=0; i < DataSets; i++)
			printf("%.8f\t %.8f\t %.8f\t %.8f\t %.8f\n", tSim[i], eSim[i], yPSim[i], ((double)yPLib[i])/PID_FIXPOINT_FACTOR, ((double)yPLib[i])/PID_FIXPOINT_FACTOR - yPSim[i]);
		break;
	case 2:
		for (i=0; i < DataSets; i++)
			printf("%.8f\t %.8f\t %.8f\t %.8f\t %.8f\n", tSim[i], eSim[i], yPISim[i], ((double)yPILib[i])/PID_FIXPOINT_FACTOR, ((double)yPILib[i])/PID_FIXPOINT_FACTOR - yPISim[i]);
		break;
	case 3:
		for (i=0; i < DataSets; i++)
			printf("%.8f\t %.8f\t %.8f\t %.8f\t %.8f\n", tSim[i], eSim[i], yPIDSim[i], ((double)yPIDLib[i])/PID_FIXPOINT_FACTOR, ((double)yPIDLib[i])/PID_FIXPOINT_FACTOR - yPIDSim[i]);
		break;
	}
#else
	switch (choice)
	{
	case 1:
		for (i=0; i < DataSets; i++)
			printf("%.8f\t %.8f\t %.8f\t %.8f\t %.8f\n", tSim[i], eSim[i], yPSim[i], yPLib[i], yPLib[i] - yPSim[i]);
		pid_ParaGet_T(0, &Kr_Read, NULL, NULL, &Tf_Read, &TSample_Read );
		pid_ParaGet_K(0, &Kp_Read, NULL, NULL, NULL, NULL );
		printf("Kr = %.2f (should be: %.2f)\n", Kr_Read, Kp);
		printf("Kp = %.2f (should be: %.2f)\n", Kp_Read, Kp);
		printf("Kr = %.2f (should be: %.2f)\n", Kr_Read, Kp);
		printf("TSample = %.2f (should be: %.2f)\n", TSample_Read, TSample);
		break;
		
	case 2:
		for (i=0; i < DataSets; i++)
			printf("%.8f\t %.8f\t %.8f\t %.8f\t %.8f\n", tSim[i], eSim[i], yPISim[i], yPILib[i], yPILib[i] - yPISim[i]);
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
		for (i=0; i < DataSets; i++)
			printf("%.8f\t %.8f\t %.8f\t %.8f\t %.8f\n", tSim[i], eSim[i], yPIDSim[i], yPIDLib[i], yPIDLib[i] - yPIDSim[i]);
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

	/* Free memory */
	free(tSim);
	free(eSim);
	free(yPSim);
	free(yPISim);
	free(yPIDSim);
	free(yPLib);
	free(yPILib);
	free(yPIDLib);

	return 0;
}

