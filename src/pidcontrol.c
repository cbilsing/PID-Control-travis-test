/*********************************************************************
* File: pidcontrol.c
*
* Implementation of the PID controller library. Compile this file
* in your project in order use the library.
*
* Use the file pidconfig.h for configuration options
* like number of controllers and value types.
*
* Refer to the header vor more information
*
*
* Copyright (c) 2014 Jan Winkler, Matthias Schäfer, Oscar Rivera
* Institut für Regelungs- und Steuerungstheorie
* Technische Universität Dresden / Dresden University of Technology
* D-01062 Dresden, Germany
*
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions 
* are met:
*
*     Redistributions of source code must retain the above copyright 
*     notice, this list of conditions and the following disclaimer. 
*
*     Redistributions in binary form must not misrepresent the orignal
*     source in the documentation and/or other materials provided 
*     with the distribution. 
*
*     The names of the authors nor its contributors may be used to 
*     endorse or promote products derived from this software without 
*     specific prior written permission. 
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
* HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED 
* OF THE POSSIBILITY OF SUCH DAMAGE.
* 
* Jan.Winkler@tu-dresden.de
* 04.06.2014
*********************************************************************/
#include "pidcontrol.h"


/* Structure defining the parameters and state of a PID controller */
typedef struct PIDControllerStruct
{
	/* The coefficients for the difference equations resulting 
	   from the controller parameters 
	*/
	PIDValue Cp;	/* Proportional (Kp)*/
	PIDValue Ci;	/* Integration rectengular approx. (Ki*TSample) or (Ki*TSample/2)*/
	PIDValue Cd;	/* Differentiation without filtering (Kd/TSample)*/
	PIDValue Cdf;	/* Differentiation with filtering, diff. part (Kd/Tf)*/
	PIDValue Cf;	/* Differentiation with filtering, filt. part (1-Ta/Tf)*/
	
	/* The boundary values */ 
	PIDValue yMax;
	PIDValue yMin;

	/* The sample time */
	PIDValue TSample;

	/* History of input, output and filtered input
	   The neweset value can be found at position 0,
	   the oldest at position 1
    */
	PIDValue e[2];		/* Control difference */
	PIDValue y[2];		/* Controller output */

	/* Current values of the P, I, and D part */
	PIDValue P;
	PIDValue I;
	PIDValue D;
	
	/* Anti-Windup enabled */
	PIDArw     Arw;

} PIDController;


/* The global array of PID controllers managed by the running
   instance of the library 
*/
PIDController PID[PID_NUM_CONTROLLERS];




/* Initialization of the library */
void pid_Init( void )
{
	PIDInd i, j;

	for (i = 0; i < PID_NUM_CONTROLLERS; i++)
	{
		PID[i].Cp		= 0;
		PID[i].Ci		= 0;
		PID[i].Cd		= 0;
		PID[i].Cf		= 0;
		PID[i].TSample	= 1;
		PID[i].Arw		= pidArw_Off;
		PID[i].P		= 0;
		PID[i].I		= 0;
		PID[i].D		= 0;

		for (j = 0; j < 2; j++) 
		{
			PID[i].e[j] = 0;
			PID[i].y[j]	= 0;
		};

#if (defined PID_VAL_FORMAT_I8)
		PID[i].yMin		= INT8_MIN;
		PID[i].yMax		= INT8_MAX;
#elif (defined PID_VAL_FORMAT_I16)
		PID[i].yMin		= INT16_MIN;
		PID[i].yMax		= INT16_MAX;
#elif (defined PID_VAL_FORMAT_I32)
		PID[i].yMin		= INT32_MIN;
		PID[i].yMax		= INT32_MAX;
#elif (defined PID_VAL_FORMAT_I64)
		PID[i].yMin		= INT64_MIN;
		PID[i].yMax		= INT64_MAX;
#elif (defined PID_VAL_FORMAT_F32)
		PID[i].yMin		= -FLT_MAX;
		PID[i].yMax		= FLT_MAX;
#elif (defined PID_VAL_FORMAT_F64)
		PID[i].yMin		= -DBL_MAX;
		PID[i].yMax		= DBL_MAX;
#else
#error "No value format (PID_VAL_FORMAT_I32, PID_VAL_FORMAT_F32, ...) specified!"
#endif

	}
}



/* Sets parameters of the controller with the index id (time constant form)*/
PIDErr pid_ParaSet_T( PIDInd id, PIDValue Kr, PIDValue Tn, PIDValue Tv, PIDValue Tf, PIDValue TSample )
{

#ifdef PID_INDEX_BOUND_CHECK
	if ( id >= PID_NUM_CONTROLLERS )    return pidErr_Index;
	if ( (Tf != 0) && (Tf < TSample) )  return pidErr_Tf;
	if ( TSample <= 0)                  return pidErr_TSample;
	if ( Tn < 0 )						return pidErr_Tn;
	if ( Tv < 0 )						return pidErr_Tv;
#endif

	PID[id].TSample = TSample;

	/* Coefficients P-part */
	PID[id].Cp		= Kr;
	
	/* Coefficients I-part */
	if (Tn == 0)
	{
		PID[id].Ci = 0;
	}
	else
	{
#if (defined PID_INTALGO_RECT)
		PID[id].Ci	= (Kr*TSample)/Tn;
#elif (defined PID_INTALGO_TRAPZ)
		PID[id].Ci	= (Kr*TSample)/(2*Tn);
#else
	#error "No integration algorithm (PID_INTALGO_TRAPZ, PID_INTALGO_RECT) specified!"
#endif
	}

	/* Coefficients D-part */
	PID[id].Cd		= (Kr*Tv)/TSample;

	if ( Tf == 0 )
	{
		PID[id].Cf	= 0;
		PID[id].Cdf = 0;
	}
	else 
	{
#ifdef PID_FIXPOINT
		PID[id].Cf	= (PID_FIXPOINT_FACTOR*(Tf - TSample))/Tf;
#else
		PID[id].Cf	= 1 - TSample/Tf;
#endif
		PID[id].Cdf = (Kr*Tv)/Tf;
	};

	return pidErr_Ok;
}



/* Sets parameters of the controller with the index id (gain constant form)*/
PIDErr pid_ParaSet_K( PIDInd id, PIDValue Kp, PIDValue Ki, PIDValue Kd, PIDValue Tf, PIDValue TSample )
{
#ifdef PID_INDEX_BOUND_CHECK
	if ( id >= PID_NUM_CONTROLLERS )   return pidErr_Index;
	if ( (Tf != 0) && (Tf < TSample) ) return pidErr_Tf;
	if ( TSample <= 0)                 return pidErr_TSample;
#endif

	PID[id].TSample = TSample;
	
	/* Proportional part */
	PID[id].Cp		= Kp;

	/* Integration part */
#if (defined PID_INTALGO_RECT)
	PID[id].Ci		= Ki*TSample;
#elif (defined PID_INTALGO_TRAPZ)
	PID[id].Ci		= Ki*TSample/2;
#else
	#error "No integration algorithm (PID_INTALGO_TRAPZ, PID_INTALGO_RECT) specified!"
#endif

	/* Differential part */
	PID[id].Cd		= Kd/TSample;
	if ( Tf == 0 )
	{
		PID[id].Cf		= 0;
		PID[id].Cdf     = 0;
	}
	else 
	{
#ifdef PID_FIXPOINT  
		PID[id].Cf		= (PID_FIXPOINT_FACTOR*(Tf - TSample))/Tf;
#else
		PID[id].Cf		= 1-TSample/Tf;
#endif
		PID[id].Cdf     = Kd/Tf;
	};

	return pidErr_Ok;
}



/* Gets the parameter of controller with id in time constant form */
PIDErr pid_ParaGet_T( PIDInd id, PIDValue* Kr, PIDValue* Tn, PIDValue* Tv, PIDValue* Tf, PIDValue* TSample )
{
#ifdef PID_INDEX_BOUND_CHECK
	if ( id >= PID_NUM_CONTROLLERS )    return pidErr_Index;
#endif
	
	/* Sample time */
	if (TSample != 0) 
		*TSample = PID[id].TSample;

	/* Coefficients P-part */
	if (Kr != 0)
		*Kr = PID[id].Cp;
	
	/* Coefficients I-part */
	if (Tn != 0)
	{
		if (PID[id].Ci == 0)
		{
			*Tn = 0;
		}
		else
		{
		#if (defined PID_INTALGO_RECT)
			*Tn = (PID[id].Cp*PID[id].TSample)/PID[id].Ci;
		#elif (defined PID_INTALGO_TRAPZ)
			*Tn = (PID[id].Cp*PID[id].TSample)/(2*PID[id].Ci);
		#else
			#error "No integration algorithm (PID_INTALGO_TRAPZ, PID_INTALGO_RECT) specified!"
		#endif
		}
	}

	/* Coefficients D-part */
	if ( Tv != 0 )
	{
		*Tv = (PID[id].Cd*PID[id].TSample)/PID[id].Cp;
	}

	/* Filter */
	if ( Tf != 0 )
	{
		if ( PID[id].Cdf == 0 )
		{
			*Tf = 0;
		}
		else 
		{
			*Tf = (PID[id].Cd*PID[id].TSample)/PID[id].Cdf;
		};
	};

	return pidErr_Ok;
}



/* Gets the parameter of controller with id in gain form */
PIDErr pid_ParaGet_K( PIDInd id, PIDValue* Kp, PIDValue* Ki, PIDValue* Kd, PIDValue* Tf, PIDValue* TSample )
{
#ifdef PID_INDEX_BOUND_CHECK
	if ( id >= PID_NUM_CONTROLLERS )    return pidErr_Index;
#endif
	
	/* Sample time */
	if (TSample != 0) 
		*TSample = PID[id].TSample;

	/* Coefficients P-part */
	if (Kp != 0)
		*Kp = PID[id].Cp;
	
	/* Coefficients I-part */
	if (Ki != 0)
	{
	#if (defined PID_INTALGO_RECT)
		*Ki = PID[id].Ci/PID[id].TSample;
	#elif (defined PID_INTALGO_TRAPZ)
		*Ki = (2*PID[id].Ci)/PID[id].TSample;
	#else
		#error "No integration algorithm (PID_INTALGO_TRAPZ, PID_INTALGO_RECT) specified!"
	#endif
	}

	/* Coefficients D-part */
	if ( Kd != 0 )
	{
		*Kd = PID[id].Cd*PID[id].TSample;
	}

	/* Filter */
	if ( Tf != 0 )
	{
		if ( PID[id].Cdf == 0 )
		{
			*Tf = 0;
		}
		else 
		{
			*Tf = (PID[id].Cd*PID[id].TSample)/PID[id].Cdf;
		};
	};

	return pidErr_Ok;


}



/* Sets the output limit of controller with id */
PIDErr pid_LimitsSet( PIDInd id, PIDValue yMin, PIDValue yMax )
{
#ifdef PID_INDEX_BOUND_CHECK
	if ( id >= PID_NUM_CONTROLLERS ) return pidErr_Index;
#endif

	PID[id].yMin = yMin;
	PID[id].yMax = yMax;

	return pidErr_Ok;
}



PIDErr pid_ArwSet( PIDInd id, PIDArw Arw )
{
#ifdef PID_INDEX_BOUND_CHECK
	if ( id >= PID_NUM_CONTROLLERS ) return pidErr_Index;
#endif

	PID[id].Arw = Arw;

	return pidErr_Ok;
}





PIDErr pid_Step( PIDInd id, PIDValue e, PIDValue* y )
{
	PIDValue IOld = 0;

#ifdef PID_INDEX_BOUND_CHECK
	if ( id >= PID_NUM_CONTROLLERS ) return pidErr_Index;
#endif

	/* Shift the history of the inputs and outputs. The following holds:
	   e_k = e[0], e_{k-1} = e[1], e_{k-2} = e[2]
    */
	PID[id].e[1] = PID[id].e[0];
	PID[id].y[1] = PID[id].y[0];

	/* Assign the new control difference to the history */
	PID[id].e[0] = e;

	/* Proportional part */
	/*********************/
	PID[id].P = (PID[id].Cp*e) PID_FIXPOINT_CORR_MUL;


	/* Integral part */
	/*****************/
	
	/* Remember I part from last step if we have to drop the calculation
	   due to anti-windup
    */ 
	IOld = PID[id].I;

	/* Do the integration depending on the chosen integration algorithm */
#if (defined PID_INTALGO_RECT) /* rectengular approximation */
    PID[id].I += (PID[id].Ci*PID[id].e[1]) PID_FIXPOINT_CORR_MUL;
#elif (defined PID_INTALGO_TRAPZ) /* trapezoidal approximation */
	PID[id].I += (PID[id].Ci*(PID[id].e[0] + PID[id].e[1])) PID_FIXPOINT_CORR_MUL;
#else
	#error "No integration algorithm (PID_INTALGO_TRAPZ, PID_INTALGO_RECT) specified!"
#endif


	/* Differential part */
	/*********************/
	
	/* Calcultion without smoothing of the input */
	if ( PID[id].Cf == 0 )
	{
		PID[id].D = (PID[id].Cd*(PID[id].e[0] - PID[id].e[1])) PID_FIXPOINT_CORR_MUL;
	}
	else /* Calcultion with smoothing of the input */
	{
		/* Differentiation incl. low-pass filtering*/
		PID[id].D = (PID[id].Cdf*(PID[id].e[0] - PID[id].e[1]) + PID[id].Cf*PID[id].D) PID_FIXPOINT_CORR_MUL;
	}

	/* Overall control output */
	PID[id].y[0] = PID[id].P + PID[id].I + PID[id].D;

	/* Check if boundary values are violated */
	if (PID[id].y[0] > PID[id].yMax) PID[id].y[0] = PID[id].yMax;
	else if (PID[id].y[0] < PID[id].yMin) PID[id].y[0] = PID[id].yMin;

	/* If Anti-Windup is activated and output is on its boundary value drop
	   the last calculation of the I-Part
    */
	if ( (PID[id].Arw == pidArw_On) && 
		 ( (PID[id].y[0] == PID[id].yMax) || (PID[id].y[0] == PID[id].yMax) ) )
	{
		PID[id].I = IOld;
	}

	/* Assign the return value */
	*y = PID[id].y[0];

	return pidErr_Ok;
}
 


/* Sets the I-part of controller with index id to the value I */
PIDErr pid_IPartSet( PIDInd id, PIDValue I )
{
#ifdef PID_INDEX_BOUND_CHECK
	if ( id >= PID_NUM_CONTROLLERS ) return pidErr_Index;
#endif
	PID[id].I = I;

	return pidErr_Ok;
}


/* Resets the internal states of the controller */
PIDErr pid_Reset( PIDInd id )
{
	int j;
#ifdef PID_INDEX_BOUND_CHECK
	if ( id >= PID_NUM_CONTROLLERS ) return pidErr_Index;
#endif

	PID[id].P		= 0;
	PID[id].I		= 0;
	PID[id].D		= 0;

	for (j = 0; j < 2; j++) 
	{
		PID[id].e[j] = 0;
		PID[id].y[j] = 0;
	};
	
	return pidErr_Ok;
}



/* Returns the current values of the P, I, and D-part */
PIDErr pid_PartsGet( PIDInd id, PIDValue* P, PIDValue* I, PIDValue* D )
{
#ifdef PID_INDEX_BOUND_CHECK
	if ( id >= PID_NUM_CONTROLLERS ) return pidErr_Index;
#endif

	*P = PID[id].P;
	*I = PID[id].I;
	*D = PID[id].D;

	return pidErr_Ok;
}
