/*********************************************************************
* File: pidcontrol.h
*
* Declaration of data types and functions for the PID controller
* library. Include this header file in your code file to make use
* of the library.
*
* Use the file pidconfig.h for configuration options like number of 
* controllers and value types.
* 
* The library provides the following functions:
*
* pid_Init			-> Initialize the library
* pid_ParaSet_T		-> Sets controller parameters in the form Kr,Tn,Tv
* pid_ParaSet_K		-> Sets controller parameters in the form Kp,Ki,Kd
* pid_LimitsSet		-> Sets boundary values
* pid_ArwSet		-> Sets Anti-Windup
* pid_Step			-> Performs one step of calculations
* pid_IPartSet		-> Sets the value of the I-part to a certain value
* pid_Reset         -> Resets the controller (for restarting it)
* pid_PartsGet		-> Returns the current P, I and D part separately
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
#ifndef PIDCONTROL_H
#define PIDCONTROL_H

#include "piddefs.h"

/* Data type for accessing the controllers via an index */
typedef unsigned char PIDInd;


/* Possible error codes returned by the functions */
typedef enum
{
	pidErr_Ok,			/* Function completed successfully */
	pidErr_Index,		/* Passed index >= PID_NUM_CONTROLLERS */
	pidErr_TSample,		/* Passed sample time <= 0 */
	pidErr_Tn,			/* Passed value for Tn <= 0 */
	pidErr_Tv,			/* Passed value for Tv <= 0 */
	pidErr_Tf			/* Passed value for Tf < sample time */
} PIDErr;


/* Defines wether to use anti windup or not */
typedef enum
{
	pidArw_Off,
	pidArw_On
} PIDArw;



/* Initializes the library. This function has to be called once before any other function is used!
   Please keep in mind that all controllers are initialized with all numeric parameters equal 
   to zero except the sample time which is set to 1 and the lower and upper limits which
   are set to the lowest/ largest possible value for the chosen value type. 
   Anti-windup is disabled and trapezoidal integration is used.
*/
void   pid_Init( void );



/* Sets the parameters of the controller in time constant form
  
   id	-> Index of the controller to be accessed
   Kr	-> Controller gain
   Tn	-> Reset time integral part (set to 0 to deactivate integral part)
   Tv	-> Derivative time (set to 0 to deactivate differential part)
   Tf	-> Time constant filter for derivative  (set to 0 to deactivate filtering)
   TSample -> The sample time
  
   Please note that the boundary values have to be set separately using pid_LimitsSet!
*/
PIDErr pid_ParaSet_T( PIDInd id, PIDValue Kr, PIDValue Tn, PIDValue Tv, PIDValue Tf, PIDValue TSample );



/* Sets the parameters of the controller in gain form
  
   id	-> Index of the controller to be accessed
   Kp	-> Gain proportional part
   Ki	-> Gain integral part
   Kd	-> Gain differential part
   Tf	-> Time constant filter for derivative  (set to 0 to deactivate filtering)
   TSample -> The sample time
  
   Please note that the boundary values have to be set separately using pid_LimitsSet!
*/
PIDErr pid_ParaSet_K( PIDInd id, PIDValue Kp, PIDValue Ki, PIDValue Kd, PIDValue Tf, PIDValue TSample );



/* Gets the parameters of the controller in time constant form
  
   id	-> Index of the controller to be accessed
   Kr	-> Address to which controller gain is written
   Tn	-> Address to which reset time integral part is written (value is 0 for deactivated integral part)
   Tv	-> Address to which derivative time is written (value is 0 for deactivated derivative part)
   Tf	-> Address to which time constant filter for derivative is written (value is 0 for deactivated filter)
   TSample -> Address to which sample time is written
  
   If only a subset of these parameters is required set the pointers of the values which are not
   required to NULL
*/
PIDErr pid_ParaGet_T( PIDInd id, PIDValue* Kr, PIDValue* Tn, PIDValue* Tv, PIDValue* Tf, PIDValue* TSample );



/* Gets the parameters of the controller in gain form
  
   id	-> Index of the controller to be accessed
   Kp	-> Address to which gain proportional part is written
   Ki	-> Address to which gain integral part is written
   Kd	-> Address to which gain differential part is written
   Tf	-> Address to which time constant filter for derivative is written (value is 0 for deactivated filter)
   TSample -> Address to which the sample time is written
  
   If only a subset of these parameters is required set the pointers of the values which are not
   required to NULL
*/
PIDErr pid_ParaGet_K( PIDInd id, PIDValue* Kp, PIDValue* Ki, PIDValue* Kd, PIDValue* Tf, PIDValue* TSample );


/* Sets the lower and upper boundary of the controller output
 
   id	-> Index of the controller to be accessed
   yMin -> The lower boundary
   yMax -> The upper boundary

   Use pid_ArwSet to avoid winding up of the integral part
 */
PIDErr pid_LimitsSet( PIDInd id, PIDValue yMin, PIDValue yMax );



/* Enables/ disbales the anti-windup mechanism
 
   id	-> Index of the controller to be accessed
   Arw  -> Set this to pidArw_On to enable anti windup and to pidArw_Off to
           disable it

   Anti windup is implemented in such a way that integration stops as far as 
   the output of the controller is in its lower or upper limit.
*/
PIDErr pid_ArwSet( PIDInd id, PIDArw Arw );



/* Performs one step of calculation. This function has to be called
   cyclically at the sample time steps by your application.

   id	-> Index of the controller to be accessed
   e	-> The actual control difference
   *y   -> Address to which the controller output is written
*/
PIDErr pid_Step( PIDInd id, PIDValue e, PIDValue* y );



/* Sets the I-part of the controller to a new value

   id	-> Index of the controller to be accessed
   I	-> The new value for the I-part
*/
PIDErr pid_IPartSet( PIDInd id, PIDValue I );



/* Resets the controller, i.e. the stored P, I, and D part
   as well as the history of the control difference 
   and the output are set to 0. Limits, parameters, anti-windup
   mode are not changed.

   id	-> Index of the controller to be accessed
*/
PIDErr pid_Reset( PIDInd id );


/* Returns the current values of the P, the I, and the D part
   
   id	-> Index of the controller to be accessed
   P	-> Adress to which the current value of the P-part is written
   I	-> Adress to which the current value of the I-part is written
   D	-> Adress to which the current value of the D-part is written
 */
PIDErr pid_PartsGet( PIDInd id, PIDValue* P, PIDValue* I, PIDValue* D );

#endif

