/*********************************************************************
* File: pidconfig.h
*
* Configuration file for the PID-controller library
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
#ifndef PID_CONFIG_H
#define PID_CONFIG_H


/* PID_NUM_CONTROLLERS:
   Define the number of controllers you want to use.
   In the functions of this library the first controller is accessed
   by 0, the second by 1, the third by 2, ...
*/
#define PID_NUM_CONTROLLERS 3


/* PID_INTALGO_XYZ:
   Define which integration algorithm is used for the I-Part of the
   controller. 
   You have the following options:
   PID_INTALGO_RECT		-> Rectangular approximation
   PID_INTALGO_TRAPZ	-> Trapezoidal approximation
*/
#define PID_INTALGO_TRAPZ



/* PID_VAL_FORMAT_XYZ:
   Define the value type used for the control difference, the 
   controller output, the parameters and the sample time
   You have the following options:
   PID_VAL_FORMAT_I8   -> signed 8bit integer (fixpoint arithmetic) 
   PID_VAL_FORMAT_I16  -> signed 16bit integer (fixpoint arithmetic) 
   PID_VAL_FORMAT_I32  -> signed 32bit integer (fixpoint arithmetic) 
   PID_VAL_FORMAT_I64  -> signed 64bit integer  (fixpoint arithmetic)
   PID_VAL_FORMAT_F32  -> 32bit floating point  (float, floating point arithmetic)
   PID_VAL_FORMAT_F64  -> 64bit floating point (double,floating point arithmetic)

   Please note that when choosing a value type which results in fixpoint
   arithmetic you also have to define the number of positions after the 
   decimal point which are considered in the integer number. This is done
   by setting the macro PID_INTEGER_PRECISION
*/
#define PID_VAL_FORMAT_F32


/* PID_INTEGER_PRECISION:
   This definition is only used when the library uses fixpoint arithmetic,
   i.e., if PID_VAL_FORMAT_I8, PID_VAL_FORMAT_I16, PID_VAL_FORMAT_I32, or
   PID_VAL_FORMAT_I64 is set. It defines the number of digits in the integer 
   numbers for the controller gains Kr, Kp, Ki, Kd, the control error e,
   the controller output y and the controller limits which are assumed 
   to be after the decimal point in floating point notation. 
   Setting PID_INTEGER_PRECISION to n means that n digits 
   after the decimal point are considered.

   Example: 
   #define PID_INTEGER_PRECISION 3
   ==> x = 14321 is a representation of 14.321

   #define PID_INTEGER_PRECISION 0
   ==> x = 14321 is a representation of 14321.0

   Please note that this holds only for the control difference, the controller
   output, the limits, and the controller gains Kr, Kp, Ki, and Kd. All
   time parameters (i.e. Tn, Tv, Tf, and TSample) are always treated without
   having digits after the decimal point. Hence, if you have TSample = 0.025s
   you have to pass it in ms as TSample = 25 and scale your gains appropriately.

   Example: 
   Unit of control error -> Newton
   Unit of controller output -> Volt
   Floating point world: Sample time 0.025s, Ki = 12.5 V/(N s)
   Fixpoint world: #define PID_INTEGER_PRECISION 3

   Then you have to assume TSample as 25 ms  and Ki as 0.125 V/(N ms) and
   you pass TSample = 25 and Ki = 125.
*/
#define PID_INTEGER_PRECISION 4


/* PID_INDEX_BOUND_CHECK:
   Undef this if you do not want to do any error checking when
   entering the function (saves computation time but parameters
   and indices are processed without any checking)
*/
#define PID_INDEX_BOUND_CHECK


/* PID_USE_OWN_STDINT:
   Define the following macro if your compiler does not ship the stdint.h
   file with integer type definitions according to the C99 standard. For
   Microsoft Visual Studio 2008 and lower this is done automatically by
   the library.
*/
/* #define PID_USE_OWN_STDINT */

#endif
 