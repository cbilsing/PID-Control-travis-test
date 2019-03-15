/*********************************************************************
* File: piddefs.h
*
* Some global definitions depending on the configuration header
* pidconfig.h
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
#ifndef PID_VALUES_H
#define PID_VALUES_H

#include <float.h>
#include "pidconfig.h"

#ifdef _MSC_VER
	#if _MSC_VER < 1600 && !(defined PID_USE_OWN_STDINT)
	#define PID_USE_OWN_STDINT
	#endif
#endif

#ifdef PID_USE_OWN_STDINT
	#include "pidstdint.h"
#else
	#include <stdint.h>
#endif


#if !(defined PID_INTALGO_TRAPZ) && !(defined PID_INTALGO_RECT)
	#error "No integration algorithm (PID_INTALGO_TRAPZ, PID_INTALGO_RECT) specified!"
#endif


#if (defined PID_VAL_FORMAT_I8)
	typedef int8_t      PIDValue;	/* signed fixcomma 8bit */
#elif (defined PID_VAL_FORMAT_I16)
	typedef int16_t      PIDValue;	/* signed fixcomma 16bit */
#elif (defined PID_VAL_FORMAT_I32)
	typedef int32_t      PIDValue;	/* signed fixcomma 32bit */
#elif (defined PID_VAL_FORMAT_I64)
	typedef int64_t      PIDValue;	/* signed fixcomma 64bit */
#elif (defined PID_VAL_FORMAT_F32)
	typedef float        PIDValue;  /* floating point 32bit */
#elif (defined PID_VAL_FORMAT_F64)
	typedef double       PIDValue;  /* floating point 32bit */
#else
	#error "No value format (PID_VAL_FORMAT_I32, PID_VAL_FORMAT_F32, ...) specified!"
#endif


#if !(defined PID_VAL_FORMAT_F32) && !(defined PID_VAL_FORMAT_F64)
	#define PID_FIXPOINT
#endif


#ifdef PID_FIXPOINT

	#if (defined PID_VAL_FORMAT_I8) && (PID_INTEGER_PRECISION > 2)
		#error "A resolution > 2 digits after the decimal point is not possible when using 8bit integers!"
	#endif

	#if (defined PID_VAL_FORMAT_I16) && (PID_INTEGER_PRECISION > 4)
		#error "A resolution > 4 digits after the decimal point is not possible when using 16bit integers!"
	#endif

	#if (defined PID_VAL_FORMAT_I32) && (PID_INTEGER_PRECISION > 9)
		#error "A resolution > 9 digits after the decimal point is not possible when using 32bit integers!"
	#endif

	#if PID_INTEGER_PRECISION == 0
		#define PID_FIXPOINT_FACTOR 1
	#elif PID_INTEGER_PRECISION == 1
		#define PID_FIXPOINT_FACTOR 10
	#elif PID_INTEGER_PRECISION == 2
		#define PID_FIXPOINT_FACTOR 100
	#elif PID_INTEGER_PRECISION == 3
		#define PID_FIXPOINT_FACTOR 1000
	#elif PID_INTEGER_PRECISION == 4
		#define PID_FIXPOINT_FACTOR 10000
	#elif PID_INTEGER_PRECISION == 5
		#define PID_FIXPOINT_FACTOR 100000
	#elif PID_INTEGER_PRECISION == 6
		#define PID_FIXPOINT_FACTOR 1000000
	#elif PID_INTEGER_PRECISION == 7
		#define PID_FIXPOINT_FACTOR 10000000
	#elif PID_INTEGER_PRECISION == 8
		#define PID_FIXPOINT_FACTOR 100000000
	#elif PID_INTEGER_PRECISION == 9
		#define PID_FIXPOINT_FACTOR 1000000000
	#elif PID_INTEGER_PRECISION == 10
		#define PID_FIXPOINT_FACTOR 10000000000
	#elif PID_INTEGER_PRECISION == 11
		#define PID_FIXPOINT_FACTOR 100000000000
	#elif PID_INTEGER_PRECISION == 12
		#define PID_FIXPOINT_FACTOR 1000000000000
	#elif PID_INTEGER_PRECISION == 13
		#define PID_FIXPOINT_FACTOR 10000000000000
	#elif PID_INTEGER_PRECISION == 14
		#define PID_FIXPOINT_FACTOR 100000000000000
	#elif PID_INTEGER_PRECISION == 15
		#define PID_FIXPOINT_FACTOR 1000000000000000
	#elif PID_INTEGER_PRECISION == 16
		#define PID_FIXPOINT_FACTOR 10000000000000000
	#else
		#error "Only time resolutions up to 1/(10^16) are supported!"
	#endif  

	#define PID_FIXPOINT_CORR_MUL /PID_FIXPOINT_FACTOR
	#define PID_FIXPOINT_CORR_DIV *PID_FIXPOINT_FACTOR

#else  /* Floating point calculation */
	#define PID_FIXPOINT_CORR_MUL
	#define PID_FIXPOINT_CORR_DIV
#endif

#endif
