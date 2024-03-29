===================================================================
                            PIDControl
          A floatingpoint/ fixpoint library providing a
            simple PID controller written in pure C
          for Personal Computers and Microcontrollers

Copyright (c) 2014 Jan Winkler, Matthias Schäfer, Oscar Rivera
Institut für Regelungs- und Steuerungstheorie
Technische Universität Dresden / Dresden University of Technology
D-01062 Dresden, Germany

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the conditions listed
at the end of this document are met!
==================================================================

This library implements the widely used PID control laws

y(t) = Kp*e(t) + Ki*∫e(t)dt + Kd*de(t)/dt 

or

y(t) = Kr*( e(t) + (1/Tn)*∫e(t)dt + Tv*de(t)/dt  )

with the control difference e, the controller output y and the
parameters Kp, Ki, Kd or Kr, Tn, Tv, respectively.

1.Features:
===========
- Up to 255 instances of P, PI, PD, or PID controllers usable
  (depending on the available memory of your hardware)
- Each controller can be run with its own sample time
- Differential part with optional filtering
- Limits of output freely adjustable
- Optional anti-windup
- Initial condition for I part of the controller adjustable
- I part of the controller freely adjustable during runtime
- Different integration algorithms for the integral part selectable
- Parameters can be passed as gains (Kp, Ki, Kd) or as time
  constants (Kr, Tn, Tv)
- Realization based on floating-point arithmetic using 32 or 64bit
  floating point numbers or based on fix-point arithmetic using
  8, 16, 32, 64, or 128bit integers.


2.Compilation:
==============
For using and compiling the library in your project follow these
steps:
1. Add the file pidcontrol.c for compilation to your project

2. Make sure that the header files pidcontrol.h, pidconfig.h, piddefs.h,
   and pidinttypes.h are in the search path of your compiler

3. Open the file pidconfig.h and adjust the 6 preprocessor definitions
   according to your needs. By this, you can adjust
   - the number of PID controllers you want to use in your project
   - the data type you want to use (float, double, integer, ...)
   - if you use fixpoint-arithmetic: the precision represented by the
     integers
   - the integration algorithm used (rectangular, trapezoidal)
   - the level of runtime error checking
   - the use of the standard integer C99 header

   More details can be found in the header file pidconfig.h

4. In the file pidcontrol.h you find the functions provided by the
   library.


3.Verifying:
=============
You can verify the library in your environment by compiling the file
pidverify.c. When linking it together with the library file pidcontrol.c
a test program is created which compares the results of the library to
results produced by a PID controller in Matlab. For this purpose the
contents of the file PIDControlTestData.txt are read in by the produced
executable and the results are displayed when running it. 
You can check the fixpoint and the floating point version of the library
depending on the selections made in the file pidconfig.h

Verifcation under Windows using MS Visual Studio:
-------------------------------------------------
Depending on your version of Visual Studio open the project file
PIDControl_2008.sln, PIDControl_2010.sln, or PIDControl_2013.sln.
The program is built as PIDControl.exe into the directory 
build/debug or build/release depending on the configuration 
selected in Visual Studio

Verification under Windows using the MINGW gcc compiler:
--------------------------------------------------------
Open a console and change into the base directory of the library
where the file Makefile.windows resides. Then type

mingw32-make -fMakefile.windows all

The program is built as pidverify.exe into the directory 
build/debug

Verification under Linux:
-------------------------
Open a terminal and change into the base directory of the library
where the file Makefile.linux resides. Then type

make -fMakefile.linux all

and the executable is built into the directory build/debug.



4. Copyright/ License:
======================
Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions 
are met:

    Redistributions of source code must retain the above copyright 
    notice, this list of conditions and the following disclaimer. 

    Redistributions in binary form must not misrepresent the orignal
    source in the documentation and/or other materials provided 
    with the distribution. 

    The names of the authors nor its contributors may be used to 
    endorse or promote products derived from this software without 
    specific prior written permission. 

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES 
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) 
HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED 
OF THE POSSIBILITY OF SUCH DAMAGE.
