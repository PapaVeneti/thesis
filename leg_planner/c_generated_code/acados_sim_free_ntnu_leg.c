/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

// system
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
// acados
#include "acados/sim/sim_common.h"
#include "acados_c/sim_interface.h"
// example specific
#include "acados_sim_solver_ntnu_leg.h"
// mex
#include "mex.h"



void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{

//    mexPrintf("\nin sim_destroy\n");

//    void *config = mxGetPr( mxGetField( prhs[0], 0, "config" ) );
//    long long *config_mat = (long long *) mxGetData( mxGetField( prhs[0], 0, "config" ) );
//    long long config = (long long) mxGetScalar( mxGetField( prhs[0], 0, "config" ) );

    /* RHS */
    const mxArray *C_sim = prhs[0];
    long long * ptr;

    // capsule
    ptr = (long long *) mxGetData( mxGetField( C_sim, 0, "capsule" ) );
    ntnu_leg_sim_solver_capsule *capsule = (ntnu_leg_sim_solver_capsule *) ptr[0];


    /* free memory */
    int status = 0;

    status = ntnu_leg_acados_sim_free(capsule);
    if (status)
    {
        mexPrintf("ntnu_leg_acados_sim_free() returned status %d.\n", status);
    }

    status = ntnu_leg_acados_sim_solver_free_capsule(capsule);
    if (status)
    {
        mexPrintf("ntnu_leg_acados_sim_solver_free_capsule() returned status %d.\n", status);
    }

    return;

}
