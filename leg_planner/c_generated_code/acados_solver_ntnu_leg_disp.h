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

#ifndef ACADOS_SOLVER_ntnu_leg_disp_H_
#define ACADOS_SOLVER_ntnu_leg_disp_H_

#include "acados/utils/types.h"

#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

#define NTNU_LEG_DISP_NX     10
#define NTNU_LEG_DISP_NZ     0
#define NTNU_LEG_DISP_NU     3
#define NTNU_LEG_DISP_NP     0
#define NTNU_LEG_DISP_NBX    10
#define NTNU_LEG_DISP_NBX0   10
#define NTNU_LEG_DISP_NBU    3
#define NTNU_LEG_DISP_NSBX   0
#define NTNU_LEG_DISP_NSBU   0
#define NTNU_LEG_DISP_NSH    2
#define NTNU_LEG_DISP_NSG    0
#define NTNU_LEG_DISP_NSPHI  0
#define NTNU_LEG_DISP_NSHN   2
#define NTNU_LEG_DISP_NSGN   0
#define NTNU_LEG_DISP_NSPHIN 0
#define NTNU_LEG_DISP_NSBXN  0
#define NTNU_LEG_DISP_NS     2
#define NTNU_LEG_DISP_NSN    2
#define NTNU_LEG_DISP_NG     2
#define NTNU_LEG_DISP_NBXN   0
#define NTNU_LEG_DISP_NGN    0
#define NTNU_LEG_DISP_NY0    17
#define NTNU_LEG_DISP_NY     17
#define NTNU_LEG_DISP_NYN    10
#define NTNU_LEG_DISP_N      150
#define NTNU_LEG_DISP_NH     2
#define NTNU_LEG_DISP_NPHI   0
#define NTNU_LEG_DISP_NHN    2
#define NTNU_LEG_DISP_NPHIN  0
#define NTNU_LEG_DISP_NR     0

#ifdef __cplusplus
extern "C" {
#endif


// ** capsule for solver data **
typedef struct ntnu_leg_disp_solver_capsule
{
    // acados objects
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;
    ocp_nlp_out *sens_out;
    ocp_nlp_solver *nlp_solver;
    void *nlp_opts;
    ocp_nlp_plan_t *nlp_solver_plan;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;

    // number of expected runtime parameters
    unsigned int nlp_np;

    /* external functions */
    // dynamics

    external_function_param_casadi *forw_vde_casadi;
    external_function_param_casadi *expl_ode_fun;




    // cost

    external_function_param_casadi *cost_y_fun;
    external_function_param_casadi *cost_y_fun_jac_ut_xt;
    external_function_param_casadi *cost_y_hess;



    external_function_param_casadi cost_y_0_fun;
    external_function_param_casadi cost_y_0_fun_jac_ut_xt;
    external_function_param_casadi cost_y_0_hess;




    // constraints
    external_function_param_casadi *nl_constr_h_fun_jac;
    external_function_param_casadi *nl_constr_h_fun;



    external_function_param_casadi nl_constr_h_e_fun_jac;
    external_function_param_casadi nl_constr_h_e_fun;

} ntnu_leg_disp_solver_capsule;

ACADOS_SYMBOL_EXPORT ntnu_leg_disp_solver_capsule * ntnu_leg_disp_acados_create_capsule(void);
ACADOS_SYMBOL_EXPORT int ntnu_leg_disp_acados_free_capsule(ntnu_leg_disp_solver_capsule *capsule);

ACADOS_SYMBOL_EXPORT int ntnu_leg_disp_acados_create(ntnu_leg_disp_solver_capsule * capsule);

ACADOS_SYMBOL_EXPORT int ntnu_leg_disp_acados_reset(ntnu_leg_disp_solver_capsule* capsule, int reset_qp_solver_mem);

/**
 * Generic version of ntnu_leg_disp_acados_create which allows to use a different number of shooting intervals than
 * the number used for code generation. If new_time_steps=NULL and n_time_steps matches the number used for code
 * generation, the time-steps from code generation is used.
 */
ACADOS_SYMBOL_EXPORT int ntnu_leg_disp_acados_create_with_discretization(ntnu_leg_disp_solver_capsule * capsule, int n_time_steps, double* new_time_steps);
/**
 * Update the time step vector. Number N must be identical to the currently set number of shooting nodes in the
 * nlp_solver_plan. Returns 0 if no error occurred and a otherwise a value other than 0.
 */
ACADOS_SYMBOL_EXPORT int ntnu_leg_disp_acados_update_time_steps(ntnu_leg_disp_solver_capsule * capsule, int N, double* new_time_steps);
/**
 * This function is used for updating an already initialized solver with a different number of qp_cond_N.
 */
ACADOS_SYMBOL_EXPORT int ntnu_leg_disp_acados_update_qp_solver_cond_N(ntnu_leg_disp_solver_capsule * capsule, int qp_solver_cond_N);
ACADOS_SYMBOL_EXPORT int ntnu_leg_disp_acados_update_params(ntnu_leg_disp_solver_capsule * capsule, int stage, double *value, int np);
ACADOS_SYMBOL_EXPORT int ntnu_leg_disp_acados_update_params_sparse(ntnu_leg_disp_solver_capsule * capsule, int stage, int *idx, double *p, int n_update);

ACADOS_SYMBOL_EXPORT int ntnu_leg_disp_acados_solve(ntnu_leg_disp_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int ntnu_leg_disp_acados_free(ntnu_leg_disp_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void ntnu_leg_disp_acados_print_stats(ntnu_leg_disp_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int ntnu_leg_disp_acados_custom_update(ntnu_leg_disp_solver_capsule* capsule, double* data, int data_len);


ACADOS_SYMBOL_EXPORT ocp_nlp_in *ntnu_leg_disp_acados_get_nlp_in(ntnu_leg_disp_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *ntnu_leg_disp_acados_get_nlp_out(ntnu_leg_disp_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *ntnu_leg_disp_acados_get_sens_out(ntnu_leg_disp_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_solver *ntnu_leg_disp_acados_get_nlp_solver(ntnu_leg_disp_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_config *ntnu_leg_disp_acados_get_nlp_config(ntnu_leg_disp_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void *ntnu_leg_disp_acados_get_nlp_opts(ntnu_leg_disp_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_dims *ntnu_leg_disp_acados_get_nlp_dims(ntnu_leg_disp_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_plan_t *ntnu_leg_disp_acados_get_nlp_plan(ntnu_leg_disp_solver_capsule * capsule);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // ACADOS_SOLVER_ntnu_leg_disp_H_
