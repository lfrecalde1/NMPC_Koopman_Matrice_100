/*
 * Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
 * Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
 * Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
 * Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
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


// standard
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/sim_interface.h"
#include "acados_sim_solver_angular_drone.h"

#define NX     ANGULAR_DRONE_NX
#define NZ     ANGULAR_DRONE_NZ
#define NU     ANGULAR_DRONE_NU
#define NP     ANGULAR_DRONE_NP


int main()
{
    int status = 0;
    sim_solver_capsule *capsule = angular_drone_acados_sim_solver_create_capsule();
    status = angular_drone_acados_sim_create(capsule);

    if (status)
    {
        printf("acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    sim_config *acados_sim_config = angular_drone_acados_get_sim_config(capsule);
    sim_in *acados_sim_in = angular_drone_acados_get_sim_in(capsule);
    sim_out *acados_sim_out = angular_drone_acados_get_sim_out(capsule);
    void *acados_sim_dims = angular_drone_acados_get_sim_dims(capsule);

    // initial condition
    double x_current[NX];
    x_current[0] = 0.0;
    x_current[1] = 0.0;
    x_current[2] = 0.0;
    x_current[3] = 0.0;
    x_current[4] = 0.0;
    x_current[5] = 0.0;
    x_current[6] = 0.0;
    x_current[7] = 0.0;
    x_current[8] = 0.0;
    x_current[9] = 0.0;
    x_current[10] = 0.0;
    x_current[11] = 0.0;
    x_current[12] = 0.0;
    x_current[13] = 0.0;
    x_current[14] = 0.0;
    x_current[15] = 0.0;
    x_current[16] = 0.0;
    x_current[17] = 0.0;
    x_current[18] = 0.0;
    x_current[19] = 0.0;
    x_current[20] = 0.0;
    x_current[21] = 0.0;
    x_current[22] = 0.0;
    x_current[23] = 0.0;

  
    x_current[0] = 0.00000007126250523015187;
    x_current[1] = -0.0000001011204631140572;
    x_current[2] = 0.006860570516437292;
    x_current[3] = 0.7912669940082956;
    x_current[4] = 0.5487368779805513;
    x_current[5] = 0.7912670166819294;
    x_current[6] = 0.548737075960909;
    x_current[7] = 0.47720732436089935;
    x_current[8] = 0.22561561966808596;
    x_current[9] = -0.000000004991600319160642;
    x_current[10] = -0.00000002887434025211633;
    x_current[11] = -2.448419529944506;
    x_current[12] = 0.000000051727273131518814;
    x_current[13] = 0.0000002456962420183118;
    x_current[14] = -0.0000002823600198098575;
    x_current[15] = 0.00000000000000000000003541199445586011;
    x_current[16] = 0.000000000000008152959285584133;
    x_current[17] = 0.0000002456962420183118;
    x_current[18] = 0.0000000000000014094283650010898;
    x_current[19] = -0.000000000000001226417440075176;
    x_current[20] = -0.0000002823600198098576;
    x_current[21] = 0.00000000000001270919661826896;
    x_current[22] = -0.00000000000001460571386612556;
    x_current[23] = -0.00000000000006937479576349804;
    
  


    // initial value for control input
    double u0[NU];
    u0[0] = 0.0;
    u0[1] = 0.0;
    u0[2] = 0.0;
    u0[3] = 0.0;

    int n_sim_steps = 3;
    // solve ocp in loop
    for (int ii = 0; ii < n_sim_steps; ii++)
    {
        sim_in_set(acados_sim_config, acados_sim_dims,
            acados_sim_in, "x", x_current);
        status = angular_drone_acados_sim_solve(capsule);

        if (status != ACADOS_SUCCESS)
        {
            printf("acados_solve() failed with status %d.\n", status);
        }

        sim_out_get(acados_sim_config, acados_sim_dims,
               acados_sim_out, "x", x_current);
        
        printf("\nx_current, %d\n", ii);
        for (int jj = 0; jj < NX; jj++)
        {
            printf("%e\n", x_current[jj]);
        }
    }

    printf("\nPerformed %d simulation steps with acados integrator successfully.\n\n", n_sim_steps);

    // free solver
    status = angular_drone_acados_sim_free(capsule);
    if (status) {
        printf("angular_drone_acados_sim_free() returned status %d. \n", status);
    }

    angular_drone_acados_sim_solver_free_capsule(capsule);

    return status;
}
