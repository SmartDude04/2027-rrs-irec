// /*
//  * Host-side closed-loop test of the generated embedded controller.
//  * Not part of the STM32 build.  Mirrors simulate() in airbrake_mpc.py
//  * (without sensor noise) so results can be compared line by line.
//  *
//  *   usage: test_host [cd_error=1.10] [h_burnout=900] [v_burnout=280]
//  */
// #include <stdio.h>
// #include <stdlib.h>
// #include "airbrake_mpc.h"

// int main(int argc, char **argv) {
//   const float cd_error = argc > 1 ? (float)atof(argv[1]) : 1.10f;
//   float h = argc > 2 ? (float)atof(argv[2]) : 900.0f;
//   float v = argc > 3 ? (float)atof(argv[3]) : 280.0f;

//   ab_rocket_t model = ab_default_rocket();
//   ab_rocket_t plant = ab_default_rocket();
//   plant.cd_scale = cd_error;

//   ab_mpc_t mpc;
//   airbrake_mpc_init(&mpc, &model, AB_TARGET_APOGEE_M);

//   float t = 0.0f;
//   uint32_t max_evals = 0;
//   int max_iters = 0, fallbacks = 0;
//   printf("# t h v alpha pred status iters fallback evals\n");
//   while (v > 0.0f) {
//     ab_info_t info;
//     float cmd = airbrake_mpc_step(&mpc, h, v, &info);
//     printf("%.1f %.3f %.3f %.5f %.3f %d %d %d %u\n", t, h, v, cmd,
//            info.apogee_pred_m, (int)info.solver_status, (int)info.solver_iters,
//            (int)info.used_fallback, (unsigned)info.model_evals);
//     if (info.model_evals > max_evals) max_evals = info.model_evals;
//     if (info.solver_iters > max_iters) max_iters = info.solver_iters;
//     fallbacks += info.used_fallback;

//     float hn = h, vn = v;
//     ab_rocket_step(&plant, &hn, &vn, cmd, AB_DT_MPC, 20, NULL);
//     if (vn <= 0.0f) {
//       float frac = v / (v - vn);
//       h = h + frac * (hn - h);
//       t += frac * AB_DT_MPC;
//       v = 0.0f;
//     } else {
//       h = hn; v = vn; t += AB_DT_MPC;
//     }
//   }
//   printf("# apogee %.3f target %.3f error %+.3f\n", h, AB_TARGET_APOGEE_M, h - AB_TARGET_APOGEE_M);
//   printf("# max_model_evals_per_step %u max_admm_iters %d fallbacks %d\n",
//          (unsigned)max_evals, max_iters, fallbacks);
//   return 0;
// }
