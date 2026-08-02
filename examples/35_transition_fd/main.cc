#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <mujoco/mujoco.h>

void setpoint(const mjModel* m, mjData* d, double dq, double dv, double du) {
  mj_resetData(m, d);
  d->qpos[0] = 0.3+dq; d->qvel[0] = -0.2+dv; d->ctrl[0] = 0.4+du;
  mj_forward(m, d);
}

int main(int argc, char** argv) {
  if (argc != 2) { std::fprintf(stderr, "用法: %s model.xml\n", argv[0]); return 1; }
  char error[1024] = {0};
  mjModel* m = mj_loadXML(argv[1], NULL, error, sizeof(error));
  if (!m) { std::fprintf(stderr, "%s\n", error); return 1; }
  mjData* linear = mj_makeData(m);
  setpoint(m, linear, 0, 0, 0);
  mjtNum A[4], B[2];
  mjd_transitionFD(m, linear, 1e-6, 1, A, B, NULL, NULL);

  const double dx[2] = {1e-4, -2e-4}, du = 1.5e-4;
  mjData* base = mj_makeData(m); setpoint(m, base, 0, 0, 0); mj_step(m, base);
  mjData* pert = mj_makeData(m); setpoint(m, pert, dx[0], dx[1], du); mj_step(m, pert);
  double actual[2] = {pert->qpos[0]-base->qpos[0], pert->qvel[0]-base->qvel[0]};
  double predicted[2] = {A[0]*dx[0]+A[1]*dx[1]+B[0]*du,
                         A[2]*dx[0]+A[3]*dx[1]+B[1]*du};
  std::printf("A = [[%.9f, %.9f], [%.9f, %.9f]]\n", A[0], A[1], A[2], A[3]);
  std::printf("B = [%.9f, %.9f]^T\n", B[0], B[1]);
  std::printf("actual delta    = [% .9g, % .9g]\n", actual[0], actual[1]);
  std::printf("linear predicted= [% .9g, % .9g]\n", predicted[0], predicted[1]);
  std::printf("prediction error norm = %.3g\n",
              std::hypot(actual[0]-predicted[0], actual[1]-predicted[1]));
  mj_deleteData(pert); mj_deleteData(base); mj_deleteData(linear); mj_deleteModel(m); return 0;
}
