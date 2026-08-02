#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <mujoco/mujoco.h>

void dare(const double A[4], const double B[2], double K[2]) {
  const double Q[4] = {20, 0, 0, 1}, R = 0.1;
  double P[4] = {20, 0, 0, 1};
  for (int iter = 0; iter < 100000; ++iter) {
    double PB[2] = {P[0]*B[0]+P[1]*B[1], P[2]*B[0]+P[3]*B[1]};
    double s = R+B[0]*PB[0]+B[1]*PB[1];
    double PA[4] = {P[0]*A[0]+P[1]*A[2], P[0]*A[1]+P[1]*A[3],
                    P[2]*A[0]+P[3]*A[2], P[2]*A[1]+P[3]*A[3]};
    double btpa[2] = {B[0]*PA[0]+B[1]*PA[2], B[0]*PA[1]+B[1]*PA[3]};
    double next[4];
    for (int i = 0; i < 2; ++i) for (int j = 0; j < 2; ++j) {
      double atpa = A[0*2+i]*PA[0*2+j] + A[1*2+i]*PA[1*2+j];
      next[2*i+j] = Q[2*i+j] + atpa - btpa[i]*btpa[j]/s;
    }
    double change = 0;
    for (int i = 0; i < 4; ++i) { change = mju_max(change, std::fabs(next[i]-P[i])); P[i]=next[i]; }
    if (change < 1e-12) break;
  }
  double PB[2] = {P[0]*B[0]+P[1]*B[1], P[2]*B[0]+P[3]*B[1]};
  double s = R+B[0]*PB[0]+B[1]*PB[1];
  K[0] = (B[0]*(P[0]*A[0]+P[1]*A[2]) + B[1]*(P[2]*A[0]+P[3]*A[2]))/s;
  K[1] = (B[0]*(P[0]*A[1]+P[1]*A[3]) + B[1]*(P[2]*A[1]+P[3]*A[3]))/s;
}

double simulate(const mjModel* m, const double K[2], bool controlled) {
  mjData* d = mj_makeData(m); d->qpos[0] = 0.15; mj_forward(m, d);
  for (int k = 0; k < 600; ++k) {
    if (controlled) d->ctrl[0] = mju_clip(-K[0]*d->qpos[0]-K[1]*d->qvel[0], -10.0, 10.0);
    mj_step(m, d);
  }
  double angle = d->qpos[0]; mj_deleteData(d); return angle;
}

int main(int argc, char** argv) {
  if (argc != 2) { std::fprintf(stderr, "用法: %s model.xml\n", argv[0]); return 1; }
  char error[1024] = {0}; mjModel* m = mj_loadXML(argv[1], NULL, error, sizeof(error));
  if (!m) { std::fprintf(stderr, "%s\n", error); return 1; }
  mjData* d = mj_makeData(m); mj_forward(m, d);
  mjtNum A[4], B[2]; mjd_transitionFD(m, d, 1e-6, 1, A, B, NULL, NULL);
  double K[2]; dare(A, B, K);
  std::printf("K = [%.6f, %.6f]\n", K[0], K[1]);
  std::printf("after 3 s: uncontrolled angle = %.6f rad\n", simulate(m, K, false));
  std::printf("after 3 s: LQR angle          = %.9f rad\n", simulate(m, K, true));
  mj_deleteData(d); mj_deleteModel(m); return 0;
}
