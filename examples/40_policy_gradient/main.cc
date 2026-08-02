#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <mujoco/mujoco.h>

double rollout(const mjModel* m, const double theta[2], double gradient[2]) {
  mjData* d = mj_makeData(m); d->qpos[0] = 1.0; mj_forward(m, d);
  double S[4] = {0, 0, 0, 0};  // rows: q,v; columns: kp,kd
  gradient[0] = gradient[1] = 0;
  double cost = 0;
  const int horizon = 300;
  for (int t = 0; t < horizon; ++t) {
    double q=d->qpos[0], v=d->qvel[0], u=-theta[0]*q-theta[1]*v;
    double du[2] = {-q-theta[0]*S[0]-theta[1]*S[2],
                    -v-theta[0]*S[1]-theta[1]*S[3]};
    cost += (q*q + .1*v*v + .001*u*u)/horizon;
    for (int j = 0; j < 2; ++j)
      gradient[j] += (2*q*S[j] + .2*v*S[2+j] + .002*u*du[j])/horizon;

    d->ctrl[0] = u;
    mjtNum A[4], B[2]; mjd_transitionFD(m, d, 1e-6, 0, A, B, NULL, NULL);
    double nextS[4];
    for (int j = 0; j < 2; ++j) {
      nextS[j] = A[0]*S[j] + A[1]*S[2+j] + B[0]*du[j];
      nextS[2+j] = A[2]*S[j] + A[3]*S[2+j] + B[1]*du[j];
    }
    for (int i = 0; i < 4; ++i) S[i]=nextS[i];
    mj_step(m, d);
  }
  mj_deleteData(d); return cost;
}

int main(int argc, char** argv) {
  if (argc != 2) { std::fprintf(stderr, "用法: %s model.xml\n", argv[0]); return 1; }
  char error[1024] = {0}; mjModel* m = mj_loadXML(argv[1], NULL, error, sizeof(error));
  if (!m) { std::fprintf(stderr, "%s\n", error); return 1; }
  double theta[2] = {.1, .1};
  std::puts("epoch   cost       kp        kd");
  for (int epoch = 0; epoch <= 30; ++epoch) {
    double g[2], cost=rollout(m, theta, g);
    if (epoch % 5 == 0) std::printf("%3d   %.6f   %.5f   %.5f\n", epoch, cost, theta[0], theta[1]);
    double norm=std::hypot(g[0],g[1]), scale=norm>1 ? 1/norm : 1;
    theta[0]=mju_clip(theta[0]-2.0*scale*g[0], 0.0, 30.0);
    theta[1]=mju_clip(theta[1]-2.0*scale*g[1], 0.0, 30.0);
  }
  mj_deleteModel(m); return 0;
}
