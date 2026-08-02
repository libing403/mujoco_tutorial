#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <mujoco/mujoco.h>

std::vector<double> trajectory(mjModel* m, double damping) {
  m->dof_damping[0] = damping;
  mjData* d = mj_makeData(m);
  d->qpos[0] = 1.0;
  mj_forward(m, d);
  std::vector<double> y;
  for (int k = 0; k <= 1000; ++k) {
    if (k % 10 == 0) y.push_back(d->qpos[0]);
    mj_step(m, d);
  }
  mj_deleteData(d);
  return y;
}

double cost(const std::vector<double>& a, const std::vector<double>& b) {
  double value = 0;
  for (size_t i = 0; i < a.size(); ++i) {
    double r = a[i]-b[i]; value += 0.5*r*r;
  }
  return value;
}

int main(int argc, char** argv) {
  if (argc != 2) { std::fprintf(stderr, "用法: %s model.xml\n", argv[0]); return 1; }
  char error[1024] = {0};
  mjModel* m = mj_loadXML(argv[1], NULL, error, sizeof(error));
  if (!m) { std::fprintf(stderr, "%s\n", error); return 1; }
  const std::vector<double> measured = trajectory(m, 0.35);
  double x = 1.2, mu = 1e-3;
  std::puts("iter   damping       cost          mu");
  for (int iter = 0; iter < 15; ++iter) {
    std::vector<double> r0 = trajectory(m, x);
    double f0 = cost(r0, measured), eps = 1e-4;
    std::vector<double> rp = trajectory(m, x+eps);
    double g = 0, h = 0;
    for (size_t i = 0; i < r0.size(); ++i) {
      double r = r0[i]-measured[i];
      double j = (rp[i]-r0[i])/eps;
      g += j*r; h += j*j;
    }
    std::printf("%3d   %.9f   %.6e   %.3g\n", iter, x, f0, mu);
    bool accepted = false;
    for (int trial = 0; trial < 12; ++trial) {
      double candidate = mju_clip(x-g/(h+mu), 0.0, 2.0);
      double f1 = cost(trajectory(m, candidate), measured);
      if (f1 < f0) { x = candidate; mu *= 0.3; accepted = true; break; }
      mu *= 10.0;
    }
    if (!accepted || std::fabs(g/(h+mu)) < 1e-10) break;
  }
  std::printf("estimated damping = %.9f, true damping = 0.350000000\n", x);
  mj_deleteModel(m); return 0;
}
