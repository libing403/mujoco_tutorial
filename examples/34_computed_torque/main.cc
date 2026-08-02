#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <mujoco/mujoco.h>

struct Result { double rms, peak_torque; };

Result run(const mjModel* m, bool computed) {
  mjData* d = mj_makeData(m);
  mj_forward(m, d);
  double sum2 = 0, peak = 0;
  const int steps = 5000;
  for (int k = 0; k < steps; ++k) {
    double t = d->time, qd[2] = {0.6*std::sin(1.5*t), -0.5*std::sin(1.5*t)};
    double vd[2] = {0.9*std::cos(1.5*t), -0.75*std::cos(1.5*t)};
    double ad[2] = {-1.35*std::sin(1.5*t), 1.125*std::sin(1.5*t)};
    mjtNum tau[2];
    if (computed) {
      mjtNum desired_acc[2];
      for (int j = 0; j < 2; ++j)
        desired_acc[j] = ad[j] + 80*(qd[j]-d->qpos[j]) + 18*(vd[j]-d->qvel[j]);
      mj_mulM(m, d, tau, desired_acc);
      for (int j = 0; j < 2; ++j) tau[j] += d->qfrc_bias[j];
    } else {
      for (int j = 0; j < 2; ++j)
        tau[j] = 8*(qd[j]-d->qpos[j]) + 1.8*(vd[j]-d->qvel[j]);
    }
    for (int j = 0; j < 2; ++j) {
      d->ctrl[j] = mju_clip(tau[j], -30.0, 30.0);
      peak = mju_max(peak, std::fabs(d->ctrl[j]));
      double e = qd[j]-d->qpos[j]; sum2 += e*e;
    }
    mj_step(m, d);
  }
  Result r{std::sqrt(sum2/(2*steps)), peak};
  mj_deleteData(d); return r;
}

int main(int argc, char** argv) {
  if (argc != 2) { std::fprintf(stderr, "用法: %s model.xml\n", argv[0]); return 1; }
  char error[1024] = {0};
  mjModel* m = mj_loadXML(argv[1], NULL, error, sizeof(error));
  if (!m) { std::fprintf(stderr, "%s\n", error); return 1; }
  Result pd = run(m, false), ct = run(m, true);
  std::printf("torque PD:       RMS error = %.6f rad, peak torque = %.3f Nm\n", pd.rms, pd.peak_torque);
  std::printf("computed torque: RMS error = %.6f rad, peak torque = %.3f Nm\n", ct.rms, ct.peak_torque);
  mj_deleteModel(m); return 0;
}
