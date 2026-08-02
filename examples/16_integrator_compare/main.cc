#include <cstdio>
#include <cstdlib>
#include <mujoco/mujoco.h>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::fprintf(stderr, "用法: %s model.xml\n", argv[0]);
    return EXIT_FAILURE;
  }
  char error[1024] = {0};
  mjModel* m = mj_loadXML(argv[1], NULL, error, sizeof(error));
  if (!m) {
    std::fprintf(stderr, "无法加载 %s:\n%s\n", argv[1], error);
    return EXIT_FAILURE;
  }
  mjData* d = mj_makeData(m);

  const int integrator[] = {mjINT_EULER, mjINT_IMPLICITFAST, mjINT_RK4};
  const char* name[] = {"Euler", "implicitfast", "RK4"};
  const mjtNum timestep[] = {0.0005, 0.002, 0.01};
  std::printf("%-13s %8s %13s %11s %11s\n",
              "integrator", "h", "max |dE|", "q(5s)", "v(5s)");

  for (int a = 0; a < 3; ++a) {
    for (int b = 0; b < 3; ++b) {
      m->opt.integrator = integrator[a];
      m->opt.timestep = timestep[b];
      mj_resetData(m, d);
      d->qpos[0] = 0.8;
      mj_forward(m, d);
      mj_energyPos(m, d);
      mj_energyVel(m, d);
      mjtNum energy0 = d->energy[0] + d->energy[1];
      mjtNum max_drift = 0;
      while (d->time < 5.0) {
        mj_step(m, d);
        mj_energyPos(m, d);
        mj_energyVel(m, d);
        mjtNum drift = mju_abs(d->energy[0] + d->energy[1] - energy0);
        max_drift = mju_max(max_drift, drift);
      }
      std::printf("%-13s %8.4g %13.6g %11.6f %11.6f\n",
                  name[a], timestep[b], max_drift, d->qpos[0], d->qvel[0]);
    }
  }

  mj_deleteData(d);
  mj_deleteModel(m);
  return EXIT_SUCCESS;
}
