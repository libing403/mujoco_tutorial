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
  d->qpos[0] = 0.8;
  mj_forward(m, d);
  mj_energyPos(m, d); mj_energyVel(m, d);
  mjtNum e0 = d->energy[0] + d->energy[1], max_drift = 0;
  while (d->time < 2.0) {
    mj_step(m, d);
    mj_energyPos(m, d); mj_energyVel(m, d);
    max_drift = mju_max(max_drift, mju_abs(d->energy[0]+d->energy[1]-e0));
  }
  std::printf("E0=%.9f Efinal=%.9f max_abs_drift=%.6g\n",
              e0, d->energy[0]+d->energy[1], max_drift);
  mj_deleteData(d); mj_deleteModel(m);
  return EXIT_SUCCESS;
}
