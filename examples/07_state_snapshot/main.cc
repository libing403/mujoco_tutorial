#include <cstdio>
#include <cstdlib>
#include <mujoco/mujoco.h>

#include <vector>

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
  int spec = mjSTATE_INTEGRATION;
  int nstate = mj_stateSize(m, spec);
  std::vector<mjtNum> snapshot(nstate);

  d->ctrl[0] = 3.0;
  for (int i = 0; i < 100; ++i) mj_step(m, d);
  mj_getState(m, d, snapshot.data(), spec);
  mjtNum saved_time = d->time, saved_q0 = d->qpos[0];

  for (int i = 0; i < 100; ++i) mj_step(m, d);
  mj_setState(m, d, snapshot.data(), spec);
  mj_forward(m, d);
  std::printf("state_size=%d restored_time=%.6f restored_q0=%.9f errors=(%.3g %.3g)\n",
              nstate, d->time, d->qpos[0], d->time-saved_time, d->qpos[0]-saved_q0);

  mj_deleteData(d); mj_deleteModel(m);
  return EXIT_SUCCESS;
}
