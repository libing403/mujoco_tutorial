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
  if (!d) return EXIT_FAILURE;
  if (m->nq > 0) {
    d->qpos[0] = 0.7;  // 给摆一个可观察的初始偏角
    mj_forward(m, d);
  }

  while (d->time < 2.0) {
    mj_step(m, d);
    if (mju_abs(d->time * 10.0 - mju_round(d->time * 10.0)) < m->opt.timestep / 2) {
      std::printf("t=%5.2f  q=%.5f  dq=%.5f\n", d->time, d->qpos[0], d->qvel[0]);
    }
  }

  mj_deleteData(d);
  mj_deleteModel(m);
  return EXIT_SUCCESS;
}
