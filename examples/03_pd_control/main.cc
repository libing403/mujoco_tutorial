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
  const mjtNum target[2] = {0.8, -1.1};
  const mjtNum kp = 80.0, kd = 8.0;

  while (d->time < 3.0) {
    for (int i = 0; i < 2; ++i) {
      d->ctrl[i] = kp * (target[i] - d->qpos[i]) - kd * d->qvel[i];
    }
    mj_step(m, d);
  }
  std::printf("target=(%.3f %.3f) final=(%.3f %.3f) error=(%.4f %.4f)\n",
              target[0], target[1], d->qpos[0], d->qpos[1],
              target[0]-d->qpos[0], target[1]-d->qpos[1]);
  mj_deleteData(d);
  mj_deleteModel(m);
  return EXIT_SUCCESS;
}
