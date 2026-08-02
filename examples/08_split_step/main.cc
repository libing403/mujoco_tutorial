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
  const mjtNum target[2] = {0.5, -0.8};

  while (d->time < 1.0) {
    mj_step1(m, d);  // 此时最新的运动学和速度派生量可供控制器读取
    for (int i = 0; i < m->nu && i < 2; ++i) {
      d->ctrl[i] = 60*(target[i]-d->qpos[i]) - 6*d->qvel[i];
    }
    mj_step2(m, d);
  }
  std::printf("split-step final q=(%.5f %.5f), time=%.3f\n",
              d->qpos[0], d->qpos[1], d->time);
  mj_deleteData(d); mj_deleteModel(m);
  return EXIT_SUCCESS;
}
