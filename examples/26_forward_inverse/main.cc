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
  d->qpos[0] = 0.4; d->qpos[1] = -0.7;
  d->qvel[0] = 0.8; d->qvel[1] = -0.3;
  d->qfrc_applied[0] = 1.2;
  d->qfrc_applied[1] = -0.4;
  mj_forward(m, d);
  mjtNum acceleration[2] = {d->qacc[0], d->qacc[1]};

  mj_inverse(m, d);
  mjtNum residual[2] = {
    d->qfrc_inverse[0] - d->qfrc_applied[0],
    d->qfrc_inverse[1] - d->qfrc_applied[1]
  };
  std::printf("forward qacc = (% .9f, % .9f)\n", acceleration[0], acceleration[1]);
  std::printf("applied force = (% .9f, % .9f)\n", d->qfrc_applied[0], d->qfrc_applied[1]);
  std::printf("inverse force = (% .9f, % .9f)\n", d->qfrc_inverse[0], d->qfrc_inverse[1]);
  std::printf("residual norm = %.3g\n", mju_norm(residual, 2));

  mj_deleteData(d);
  mj_deleteModel(m);
  return mju_norm(residual, 2) < 1e-10 ? EXIT_SUCCESS : EXIT_FAILURE;
}
