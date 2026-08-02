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
  d->qpos[0] = 0.6; d->qpos[1] = -0.9;
  mju_zero(d->qvel, m->nv);
  mju_zero(d->qacc, m->nv);
  mj_inverse(m, d);
  std::printf("保持静止所需广义力 qfrc_inverse:");
  for (int i = 0; i < m->nv; ++i) std::printf(" % .6f", d->qfrc_inverse[i]);
  std::printf("\n");
  mj_deleteData(d); mj_deleteModel(m);
  return EXIT_SUCCESS;
}
