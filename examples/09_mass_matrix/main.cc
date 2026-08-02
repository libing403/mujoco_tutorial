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
  d->qpos[0] = 0.3; d->qpos[1] = -0.5;
  mj_forward(m, d);
  std::vector<mjtNum> M(m->nv*m->nv);
  mj_fullM(m, d, M.data());
  std::printf("dense M(q), %lld x %lld:\n", (long long)m->nv, (long long)m->nv);
  for (int r = 0; r < m->nv; ++r) {
    for (int c = 0; c < m->nv; ++c) std::printf(" % .8f", M[r*m->nv+c]);
    std::printf("\n");
  }
  mjtNum asym = 0;
  for (int r = 0; r < m->nv; ++r)
    for (int c = 0; c < m->nv; ++c)
      asym = mju_max(asym, mju_abs(M[r*m->nv+c]-M[c*m->nv+r]));
  std::printf("max symmetry error = %.3g\n", asym);
  mj_deleteData(d); mj_deleteModel(m);
  return asym < 1e-12 ? EXIT_SUCCESS : EXIT_FAILURE;
}
