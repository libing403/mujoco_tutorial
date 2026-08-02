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
  d->qpos[0] = 0.4; d->qpos[1] = -0.7;
  d->qvel[0] = 0.2; d->qvel[1] = -0.1;
  mj_forward(m, d);

  int site = mj_name2id(m, mjOBJ_SITE, "tool");
  if (site < 0) { std::fprintf(stderr, "缺少 site: tool\n"); return EXIT_FAILURE; }
  std::vector<mjtNum> jacp(3*m->nv), jacr(3*m->nv), velocity(3);
  mj_jacSite(m, d, jacp.data(), jacr.data(), site);
  mju_mulMatVec(velocity.data(), jacp.data(), d->qvel, 3, m->nv);
  std::printf("tool pos=(%.4f %.4f %.4f) linear_velocity=J*qvel=(%.4f %.4f %.4f)\n",
              d->site_xpos[3*site], d->site_xpos[3*site+1], d->site_xpos[3*site+2],
              velocity[0], velocity[1], velocity[2]);
  for (int r = 0; r < 3; ++r) {
    std::printf("Jp row %d:", r);
    for (int c = 0; c < m->nv; ++c) std::printf(" % .5f", jacp[r*m->nv+c]);
    std::printf("\n");
  }
  mj_deleteData(d); mj_deleteModel(m);
  return EXIT_SUCCESS;
}
