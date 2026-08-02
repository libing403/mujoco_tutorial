#include <cstdio>
#include <cstdlib>
#include <vector>
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
  mjData* a = mj_makeData(m);
  mjData* b = mj_makeData(m);
  a->qpos[0] = b->qpos[0] = 2.0;
  a->qpos[1] = 1.0;
  b->qpos[1] = 1.0 + 1e-9;
  mj_forward(m, a);
  mj_forward(m, b);
  int tip = mj_name2id(m, mjOBJ_SITE, "tip");
  std::vector<mjtNum> dq(m->nv);

  std::printf("initial perturbation = %.1e rad\n", b->qpos[1]-a->qpos[1]);
  std::printf("%6s %14s %14s\n", "time", "||delta q||", "tip distance");
  int next_second = 0;
  while (a->time < 15.0) {
    if (a->time + 0.5*m->opt.timestep >= next_second) {
      mj_differentiatePos(m, dq.data(), 1.0, a->qpos, b->qpos);
      mjtNum dp[3];
      mju_sub3(dp, a->site_xpos+3*tip, b->site_xpos+3*tip);
      std::printf("%6.1f %14.6e %14.6e\n",
                  a->time, mju_norm(dq.data(), m->nv), mju_norm3(dp));
      ++next_second;
    }
    mj_step(m, a);
    mj_step(m, b);
  }

  mj_deleteData(b);
  mj_deleteData(a);
  mj_deleteModel(m);
  return EXIT_SUCCESS;
}
