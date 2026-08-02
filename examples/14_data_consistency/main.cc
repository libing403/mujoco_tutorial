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
  mjData* d = mj_makeData(m);
  int shoulder = mj_name2id(m, mjOBJ_JOINT, "shoulder");
  int tool = mj_name2id(m, mjOBJ_SITE, "tool");
  int qadr = m->jnt_qposadr[shoulder];

  mj_forward(m, d);
  mjtNum initial[3];
  mju_copy3(initial, d->site_xpos + 3*tool);
  std::printf("initial             tool=(%.6f %.6f %.6f)\n",
              initial[0], initial[1], initial[2]);

  d->qpos[qadr] = 0.8;
  std::printf("qpos changed        tool=(%.6f %.6f %.6f)  <- stale\n",
              d->site_xpos[3*tool], d->site_xpos[3*tool+1], d->site_xpos[3*tool+2]);
  mj_forward(m, d);
  std::printf("after mj_forward    tool=(%.6f %.6f %.6f)\n",
              d->site_xpos[3*tool], d->site_xpos[3*tool+1], d->site_xpos[3*tool+2]);

  int spec = mjSTATE_INTEGRATION;
  std::vector<mjtNum> snapshot(mj_stateSize(m, spec));
  mj_getState(m, d, snapshot.data(), spec);
  mjtNum saved_q = d->qpos[qadr];
  mjtNum saved_tool[3];
  mju_copy3(saved_tool, d->site_xpos + 3*tool);
  for (int i = 0; i < 200; ++i) mj_step(m, d);
  mj_setState(m, d, snapshot.data(), spec);
  mj_forward(m, d);
  mjtNum position_error[3];
  mju_sub3(position_error, d->site_xpos + 3*tool, saved_tool);
  std::printf("restored errors     q=%.3g tool_norm=%.3g\n",
              d->qpos[qadr]-saved_q, mju_norm3(position_error));

  mj_deleteData(d);
  mj_deleteModel(m);
  return EXIT_SUCCESS;
}
