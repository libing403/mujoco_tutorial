#include <cmath>
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
    std::fprintf(stderr, "无法加载模型:\n%s\n", error);
    return EXIT_FAILURE;
  }
  mjData* d = mj_makeData(m);
  int box_body = mj_name2id(m, mjOBJ_BODY, "box");
  int box_geom = mj_name2id(m, mjOBJ_GEOM, "box_geom");

  while (d->time < 1.0) mj_step(m, d);       // 先让方块落稳
  for (int k = 0; k < 500; ++k) {
    d->xfrc_applied[6*box_body] = 8.0;       // 世界系 +x，低于 mu*m*g
    mj_step(m, d);
  }

  mjtNum world_force[3] = {0, 0, 0};
  mjtNum max_ratio = 0;
  int used = 0;
  for (int i = 0; i < d->ncon; ++i) {
    const mjContact& c = d->contact[i];
    if (c.geom[0] != box_geom && c.geom[1] != box_geom) continue;
    mjtNum w[6];
    mj_contactForce(m, d, i, w);
    for (int axis = 0; axis < 3; ++axis) {
      world_force[axis] += c.frame[axis] * w[0]
                         + c.frame[3+axis] * w[1]
                         + c.frame[6+axis] * w[2];
    }
    mjtNum ratio = std::sqrt(w[1]*w[1] + w[2]*w[2]) / (0.8*w[0]);
    max_ratio = mju_max(max_ratio, ratio);
    ++used;
  }

  std::printf("contacts=%d  friction_cone=%s\n", used,
              mj_isPyramidal(m) ? "pyramidal" : "elliptic");
  std::printf("world contact force = [% .6f, % .6f, % .6f] N\n",
              world_force[0], world_force[1], world_force[2]);
  std::printf("expected approximately [-8, 0, %.5f] N\n", 2*9.81);
  std::printf("max contact friction utilization = %.4f\n", max_ratio);
  std::printf("box vx = %.6g m/s\n", d->qvel[0]);

  mj_deleteData(d);
  mj_deleteModel(m);
  return EXIT_SUCCESS;
}
