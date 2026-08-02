#include <cstdio>
#include <cstdlib>
#include <limits>
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
  int soft_geom = mj_name2id(m, mjOBJ_GEOM, "soft_ball");
  int hard_geom = mj_name2id(m, mjOBJ_GEOM, "hard_ball");
  int soft_body = mj_name2id(m, mjOBJ_BODY, "soft_body");
  int hard_body = mj_name2id(m, mjOBJ_BODY, "hard_body");
  mjtNum min_dist[2] = {std::numeric_limits<mjtNum>::infinity(),
                        std::numeric_limits<mjtNum>::infinity()};
  mjtNum max_force[2] = {0, 0};

  while (d->time < 2.0) {
    mj_step(m, d);
    for (int i = 0; i < d->ncon; ++i) {
      int g1 = d->contact[i].geom[0], g2 = d->contact[i].geom[1];
      int which = (g1 == soft_geom || g2 == soft_geom) ? 0 :
                  (g1 == hard_geom || g2 == hard_geom) ? 1 : -1;
      if (which >= 0) {
        mjtNum wrench[6];
        mj_contactForce(m, d, i, wrench);
        min_dist[which] = mju_min(min_dist[which], d->contact[i].dist);
        max_force[which] = mju_max(max_force[which], wrench[0]);
      }
    }
  }

  std::printf("type   min_contact_dist   peak_normal_force   final_center_z   final_vz\n");
  std::printf("soft   % .9f       % .6f          %.9f      % .3g\n",
              min_dist[0], max_force[0], d->xpos[3*soft_body+2], d->qvel[2]);
  std::printf("hard   % .9f       % .6f          %.9f      % .3g\n",
              min_dist[1], max_force[1], d->xpos[3*hard_body+2], d->qvel[8]);

  mj_deleteData(d);
  mj_deleteModel(m);
  return EXIT_SUCCESS;
}
