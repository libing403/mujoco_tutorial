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
  int body = mj_name2id(m, mjOBJ_BODY, "ball");
  std::printf("ball body mass = %.3f kg\n", m->body_mass[body]);
  for (int g = 0; g < m->ngeom; ++g) {
    const char* name = mj_id2name(m, mjOBJ_GEOM, g);
    std::printf("%-15s group=%d contype=%d conaffinity=%d\n",
                name, m->geom_group[g], m->geom_contype[g], m->geom_conaffinity[g]);
  }

  while (d->time < 1.0) mj_step(m, d);
  std::printf("contacts after falling: %d\n", d->ncon);
  for (int i = 0; i < d->ncon; ++i) {
    int g1 = d->contact[i].geom[0];
    int g2 = d->contact[i].geom[1];
    std::printf("  %s <-> %s\n",
                mj_id2name(m, mjOBJ_GEOM, g1),
                mj_id2name(m, mjOBJ_GEOM, g2));
  }

  mj_deleteData(d);
  mj_deleteModel(m);
  return EXIT_SUCCESS;
}
