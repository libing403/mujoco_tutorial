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
  for (int i = 0; i < m->nu; ++i) d->ctrl[i] = 1.0;

  const mjtNum sample[] = {0.1, 0.5, 1.0};
  std::printf("all controls are numerically 1, but have different meanings\n");
  for (int s = 0; s < 3; ++s) {
    while (d->time < sample[s]) mj_step(m, d);
    std::printf("t=%.1f\n", d->time);
    for (int i = 0; i < 3; ++i) {
      const char* name = mj_id2name(m, mjOBJ_ACTUATOR, i);
      std::printf("  %-8s q=% .5f v=% .5f actuator_force=% .5f dof_force=% .5f\n",
                  name, d->qpos[i], d->qvel[i],
                  d->actuator_force[i], d->qfrc_actuator[i]);
    }
  }

  mj_deleteData(d);
  mj_deleteModel(m);
  return EXIT_SUCCESS;
}
