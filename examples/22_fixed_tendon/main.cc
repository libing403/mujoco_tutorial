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
  d->qpos[0] = 0.4;
  d->qpos[1] = -0.1;
  d->qvel[0] = 0.3;
  d->qvel[1] = 0.2;
  d->ctrl[0] = 1.0;
  mj_forward(m, d);

  std::printf("l = q1 - 2*q2 = %.6f (expected 0.6)\n", d->ten_length[0]);
  std::printf("ldot = v1 - 2*v2 = %.6f (expected -0.1)\n", d->ten_velocity[0]);
  std::printf("actuator scalar force = %.6f\n", d->actuator_force[0]);
  std::printf("qfrc_actuator = (%.6f, %.6f), expected (1, -2)\n",
              d->qfrc_actuator[0], d->qfrc_actuator[1]);

  mj_deleteData(d);
  mj_deleteModel(m);
  return EXIT_SUCCESS;
}
