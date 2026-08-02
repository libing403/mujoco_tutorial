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
  for (int k = 0; k < 1000; ++k) mj_step(m, d);

  std::printf("time=%.3f contacts=%d sensor_data=%lld\n", d->time, d->ncon,
              (long long)m->nsensordata);
  for (int i = 0; i < m->nsensor; ++i) {
    const char* name = mj_id2name(m, mjOBJ_SENSOR, i);
    int adr = m->sensor_adr[i], dim = m->sensor_dim[i];
    std::printf("%-12s =", name ? name : "(unnamed)");
    for (int j = 0; j < dim; ++j) std::printf(" % .5f", d->sensordata[adr+j]);
    std::printf("\n");
  }
  if (d->ncon) {
    mjtNum wrench[6];
    mj_contactForce(m, d, 0, wrench);
    std::printf("contact[0] frame force=(%.4f %.4f %.4f)\n", wrench[0], wrench[1], wrench[2]);
  }
  mj_deleteData(d);
  mj_deleteModel(m);
  return EXIT_SUCCESS;
}
