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
  std::printf("nactuator=%lld nu=%lld na=%lld\n",
              (long long)m->nactuator, (long long)m->nu, (long long)m->na);
  for (int i = 0; i < m->nactuator; ++i) {
    std::printf("%-11s actadr=%d actnum=%d\n",
                mj_id2name(m, mjOBJ_ACTUATOR, i),
                m->actuator_actadr[i], m->actuator_actnum[i]);
    d->ctrl[i] = 1.0;
  }

  const mjtNum sample[] = {0.0, 0.01, 0.05, 0.10, 0.25};
  std::printf("\n%7s %11s %11s %11s %11s %11s\n",
              "time", "force_none", "act_filter", "force_filter",
              "act_exact", "force_exact");
  for (int s = 0; s < 5; ++s) {
    while (d->time + 0.5*m->opt.timestep < sample[s]) mj_step(m, d);
    mj_forward(m, d);
    int af = m->actuator_actadr[1];
    int ae = m->actuator_actadr[2];
    std::printf("%7.3f %11.6f %11.6f %11.6f %11.6f %11.6f\n",
                d->time, d->actuator_force[0], d->act[af], d->actuator_force[1],
                d->act[ae], d->actuator_force[2]);
  }

  mj_deleteData(d);
  mj_deleteModel(m);
  return EXIT_SUCCESS;
}
