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
  mj_forward(m, d);

  std::printf("nbody=%lld njnt=%lld nq=%lld nv=%lld nu=%lld nsensor=%lld\n",
              (long long)m->nbody, (long long)m->njnt, (long long)m->nq,
              (long long)m->nv, (long long)m->nu, (long long)m->nsensor);
  for (int i = 0; i < m->njnt; ++i) {
    const char* name = mj_id2name(m, mjOBJ_JOINT, i);
    std::printf("joint[%d] %-10s type=%d qposadr=%d dofadr=%d\n", i,
                name ? name : "(unnamed)", m->jnt_type[i],
                m->jnt_qposadr[i], m->jnt_dofadr[i]);
  }
  for (int i = 1; i < m->nbody; ++i) {
    const char* name = mj_id2name(m, mjOBJ_BODY, i);
    std::printf("body[%d] %-10s world_pos=(%.3f %.3f %.3f)\n", i,
                name ? name : "(unnamed)", d->xpos[3*i], d->xpos[3*i+1], d->xpos[3*i+2]);
  }

  mj_deleteData(d);
  mj_deleteModel(m);
  return EXIT_SUCCESS;
}
