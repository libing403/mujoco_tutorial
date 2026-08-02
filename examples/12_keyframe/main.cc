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
  for (int key = 0; key < m->nkey; ++key) {
    mj_resetDataKeyframe(m, d, key);
    mj_forward(m, d);
    const char* name = mj_id2name(m, mjOBJ_KEY, key);
    std::printf("key[%d] %-8s time=%.2f q=(%.4f %.4f) tool=(%.4f %.4f %.4f)\n",
                key, name ? name : "(unnamed)", d->time, d->qpos[0], d->qpos[1],
                d->site_xpos[0], d->site_xpos[1], d->site_xpos[2]);
  }
  mj_deleteData(d); mj_deleteModel(m);
  return EXIT_SUCCESS;
}
