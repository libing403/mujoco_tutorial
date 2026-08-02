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

  const char* type_name[] = {"free", "ball", "slide", "hinge"};
  std::printf("njnt=%lld, nq=%lld, nv=%lld\n",
              (long long)m->njnt, (long long)m->nq, (long long)m->nv);
  for (int j = 0; j < m->njnt; ++j) {
    const char* name = mj_id2name(m, mjOBJ_JOINT, j);
    std::printf("%-6s type=%-5s qposadr=%d dofadr=%d\n",
                name, type_name[m->jnt_type[j]],
                m->jnt_qposadr[j], m->jnt_dofadr[j]);
  }

  std::vector<mjtNum> velocity(m->nv, 0.1);
  mj_integratePos(m, d->qpos, velocity.data(), 0.01);
  int ball = mj_name2id(m, mjOBJ_JOINT, "ball");
  int free_joint = mj_name2id(m, mjOBJ_JOINT, "free");
  int bq = m->jnt_qposadr[ball];
  int fq = m->jnt_qposadr[free_joint] + 3;
  std::printf("ball quaternion norm = %.12f\n", mju_norm(d->qpos+bq, 4));
  std::printf("free quaternion norm = %.12f\n", mju_norm(d->qpos+fq, 4));

  mj_deleteData(d);
  mj_deleteModel(m);
  return EXIT_SUCCESS;
}
