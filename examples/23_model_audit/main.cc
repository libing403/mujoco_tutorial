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
  int failures = 0;
  std::printf("nbody=%lld njnt=%lld nq=%lld nv=%lld nu=%lld\n",
              (long long)m->nbody, (long long)m->njnt,
              (long long)m->nq, (long long)m->nv, (long long)m->nu);
  if (m->nq - m->nv != 1) { std::printf("FAIL: expected one free-base quaternion redundancy\n"); ++failures; }

  mjtNum total_mass = 0;
  for (int b = 1; b < m->nbody; ++b) {
    mjtNum mass = m->body_mass[b];
    const mjtNum* I = m->body_inertia + 3*b;
    const char* name = mj_id2name(m, mjOBJ_BODY, b);
    bool valid = mass > 0 && I[0] > 0 && I[1] > 0 && I[2] > 0 &&
                 I[0]+I[1] >= I[2] && I[0]+I[2] >= I[1] && I[1]+I[2] >= I[0];
    std::printf("body %-12s mass=%5.2f inertia=(%.4f %.4f %.4f) %s\n",
                name, mass, I[0], I[1], I[2], valid ? "OK" : "FAIL");
    total_mass += mass;
    if (!valid) ++failures;
  }
  std::printf("total moving-body mass = %.3f kg\n", total_mass);

  for (int j = 0; j < m->njnt; ++j) {
    const char* name = mj_id2name(m, mjOBJ_JOINT, j);
    std::printf("joint %-12s type=%d qadr=%d dadr=%d",
                name, m->jnt_type[j], m->jnt_qposadr[j], m->jnt_dofadr[j]);
    if (m->jnt_limited[j]) std::printf(" range=[%.2f %.2f]", m->jnt_range[2*j], m->jnt_range[2*j+1]);
    std::printf("\n");
  }

  mj_forward(m, d);
  int torso = mj_name2id(m, mjOBJ_BODY, "torso");
  int left = mj_name2id(m, mjOBJ_SITE, "left_foot");
  int right = mj_name2id(m, mjOBJ_SITE, "right_foot");
  std::printf("torso subtree COM=(%.3f %.3f %.3f)\n",
              d->subtree_com[3*torso], d->subtree_com[3*torso+1], d->subtree_com[3*torso+2]);
  std::printf("feet z: left=%.3f right=%.3f\n", d->site_xpos[3*left+2], d->site_xpos[3*right+2]);
  std::printf("audit result: %s\n", failures ? "FAIL" : "PASS");

  mj_deleteData(d);
  mj_deleteModel(m);
  return failures ? EXIT_FAILURE : EXIT_SUCCESS;
}
