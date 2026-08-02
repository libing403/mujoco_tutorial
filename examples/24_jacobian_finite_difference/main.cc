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
  d->qpos[0] = 0.4;
  d->qpos[1] = -0.7;
  mj_forward(m, d);
  int site = mj_name2id(m, mjOBJ_SITE, "tool");

  std::vector<mjtNum> analytic(3*m->nv), rotational(3*m->nv);
  std::vector<mjtNum> finite(3*m->nv), q0(m->nq), direction(m->nv);
  mj_jacSite(m, d, analytic.data(), rotational.data(), site);
  mju_copy(q0.data(), d->qpos, m->nq);
  const mjtNum eps = 1e-6;

  for (int col = 0; col < m->nv; ++col) {
    mju_zero(direction.data(), m->nv);
    direction[col] = 1;
    mju_copy(d->qpos, q0.data(), m->nq);
    mj_integratePos(m, d->qpos, direction.data(), eps);
    mj_forward(m, d);
    mjtNum plus[3];
    mju_copy3(plus, d->site_xpos + 3*site);

    mju_copy(d->qpos, q0.data(), m->nq);
    mj_integratePos(m, d->qpos, direction.data(), -eps);
    mj_forward(m, d);
    mjtNum minus[3];
    mju_copy3(minus, d->site_xpos + 3*site);
    for (int row = 0; row < 3; ++row) {
      finite[row*m->nv+col] = (plus[row]-minus[row])/(2*eps);
    }
  }

  mjtNum max_error = 0;
  std::printf("row       analytic J                finite-difference J\n");
  for (int row = 0; row < 3; ++row) {
    std::printf(" %d   ", row);
    for (int col = 0; col < m->nv; ++col) std::printf(" % .8f", analytic[row*m->nv+col]);
    std::printf("       ");
    for (int col = 0; col < m->nv; ++col) {
      std::printf(" % .8f", finite[row*m->nv+col]);
      max_error = mju_max(max_error,
                          mju_abs(analytic[row*m->nv+col]-finite[row*m->nv+col]));
    }
    std::printf("\n");
  }
  std::printf("max absolute error = %.3g\n", max_error);

  mj_deleteData(d);
  mj_deleteModel(m);
  return max_error < 1e-7 ? EXIT_SUCCESS : EXIT_FAILURE;
}
