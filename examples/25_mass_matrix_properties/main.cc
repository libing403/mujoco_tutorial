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
  d->qpos[0] = 0.4; d->qpos[1] = -0.7;
  d->qvel[0] = 0.8; d->qvel[1] = -0.3;
  mj_forward(m, d);

  std::vector<mjtNum> M(m->nv*m->nv);
  mj_fullM(m, d, M.data());
  std::printf("M(q):\n");
  for (int r = 0; r < m->nv; ++r) {
    for (int c = 0; c < m->nv; ++c) std::printf(" % .9f", M[r*m->nv+c]);
    std::printf("\n");
  }
  mjtNum symmetry = mju_abs(M[1]-M[2]);
  mjtNum determinant = M[0]*M[3]-M[1]*M[2];

  std::vector<mjtNum> Mv(m->nv);
  mj_mulM(m, d, Mv.data(), d->qvel);
  mjtNum kinetic_matrix = 0.5*mju_dot(d->qvel, Mv.data(), m->nv);
  mj_energyVel(m, d);

  mjtNum rhs[2] = {1.2, -0.4};
  mjtNum solution[2], reconstructed[2];
  mj_solveM(m, d, solution, rhs, 1);
  mj_mulM(m, d, reconstructed, solution);
  mjtNum residual[2] = {reconstructed[0]-rhs[0], reconstructed[1]-rhs[1]};

  std::printf("symmetry error = %.3g, determinant = %.9f\n", symmetry, determinant);
  std::printf("kinetic 0.5*vT*M*v = %.12f, mj_energyVel = %.12f, error = %.3g\n",
              kinetic_matrix, d->energy[1], mju_abs(kinetic_matrix-d->energy[1]));
  std::printf("solve residual norm = %.3g\n", mju_norm(residual, 2));

  bool ok = symmetry < 1e-12 && M[0] > 0 && determinant > 0 &&
            mju_abs(kinetic_matrix-d->energy[1]) < 1e-12 && mju_norm(residual, 2) < 1e-12;
  mj_deleteData(d);
  mj_deleteModel(m);
  return ok ? EXIT_SUCCESS : EXIT_FAILURE;
}
