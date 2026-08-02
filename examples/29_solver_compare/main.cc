#include <chrono>
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
    std::fprintf(stderr, "无法加载模型:\n%s\n", error);
    return EXIT_FAILURE;
  }
  const int solvers[3] = {mjSOL_PGS, mjSOL_CG, mjSOL_NEWTON};
  const char* names[3] = {"PGS", "CG", "Newton"};
  std::puts("solver   us/step   avg_iter   max_iter   min_dist   top_z");
  for (int s = 0; s < 3; ++s) {
    m->opt.solver = solvers[s];
    mjData* d = mj_makeData(m);
    long long sum_iter = 0;
    int max_iter = 0;
    auto begin = std::chrono::steady_clock::now();
    const int steps = 3000;
    for (int k = 0; k < steps; ++k) {
      mj_step(m, d);
      int it = d->solver_niter[0];
      sum_iter += it;
      max_iter = it > max_iter ? it : max_iter;
    }
    auto end = std::chrono::steady_clock::now();
    double us = std::chrono::duration<double, std::micro>(end-begin).count()/steps;
    mjtNum min_dist = 0;  // 只比较稳态；落下过程的瞬态穿入不代表稳态精度
    for (int i = 0; i < d->ncon; ++i)
      min_dist = d->contact[i].dist < min_dist ? d->contact[i].dist : min_dist;
    int top = mj_name2id(m, mjOBJ_BODY, "box5");
    std::printf("%-7s %8.3f %10.3f %10d %10.6f %8.5f\n", names[s], us,
                double(sum_iter)/steps, max_iter, min_dist, d->xpos[3*top+2]);
    mj_deleteData(d);
  }
  mj_deleteModel(m);
  return EXIT_SUCCESS;
}
