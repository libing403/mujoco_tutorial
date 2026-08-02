#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <mujoco/mujoco.h>

int main(int argc, char** argv) {
  if (argc != 2) { std::fprintf(stderr, "用法: %s model.xml\n", argv[0]); return 1; }
  char error[1024] = {0};
  mjModel* m = mj_loadXML(argv[1], NULL, error, sizeof(error));
  if (!m) { std::fprintf(stderr, "%s\n", error); return 1; }
  mjData* d = mj_makeData(m);
  int tcp = mj_name2id(m, mjOBJ_SITE, "tcp");
  int target = mj_name2id(m, mjOBJ_SITE, "target");
  const double lambda2 = 0.02*0.02;

  for (int iter = 0; iter < 100; ++iter) {
    mj_forward(m, d);
    double e[2] = {d->site_xpos[3*target]-d->site_xpos[3*tcp],
                   d->site_xpos[3*target+1]-d->site_xpos[3*tcp+1]};
    if (std::hypot(e[0], e[1]) < 1e-9) break;
    mjtNum jp[6], jr[6];
    mj_jacSite(m, d, jp, jr, tcp);
    double a = jp[0]*jp[0] + jp[1]*jp[1] + lambda2;
    double b = jp[0]*jp[2] + jp[1]*jp[3];
    double c = jp[2]*jp[2] + jp[3]*jp[3] + lambda2;
    double det = a*c-b*b;
    double y[2] = {(c*e[0]-b*e[1])/det, (-b*e[0]+a*e[1])/det};
    mjtNum dq[2] = {jp[0]*y[0]+jp[2]*y[1], jp[1]*y[0]+jp[3]*y[1]};
    double norm = std::hypot(dq[0], dq[1]);
    double step = norm > 0.2 ? 0.2/norm : 1.0;
    mj_integratePos(m, d->qpos, dq, step);
  }
  mj_forward(m, d);
  double ex = d->site_xpos[3*target]-d->site_xpos[3*tcp];
  double ey = d->site_xpos[3*target+1]-d->site_xpos[3*tcp+1];
  std::printf("q = [% .9f, % .9f]\n", d->qpos[0], d->qpos[1]);
  std::printf("tcp = [%.9f, %.9f], target = [%.9f, %.9f]\n",
              d->site_xpos[3*tcp], d->site_xpos[3*tcp+1],
              d->site_xpos[3*target], d->site_xpos[3*target+1]);
  std::printf("position error = %.3g m\n", std::hypot(ex, ey));
  mj_deleteData(d); mj_deleteModel(m); return 0;
}
