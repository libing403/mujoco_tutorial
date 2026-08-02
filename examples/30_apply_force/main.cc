#include <cmath>
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
  mjData* d = mj_makeData(m);
  d->qpos[0] = 0.4;
  d->qpos[1] = -0.7;
  mj_forward(m, d);

  int site = mj_name2id(m, mjOBJ_SITE, "tcp");
  int body = m->site_bodyid[site];
  const mjtNum force[3] = {3.0, -2.0, 1.0};
  const mjtNum torque[3] = {0, 0, 0};
  mjtNum from_api[2] = {0, 0};
  mj_applyFT(m, d, force, torque, d->site_xpos + 3*site, body, from_api);

  mjtNum jacp[6], jacr[6];
  mj_jacSite(m, d, jacp, jacr, site);
  mjtNum from_jacobian[2] = {0, 0};
  for (int j = 0; j < m->nv; ++j)
    for (int xyz = 0; xyz < 3; ++xyz)
      from_jacobian[j] += jacp[xyz*m->nv+j] * force[xyz];

  double max_error = 0;
  for (int j = 0; j < m->nv; ++j) {
    max_error = mju_max(max_error, std::fabs(from_api[j]-from_jacobian[j]));
    std::printf("dof %d: mj_applyFT=% .9f  J^T f=% .9f\n",
                j, from_api[j], from_jacobian[j]);
  }
  std::printf("max error = %.3g\n", max_error);
  mj_deleteData(d);
  mj_deleteModel(m);
  return EXIT_SUCCESS;
}
