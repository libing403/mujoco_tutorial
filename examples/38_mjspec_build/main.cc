#include <cstdio>
#include <cstdlib>
#include <mujoco/mujoco.h>

int main() {
  mjSpec* spec = mj_makeSpec();
  mjsBody* body = mjs_addBody(mjs_findBody(spec, "world"), NULL);
  mjs_setName(body->element, "pendulum");

  mjsJoint* joint = mjs_addJoint(body, NULL);
  mjs_setName(joint->element, "hinge");
  joint->type = mjJNT_HINGE;
  joint->axis[0] = 0; joint->axis[1] = 1; joint->axis[2] = 0;
  joint->damping[0] = 0.05;

  mjsGeom* geom = mjs_addGeom(body, NULL);
  mjs_setName(geom->element, "rod");
  geom->type = mjGEOM_CAPSULE;
  geom->fromto[0] = 0; geom->fromto[1] = 0; geom->fromto[2] = 0;
  geom->fromto[3] = 0; geom->fromto[4] = 0; geom->fromto[5] = -0.5;
  geom->size[0] = 0.035; geom->mass = 1.0;

  mjModel* m = mj_compile(spec, NULL);
  if (!m) {
    std::fprintf(stderr, "编译失败: %s\n", mjs_getError(spec));
    mj_deleteSpec(spec); return EXIT_FAILURE;
  }
  mjData* d = mj_makeData(m); d->qpos[0] = 0.8; mj_forward(m, d);
  for (int k = 0; k < 1000; ++k) mj_step(m, d);
  std::printf("compiled: nq=%lld nv=%lld nbody=%lld, q(1s)=%.6f rad\n",
              static_cast<long long>(m->nq), static_cast<long long>(m->nv),
              static_cast<long long>(m->nbody), d->qpos[0]);

  char error[1024] = {0};
  char xml[4096];
  if (mj_saveXMLString(spec, xml, sizeof(xml), error, sizeof(error)) == 0)
    std::printf("\n--- generated MJCF ---\n%s", xml);
  else
    std::fprintf(stderr, "保存 XML 失败: %s\n", error);

  mj_deleteData(d); mj_deleteModel(m); mj_deleteSpec(spec); return 0;
}
