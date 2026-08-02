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

  for (int j = 0; j < m->njnt; ++j) {
    int dof = m->jnt_dofadr[j];
    const char* name = mj_id2name(m, mjOBJ_JOINT, j);
    std::printf("%-14s damping=%.2f armature=%.2f\n",
                name, m->dof_damping[dof], m->dof_armature[dof]);
  }
  for (int g = 0; g < m->ngeom; ++g) {
    const char* name = mj_id2name(m, mjOBJ_GEOM, g);
    std::printf("%-14s rgba=(%.1f %.1f %.1f %.1f)\n", name,
                m->geom_rgba[4*g], m->geom_rgba[4*g+1],
                m->geom_rgba[4*g+2], m->geom_rgba[4*g+3]);
  }

  if (!mj_saveLastXML("compiled.xml", m, error, sizeof(error))) {
    std::fprintf(stderr, "保存 compiled.xml 失败: %s\n", error);
  } else {
    std::printf("已生成 compiled.xml，可与 model.xml 对照。\n");
  }
  mj_deleteModel(m);
  return EXIT_SUCCESS;
}
