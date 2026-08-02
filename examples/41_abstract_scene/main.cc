#include <cstdio>
#include <cstdlib>
#include <mujoco/mujoco.h>

int main(int argc, char** argv) {
  if (argc != 2) { std::fprintf(stderr, "用法: %s model.xml\n", argv[0]); return 1; }
  char error[1024] = {0}; mjModel* m = mj_loadXML(argv[1], NULL, error, sizeof(error));
  if (!m) { std::fprintf(stderr, "%s\n", error); return 1; }
  mjData* d = mj_makeData(m);
  for (int k = 0; k < 500; ++k) mj_step(m, d);

  mjvOption opt; mjv_defaultOption(&opt); opt.flags[mjVIS_CONTACTPOINT] = 1;
  mjvCamera cam; mjv_defaultCamera(&cam);
  cam.type = mjCAMERA_TRACKING; cam.trackbodyid = mj_name2id(m, mjOBJ_BODY, "ball");
  cam.distance = 2; cam.azimuth = 90; cam.elevation = -20;
  mjvScene scene; mjv_defaultScene(&scene); mjv_makeScene(m, &scene, 200);
  mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scene);

  int categories[3] = {0, 0, 0};
  for (int i = 0; i < scene.ngeom; ++i) {
    if (scene.geoms[i].category == mjCAT_STATIC) ++categories[0];
    else if (scene.geoms[i].category == mjCAT_DYNAMIC) ++categories[1];
    else if (scene.geoms[i].category == mjCAT_DECOR) ++categories[2];
  }
  mjtNum head[3], forward[3], up[3]; mjv_cameraInModel(head, forward, up, &scene);
  std::printf("scene geoms=%d (static=%d dynamic=%d decor=%d), contacts=%d\n",
              scene.ngeom, categories[0], categories[1], categories[2], d->ncon);
  std::printf("camera head=[%.4f %.4f %.4f], forward=[%.4f %.4f %.4f]\n",
              head[0], head[1], head[2], forward[0], forward[1], forward[2]);
  mjv_freeScene(&scene); mj_deleteData(d); mj_deleteModel(m); return 0;
}
